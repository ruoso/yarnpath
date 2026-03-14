#include <gtest/gtest.h>
#include <stitch_shape/stitch_shape.hpp>
#include <cmath>

using namespace yarnpath;

class StitchShapeTest : public ::testing::Test {
protected:
    YarnProperties yarn;
    Gauge gauge;

    void SetUp() override {
        yarn = YarnProperties::worsted();
        gauge = Gauge::worsted();
    }
};

TEST_F(StitchShapeTest, KnitShape) {
    auto params = compute_stitch_shape(yarn, gauge,
        LoopOrientation::Front, WrapDirection::None, WorkType::Worked);

    // Front orientation → positive z_bulge (forward crossing)
    EXPECT_GT(params.z_bulge, 0.0f);
    EXPECT_FLOAT_EQ(params.z_bulge_factor, 1.0f);
    EXPECT_TRUE(params.symmetric_exit);
}

TEST_F(StitchShapeTest, PurlShape) {
    auto params = compute_stitch_shape(yarn, gauge,
        LoopOrientation::Back, WrapDirection::None, WorkType::Worked);

    // Back orientation → negative z_bulge (backward crossing)
    EXPECT_LT(params.z_bulge, 0.0f);
    EXPECT_FLOAT_EQ(params.z_bulge_factor, -1.0f);
    EXPECT_TRUE(params.symmetric_exit);
}

TEST_F(StitchShapeTest, PurlMagnitudeMatchesKnit) {
    auto knit = compute_stitch_shape(yarn, gauge,
        LoopOrientation::Front, WrapDirection::None, WorkType::Worked);
    auto purl = compute_stitch_shape(yarn, gauge,
        LoopOrientation::Back, WrapDirection::None, WorkType::Worked);

    EXPECT_FLOAT_EQ(std::abs(knit.z_bulge), std::abs(purl.z_bulge));
    EXPECT_FLOAT_EQ(knit.loop_width, purl.loop_width);
    EXPECT_FLOAT_EQ(knit.loop_height, purl.loop_height);
}

TEST_F(StitchShapeTest, NeutralShape) {
    auto params = compute_stitch_shape(yarn, gauge,
        LoopOrientation::Neutral, WrapDirection::None, WorkType::Worked);

    // Neutral → positive z_bulge (factor 0.5), reduced apex
    EXPECT_GT(params.z_bulge, 0.0f);
    EXPECT_FLOAT_EQ(params.z_bulge_factor, 0.5f);
    EXPECT_FLOAT_EQ(params.apex_height_factor, 0.8f);
    EXPECT_TRUE(params.symmetric_exit);
}

TEST_F(StitchShapeTest, K2togLean) {
    auto params = compute_stitch_shape(yarn, gauge,
        LoopOrientation::Front, WrapDirection::Clockwise, WorkType::Worked);

    // Clockwise → right lean (positive apex_lean_x)
    EXPECT_GT(params.apex_lean_x, 0.0f);
}

TEST_F(StitchShapeTest, SSKLean) {
    auto params = compute_stitch_shape(yarn, gauge,
        LoopOrientation::Front, WrapDirection::CounterClockwise, WorkType::Worked);

    // CounterClockwise → left lean (negative apex_lean_x)
    EXPECT_LT(params.apex_lean_x, 0.0f);
}

TEST_F(StitchShapeTest, SSKLeanMagnitudeMatchesK2tog) {
    auto k2tog = compute_stitch_shape(yarn, gauge,
        LoopOrientation::Front, WrapDirection::Clockwise, WorkType::Worked);
    auto ssk = compute_stitch_shape(yarn, gauge,
        LoopOrientation::Front, WrapDirection::CounterClockwise, WorkType::Worked);

    EXPECT_FLOAT_EQ(std::abs(k2tog.apex_lean_x), std::abs(ssk.apex_lean_x));
}

TEST_F(StitchShapeTest, SlipShape) {
    auto slip = compute_stitch_shape(yarn, gauge,
        LoopOrientation::Front, WrapDirection::None, WorkType::Transferred);

    // Slip: shallow z_bulge (factor *= 0.3), tall (height_multiplier 1.8, apex_height_factor 2.0)
    // z_bulge_factor starts at 1.0 for Front, then *= 0.3 → 0.3
    EXPECT_FLOAT_EQ(slip.z_bulge_factor, 0.3f);
    EXPECT_FLOAT_EQ(slip.height_multiplier, 1.8f);
    EXPECT_FLOAT_EQ(slip.apex_height_factor, 2.0f);

    // z_bulge should be small compared to a regular knit
    auto knit = compute_stitch_shape(yarn, gauge,
        LoopOrientation::Front, WrapDirection::None, WorkType::Worked);
    EXPECT_LT(std::abs(slip.z_bulge), std::abs(knit.z_bulge));
}

TEST_F(StitchShapeTest, YarnOverShape) {
    auto yo = compute_stitch_shape(yarn, gauge,
        LoopOrientation::Neutral, WrapDirection::None, WorkType::Created);

    // YarnOver: Neutral + Created → width_multiplier 1.4, apex_height_factor 1.2
    EXPECT_FLOAT_EQ(yo.width_multiplier, 1.4f);
    EXPECT_FLOAT_EQ(yo.apex_height_factor, 1.2f);
}

TEST_F(StitchShapeTest, BoundingHalfExtent) {
    auto knit = compute_stitch_shape(yarn, gauge,
        LoopOrientation::Front, WrapDirection::None, WorkType::Worked);

    Vec3 half = knit.bounding_half_extent(yarn.compressed_radius);

    // Manual calculation:
    //   x = loop_width * 0.5
    //   y = loop_height * 0.5
    //   z = |z_bulge| + compressed_radius
    EXPECT_FLOAT_EQ(half.x, knit.loop_width * 0.5f);
    EXPECT_FLOAT_EQ(half.y, knit.loop_height * 0.5f);
    EXPECT_FLOAT_EQ(half.z, std::abs(knit.z_bulge) + yarn.compressed_radius);
}

TEST_F(StitchShapeTest, BoundingHalfExtentPurl) {
    auto purl = compute_stitch_shape(yarn, gauge,
        LoopOrientation::Back, WrapDirection::None, WorkType::Worked);

    Vec3 half = purl.bounding_half_extent(yarn.compressed_radius);

    // Same formula, but z_bulge is negative → abs should give same Z extent as knit
    auto knit = compute_stitch_shape(yarn, gauge,
        LoopOrientation::Front, WrapDirection::None, WorkType::Worked);
    Vec3 knit_half = knit.bounding_half_extent(yarn.compressed_radius);

    EXPECT_FLOAT_EQ(half.z, knit_half.z);
}

TEST_F(StitchShapeTest, DerivedDimensions) {
    // Verify z_bulge = compressed_diameter * 2 * z_bulge_factor
    auto knit = compute_stitch_shape(yarn, gauge,
        LoopOrientation::Front, WrapDirection::None, WorkType::Worked);

    float expected_z_bulge = yarn.compressed_radius * 2.0f * 2.0f * 1.0f;
    EXPECT_FLOAT_EQ(knit.z_bulge, expected_z_bulge);

    // Verify loop_width and loop_height come from LoopDimensions
    auto dims = LoopDimensions::calculate(yarn, gauge);
    EXPECT_FLOAT_EQ(knit.loop_width, dims.loop_width * 1.0f);  // width_multiplier = 1.0
    EXPECT_FLOAT_EQ(knit.loop_height, dims.loop_height * 1.0f);  // height_multiplier = 1.0
}
