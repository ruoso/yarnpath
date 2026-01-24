#include <gtest/gtest.h>
#include "geometry.hpp"

using namespace yarnpath;

// ============================================
// YarnProperties Tests
// ============================================

TEST(YarnPropertiesTest, Defaults) {
    YarnProperties yarn;
    EXPECT_FLOAT_EQ(yarn.radius, 1.0f);
    EXPECT_FLOAT_EQ(yarn.min_bend_radius, 3.0f);
}

TEST(YarnPropertiesTest, DerivedProperties) {
    YarnProperties yarn;
    yarn.radius = 2.0f;
    yarn.min_bend_radius = 6.0f;

    EXPECT_FLOAT_EQ(yarn.min_clearance(), 4.0f);
    EXPECT_NEAR(yarn.max_curvature(), 1.0f / 6.0f, 0.001f);
}

TEST(YarnPropertiesTest, Presets) {
    auto fingering = YarnProperties::fingering();
    EXPECT_LT(fingering.radius, 1.0f);

    auto bulky = YarnProperties::bulky();
    EXPECT_GT(bulky.radius, 1.0f);
}

// ============================================
// Gauge Tests
// ============================================

TEST(GaugeTest, Defaults) {
    Gauge gauge;
    EXPECT_FLOAT_EQ(gauge.stitches_per_unit, 4.0f);
    EXPECT_FLOAT_EQ(gauge.rows_per_unit, 5.0f);
}

TEST(GaugeTest, StitchWidth) {
    Gauge gauge;
    gauge.stitches_per_unit = 5.0f;
    EXPECT_FLOAT_EQ(gauge.stitch_width(), 0.2f);
}

TEST(GaugeTest, RowHeight) {
    Gauge gauge;
    gauge.rows_per_unit = 4.0f;
    EXPECT_FLOAT_EQ(gauge.row_height(), 0.25f);
}

TEST(GaugeTest, Conversion) {
    Gauge gauge;
    gauge.stitches_per_unit = 4.0f;
    gauge.rows_per_unit = 5.0f;

    EXPECT_FLOAT_EQ(gauge.stitch_to_u(4.0f), 1.0f);
    EXPECT_FLOAT_EQ(gauge.row_to_v(5.0f), 1.0f);
}
