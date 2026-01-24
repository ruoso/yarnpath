#include <gtest/gtest.h>
#include "geometry.hpp"

using namespace yarnpath;

// ============================================
// CubicBezier Tests
// ============================================

TEST(CubicBezierTest, Evaluate) {
    CubicBezier curve(
        Vec3(0.0f, 0.0f, 0.0f),
        Vec3(0.0f, 1.0f, 0.0f),
        Vec3(1.0f, 1.0f, 0.0f),
        Vec3(1.0f, 0.0f, 0.0f)
    );

    Vec3 start = curve.evaluate(0.0f);
    EXPECT_FLOAT_EQ(start.x, 0.0f);
    EXPECT_FLOAT_EQ(start.y, 0.0f);

    Vec3 end = curve.evaluate(1.0f);
    EXPECT_FLOAT_EQ(end.x, 1.0f);
    EXPECT_FLOAT_EQ(end.y, 0.0f);
}

TEST(CubicBezierTest, ArcLength) {
    // Straight line should have arc length equal to distance
    CubicBezier line(
        Vec3(0.0f, 0.0f, 0.0f),
        Vec3(1.0f, 0.0f, 0.0f),
        Vec3(2.0f, 0.0f, 0.0f),
        Vec3(3.0f, 0.0f, 0.0f)
    );

    float arc_len = line.arc_length(100);
    EXPECT_NEAR(arc_len, 3.0f, 0.01f);
}

TEST(CubicBezierTest, FromHermite) {
    Vec3 p0(0.0f, 0.0f, 0.0f);
    Vec3 t0(3.0f, 0.0f, 0.0f);
    Vec3 p1(3.0f, 0.0f, 0.0f);
    Vec3 t1(3.0f, 0.0f, 0.0f);

    CubicBezier curve = CubicBezier::from_hermite(p0, t0, p1, t1);

    // Check endpoints match
    Vec3 start = curve.evaluate(0.0f);
    Vec3 end = curve.evaluate(1.0f);

    EXPECT_NEAR(start.x, p0.x, 0.001f);
    EXPECT_NEAR(start.y, p0.y, 0.001f);
    EXPECT_NEAR(end.x, p1.x, 0.001f);
    EXPECT_NEAR(end.y, p1.y, 0.001f);
}

TEST(CubicBezierTest, Split) {
    CubicBezier curve(
        Vec3(0.0f, 0.0f, 0.0f),
        Vec3(0.0f, 1.0f, 0.0f),
        Vec3(1.0f, 1.0f, 0.0f),
        Vec3(1.0f, 0.0f, 0.0f)
    );

    auto [left, right] = curve.split(0.5f);

    Vec3 mid_original = curve.evaluate(0.5f);
    Vec3 left_end = left.evaluate(1.0f);
    Vec3 right_start = right.evaluate(0.0f);

    EXPECT_NEAR(left_end.x, mid_original.x, 0.001f);
    EXPECT_NEAR(left_end.y, mid_original.y, 0.001f);
    EXPECT_NEAR(right_start.x, mid_original.x, 0.001f);
    EXPECT_NEAR(right_start.y, mid_original.y, 0.001f);
}

// ============================================
// BezierSpline Tests
// ============================================

TEST(BezierSplineTest, Empty) {
    BezierSpline spline;
    EXPECT_TRUE(spline.empty());
    EXPECT_EQ(spline.segment_count(), 0u);
}

TEST(BezierSplineTest, AddSegment) {
    BezierSpline spline;
    CubicBezier seg(
        Vec3(0.0f, 0.0f, 0.0f),
        Vec3(1.0f, 0.0f, 0.0f),
        Vec3(2.0f, 0.0f, 0.0f),
        Vec3(3.0f, 0.0f, 0.0f)
    );
    spline.add_segment(seg);

    EXPECT_FALSE(spline.empty());
    EXPECT_EQ(spline.segment_count(), 1u);
}

TEST(BezierSplineTest, ToPolyline) {
    BezierSpline spline;
    CubicBezier seg(
        Vec3(0.0f, 0.0f, 0.0f),
        Vec3(1.0f, 0.0f, 0.0f),
        Vec3(2.0f, 0.0f, 0.0f),
        Vec3(3.0f, 0.0f, 0.0f)
    );
    spline.add_segment(seg);

    auto points = spline.to_polyline_fixed(10);
    EXPECT_EQ(points.size(), 11u);  // Start + 10 samples
}
