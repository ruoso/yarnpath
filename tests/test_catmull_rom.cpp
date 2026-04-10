#include <gtest/gtest.h>
#include <math/catmull_rom_spline.hpp>
#include <cmath>

using namespace yarnpath;

// --- Basic construction ---

TEST(CatmullRomSpline, EmptySpline) {
    CatmullRomSpline spline;
    EXPECT_TRUE(spline.empty());
    EXPECT_EQ(spline.segment_count(), 0u);
    EXPECT_EQ(spline.waypoint_count(), 0u);
}

TEST(CatmullRomSpline, SingleWaypoint) {
    CatmullRomSpline spline;
    spline.add_waypoint(Vec3(1, 2, 3));
    EXPECT_TRUE(spline.empty());  // need >= 2 for a valid spline
    EXPECT_EQ(spline.waypoint_count(), 1u);
    EXPECT_EQ(spline.segment_count(), 0u);
}

TEST(CatmullRomSpline, TwoWaypoints) {
    CatmullRomSpline spline;
    spline.add_waypoint(Vec3(0, 0, 0));
    spline.add_waypoint(Vec3(1, 0, 0));
    EXPECT_FALSE(spline.empty());
    EXPECT_EQ(spline.segment_count(), 1u);
    EXPECT_EQ(spline.waypoint_count(), 2u);
}

TEST(CatmullRomSpline, AddWaypoints) {
    CatmullRomSpline spline;
    spline.add_waypoints({{0,0,0}, {1,0,0}, {2,1,0}, {3,0,0}});
    EXPECT_EQ(spline.waypoint_count(), 4u);
    EXPECT_EQ(spline.segment_count(), 3u);
}

// --- Endpoint access ---

TEST(CatmullRomSpline, StartEnd) {
    CatmullRomSpline spline;
    spline.add_waypoints({{1,2,3}, {4,5,6}, {7,8,9}});
    EXPECT_EQ(spline.start(), Vec3(1,2,3));
    EXPECT_EQ(spline.end(), Vec3(7,8,9));
}

// --- Interpolation ---

TEST(CatmullRomSpline, InterpolatesWaypoints) {
    CatmullRomSpline spline;
    spline.add_waypoints({{0,0,0}, {1,0,0}, {2,1,0}, {3,0,0}});

    // At t=0, should be at first waypoint
    Vec3 p0 = spline.evaluate(0.0f);
    EXPECT_NEAR(p0.x, 0.0f, 1e-5f);
    EXPECT_NEAR(p0.y, 0.0f, 1e-5f);

    // At t=1, should be at second waypoint
    Vec3 p1 = spline.evaluate(1.0f);
    EXPECT_NEAR(p1.x, 1.0f, 1e-5f);
    EXPECT_NEAR(p1.y, 0.0f, 1e-5f);

    // At t=2, should be at third waypoint
    Vec3 p2 = spline.evaluate(2.0f);
    EXPECT_NEAR(p2.x, 2.0f, 1e-5f);
    EXPECT_NEAR(p2.y, 1.0f, 1e-5f);

    // At t=3 (end), should be at last waypoint
    Vec3 p3 = spline.evaluate(3.0f);
    EXPECT_NEAR(p3.x, 3.0f, 1e-5f);
    EXPECT_NEAR(p3.y, 0.0f, 1e-5f);
}

TEST(CatmullRomSpline, TwoPointInterpolation) {
    CatmullRomSpline spline;
    spline.add_waypoints({{0,0,0}, {2,0,0}});

    // Midpoint should be roughly (1, 0, 0) for a straight line
    Vec3 mid = spline.evaluate(0.5f);
    EXPECT_NEAR(mid.x, 1.0f, 1e-4f);
    EXPECT_NEAR(mid.y, 0.0f, 1e-4f);
    EXPECT_NEAR(mid.z, 0.0f, 1e-4f);
}

TEST(CatmullRomSpline, ClampsBeyondRange) {
    CatmullRomSpline spline;
    spline.add_waypoints({{0,0,0}, {1,0,0}});

    Vec3 before = spline.evaluate(-1.0f);
    EXPECT_NEAR(before.x, 0.0f, 1e-5f);

    Vec3 after = spline.evaluate(5.0f);
    EXPECT_NEAR(after.x, 1.0f, 1e-5f);
}

// --- Tangent ---

TEST(CatmullRomSpline, TangentStraightLine) {
    CatmullRomSpline spline;
    spline.add_waypoints({{0,0,0}, {1,0,0}, {2,0,0}});

    Vec3 t = spline.tangent(0.5f);
    // Should point along +X
    EXPECT_GT(t.x, 0.9f);
    EXPECT_NEAR(t.y, 0.0f, 0.1f);
    EXPECT_NEAR(t.z, 0.0f, 0.1f);
}

TEST(CatmullRomSpline, TangentIsUnitLength) {
    CatmullRomSpline spline;
    spline.add_waypoints({{0,0,0}, {1,2,0}, {3,1,0}, {4,3,0}});

    for (float t = 0.0f; t <= 3.0f; t += 0.25f) {
        Vec3 tan = spline.tangent(t);
        EXPECT_NEAR(tan.length(), 1.0f, 0.05f) << "at t=" << t;
    }
}

// --- Arc length ---

TEST(CatmullRomSpline, ArcLengthStraightLine) {
    CatmullRomSpline spline;
    spline.add_waypoints({{0,0,0}, {1,0,0}, {2,0,0}});

    float arc = spline.total_arc_length(50);
    EXPECT_NEAR(arc, 2.0f, 0.01f);
}

TEST(CatmullRomSpline, ArcLengthGreaterThanChord) {
    CatmullRomSpline spline;
    spline.add_waypoints({{0,0,0}, {1,1,0}, {2,0,0}});

    float arc = spline.total_arc_length(50);
    float chord = (spline.end() - spline.start()).length();
    EXPECT_GT(arc, chord);
}

// --- Polyline ---

TEST(CatmullRomSpline, ToPolylineFixed) {
    CatmullRomSpline spline;
    spline.add_waypoints({{0,0,0}, {1,0,0}, {2,1,0}});

    auto poly = spline.to_polyline_fixed(10);
    // 2 segments * 10 samples + 1 = 21 points
    EXPECT_EQ(poly.size(), 21u);

    // First and last should match waypoints
    EXPECT_NEAR(poly.front().x, 0.0f, 1e-5f);
    EXPECT_NEAR(poly.back().x, 2.0f, 1e-5f);
    EXPECT_NEAR(poly.back().y, 1.0f, 1e-5f);
}

TEST(CatmullRomSpline, ToPolylineByLength) {
    CatmullRomSpline spline;
    spline.add_waypoints({{0,0,0}, {1,0,0}, {2,0,0}, {3,0,0}});

    auto poly = spline.to_polyline(0.5f);
    EXPECT_GT(poly.size(), 4u);

    // Should start and end at waypoints
    EXPECT_NEAR(poly.front().x, 0.0f, 1e-5f);
    EXPECT_NEAR(poly.back().x, 3.0f, 1e-5f);
}

// --- Edge cases ---

TEST(CatmullRomSpline, CollinearWaypoints) {
    CatmullRomSpline spline;
    spline.add_waypoints({{0,0,0}, {1,0,0}, {2,0,0}, {3,0,0}});

    // All intermediate evaluations should be on the X axis
    for (float t = 0.0f; t <= 3.0f; t += 0.1f) {
        Vec3 p = spline.evaluate(t);
        EXPECT_NEAR(p.y, 0.0f, 1e-4f) << "at t=" << t;
        EXPECT_NEAR(p.z, 0.0f, 1e-4f) << "at t=" << t;
    }
}

TEST(CatmullRomSpline, EmptyPolyline) {
    CatmullRomSpline spline;
    auto poly = spline.to_polyline_fixed(10);
    EXPECT_TRUE(poly.empty());
}

TEST(CatmullRomSpline, EmptyArcLength) {
    CatmullRomSpline spline;
    EXPECT_FLOAT_EQ(spline.total_arc_length(), 0.0f);
}

TEST(CatmullRomSpline, ThreePointCurve) {
    CatmullRomSpline spline;
    spline.add_waypoints({{0,0,0}, {1,1,0}, {2,0,0}});

    // The curve should pass through the middle waypoint
    Vec3 mid = spline.evaluate(1.0f);
    EXPECT_NEAR(mid.x, 1.0f, 1e-5f);
    EXPECT_NEAR(mid.y, 1.0f, 1e-5f);

    // Midpoints should be smooth (above 0 in Y)
    Vec3 q1 = spline.evaluate(0.5f);
    EXPECT_GT(q1.y, 0.0f);
    Vec3 q2 = spline.evaluate(1.5f);
    EXPECT_GT(q2.y, 0.0f);
}
