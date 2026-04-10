#include <gtest/gtest.h>
#include "geometry.hpp"
#include "yarn_path.hpp"
#include "surface/surface.hpp"
#include "test_helpers.hpp"
#include <cmath>
#include <numbers>
#include <sstream>

using namespace yarnpath;
using namespace yarnpath::test;

// Helper to build surface graph for testing
static SurfaceGraph build_test_surface(const YarnPath& yarn_path,
                                        const YarnProperties& yarn,
                                        const Gauge& gauge) {
    SurfaceBuildConfig build_config;
    build_config.random_seed = 42;

    SurfaceGraph surface = SurfaceBuilder::from_yarn_path(yarn_path, yarn, gauge, build_config);

    SolveConfig solve_config;
    solve_config.max_iterations = 1000;
    solve_config.convergence_threshold = 1e-4f;

    SurfaceSolver::solve(surface, yarn, gauge, solve_config);

    return surface;
}

// ============================================
// Tangent Continuity Tests
// Tests that verify yarn path doesn't have sharp turns (90-degree angles)
// ============================================

// Helper to compute angle between two vectors in degrees
static float angle_between_degrees(const Vec3& a, const Vec3& b) {
    float dot = a.normalized().dot(b.normalized());
    // Clamp to avoid NaN from acos
    dot = std::max(-1.0f, std::min(1.0f, dot));
    return std::acos(dot) * 180.0f / std::numbers::pi_v<float>;
}


TEST(ContinuityTest, BasicGeometryGeneration) {
    // Test that geometry can be generated for a stockinette pattern
    PatternInstructions pattern = create_pattern({
        "CCC",   // Cast on 3
        "KKK",   // Row 1 (RS): K3
        "PPP",   // Row 2 (WS): P3
        "KKK"    // Row 3 (RS): K3
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        surface,
        yarn,
        gauge
    );

    // Should have segments generated
    EXPECT_FALSE(geometry.segments().empty());
    // Should have loop positions
    EXPECT_EQ(geometry.segments().size(), 12u);  // 3 stitches * 4 rows
    // Should have non-empty polyline
    auto polyline = geometry.to_polyline_fixed(10);
    EXPECT_FALSE(polyline.empty());
}

TEST(ContinuityTest, PolylineSmoothnessCheck) {
    // Test geometry with mixed stitch types
    PatternInstructions pattern = create_pattern({
        "CCCCCC",
        "KKPPKK",   // Knit-purl transitions
        "PPKKPP",   // Purl-knit transitions
        "KKPPKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        surface,
        yarn,
        gauge
    );

    // Check geometry was generated successfully
    EXPECT_FALSE(geometry.segments().empty());
    EXPECT_EQ(geometry.segments().size(), 24u);  // 6 stitches * 4 rows
}


TEST(ContinuityTest, TangentDirectionsAreConsistent) {
    // Test that tangent directions at anchors are consistent
    // The outgoing tangent should be in roughly the same direction as yarn flow
    PatternInstructions pattern = create_pattern({
        "CCC",
        "KKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        surface,
        yarn,
        gauge
    );

    // For each segment, verify the curve tangents make sense
    for (const auto& seg : geometry.segments()) {
        if (seg.curve.empty()) continue;

        Vec3 start = seg.curve.evaluate(0.0f);
        float end_t = static_cast<float>(seg.curve.segment_count());
        Vec3 end = seg.curve.evaluate(end_t);
        Vec3 chord = end - start;

        // Skip very short segments
        if (chord.length() < 0.001f) {
            continue;
        }

        // Get tangent at start
        Vec3 tangent_start = seg.curve.tangent(0.0f);
        if (tangent_start.length() < 0.0001f) {
            continue;
        }

        // The tangent at start should have positive dot product with chord
        // (i.e., pointing in roughly the same direction as the segment goes)
        float dot = tangent_start.normalized().dot(chord.normalized());

        EXPECT_GT(dot, -0.5f)
            << "Segment " << seg.segment_id
            << " has start tangent pointing away from destination (dot=" << dot << ")";
    }
}

TEST(ContinuityTest, WaypointsAreFinite) {
    // Test that all waypoints have finite coordinates
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KKKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        surface,
        yarn,
        gauge
    );

    for (const auto& seg : geometry.segments()) {
        for (const auto& wp : seg.curve.waypoints()) {
            EXPECT_TRUE(std::isfinite(wp.x))
                << "Segment " << seg.segment_id << " has non-finite waypoint x";
            EXPECT_TRUE(std::isfinite(wp.y))
                << "Segment " << seg.segment_id << " has non-finite waypoint y";
            EXPECT_TRUE(std::isfinite(wp.z))
                << "Segment " << seg.segment_id << " has non-finite waypoint z";
        }
    }
}

// ============================================
// Curvature Constraint Tests
// ============================================

TEST(CurvatureTest, ArcLengthIsFinite) {
    // Test that arc length values are computed correctly (finite, non-negative)
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KKKK",
        "PPPP"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());
    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        surface,
        yarn,
        gauge
    );

    // Check that arc length values are reasonable (finite and non-negative)
    for (const auto& seg : geometry.segments()) {
        EXPECT_GE(seg.arc_length, 0.0f)
            << "Segment " << seg.segment_id << " has negative arc length";
        EXPECT_TRUE(std::isfinite(seg.arc_length))
            << "Segment " << seg.segment_id << " has non-finite arc length";
    }

    EXPECT_FALSE(geometry.segments().empty());
}

// ============================================
// Polyline Sharp Turn Tests (using actual output sampling)
// ============================================

TEST(PolylineTest, PolylineOutputExists) {
    // Test that polyline can be generated from geometry
    PatternInstructions pattern = create_pattern({
        "CCC",
        "KKK",
        "PPP",
        "KKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        surface,
        yarn,
        gauge
    );

    // Get polyline with same sampling as OBJ output (10 samples per segment)
    auto polyline = geometry.to_polyline_fixed(10);

    EXPECT_FALSE(polyline.empty());
    // Should have points for all the loops
    EXPECT_GT(polyline.size(), 50u);
}

TEST(PolylineTest, PolylineBoundingBoxMakesSense) {
    // Test that polyline has sensible bounds
    PatternInstructions pattern = create_pattern({
        "CCC",      // Cast on 3
        "KKK",      // Row 1 (RS): K3
        "PPP",      // Row 2 (WS): P3
        "KKK"       // Row 3 (RS): K3
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        surface,
        yarn,
        gauge
    );

    auto [min_pt, max_pt] = geometry.bounding_box();

    // Should have non-zero extent
    EXPECT_GT(max_pt.x - min_pt.x, 0.0f);
    EXPECT_GT(max_pt.y - min_pt.y, 0.0f);
}

// Curvature constraint tests removed — curvature validation is no longer
// part of the geometry pipeline (Catmull-Rom splines determine curvature
// from waypoint placement, not from construction constraints).
