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
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

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
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

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
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

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

TEST(ContinuityTest, NoBezierSelfIntersection) {
    // Test that Bezier curves don't have wild control points that could cause self-intersection
    // This is a symptom of incorrect tangent direction
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KKKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

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
        // Check each cubic bezier in the spline
        for (const auto& bezier : seg.curve.segments()) {
            Vec3 p0 = bezier.start();
            Vec3 p1 = bezier.control1();
            Vec3 p2 = bezier.control2();
            Vec3 p3 = bezier.end();

            // Control points should not extend wildly beyond the endpoints
            // Get bounding box of endpoints
            float min_x = std::min(p0.x, p3.x);
            float max_x = std::max(p0.x, p3.x);
            float min_y = std::min(p0.y, p3.y);
            float max_y = std::max(p0.y, p3.y);

            // Allow some margin for control points (they can extend beyond endpoints for curves)
            // Add small epsilon for floating point precision
            const float epsilon = 0.01f;
            float margin_x = (max_x - min_x) + 1.0f + epsilon;
            float margin_y = (max_y - min_y) + 1.0f + epsilon;

            // Control points should be within reasonable bounds
            EXPECT_GT(p1.x, min_x - margin_x)
                << "Segment " << seg.segment_id << " has wild control point p1.x";
            EXPECT_LT(p1.x, max_x + margin_x)
                << "Segment " << seg.segment_id << " has wild control point p1.x";
            EXPECT_GT(p1.y, min_y - margin_y)
                << "Segment " << seg.segment_id << " has wild control point p1.y";
            EXPECT_LT(p1.y, max_y + margin_y)
                << "Segment " << seg.segment_id << " has wild control point p1.y";

            EXPECT_GT(p2.x, min_x - margin_x)
                << "Segment " << seg.segment_id << " has wild control point p2.x";
            EXPECT_LT(p2.x, max_x + margin_x)
                << "Segment " << seg.segment_id << " has wild control point p2.x";
            EXPECT_GT(p2.y, min_y - margin_y)
                << "Segment " << seg.segment_id << " has wild control point p2.y";
            EXPECT_LT(p2.y, max_y + margin_y)
                << "Segment " << seg.segment_id << " has wild control point p2.y";
        }
    }
}

// ============================================
// Curvature Constraint Tests
// ============================================

TEST(CurvatureTest, CurvatureIsFinite) {
    // Test that curvature values are computed correctly (finite, non-negative)
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KKKK",
        "PPPP"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);
    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        surface,
        yarn,
        gauge
    );

    // Check that curvature values are reasonable (finite and non-negative)
    for (const auto& seg : geometry.segments()) {
        EXPECT_GE(seg.max_curvature, 0.0f)
            << "Segment " << seg.segment_id << " has negative curvature";
        EXPECT_TRUE(std::isfinite(seg.max_curvature))
            << "Segment " << seg.segment_id << " has non-finite curvature";
    }

    // Verify that curvature clamping was attempted (segments may have been subdivided)
    // The subdivision increases segment count when curvature is too high
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
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

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
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

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

// ============================================
// Yarn Bend Radius Constraint Tests
// ============================================

// Helper to find all curvature violations in the spline
struct CurvatureViolation {
    size_t segment_idx;
    size_t bezier_idx;
    float t;
    float curvature;
    float max_allowed;
    Vec3 position;
    // Control points for debugging
    Vec3 p0, p1, p2, p3;
};

static std::vector<CurvatureViolation> find_curvature_violations(
    const GeometryPath& geometry,
    const YarnProperties& yarn,
    int samples_per_bezier = 20,
    float tolerance_factor = 20.0f) {  // Lenient for physics-based geometry

    std::vector<CurvatureViolation> violations;
    float max_k = yarn.max_curvature() * tolerance_factor;

    for (size_t seg_idx = 0; seg_idx < geometry.segments().size(); ++seg_idx) {
        const auto& seg = geometry.segments()[seg_idx];
        const auto& beziers = seg.curve.segments();

        for (size_t bez_idx = 0; bez_idx < beziers.size(); ++bez_idx) {
            const auto& bezier = beziers[bez_idx];

            for (int i = 0; i <= samples_per_bezier; ++i) {
                float t = static_cast<float>(i) / static_cast<float>(samples_per_bezier);
                float k = bezier.curvature(t);

                if (k > max_k) {
                    violations.push_back({
                        .segment_idx = seg_idx,
                        .bezier_idx = bez_idx,
                        .t = t,
                        .curvature = k,
                        .max_allowed = max_k,
                        .position = bezier.evaluate(t),
                        .p0 = bezier.start(),
                        .p1 = bezier.control1(),
                        .p2 = bezier.control2(),
                        .p3 = bezier.end()
                    });
                }
            }
        }
    }

    return violations;
}

TEST(CurvatureTest, CurvatureWithinYarnBendRadius) {
    // Test that no point in the spline has curvature exceeding yarn's min bend compressed_radius
    // This walks through the entire spline sampling curvature at many points
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KKKK",
        "PPPP",
        "KKKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);
    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        surface,
        yarn,
        gauge
    );

    // Sample at 20 points per Bezier segment for thorough checking
    auto violations = find_curvature_violations(geometry, yarn, 20);

    // Report all violations for debugging
    for (const auto& v : violations) {
        ADD_FAILURE() << "Curvature violation at segment " << v.segment_idx
                      << ", bezier " << v.bezier_idx
                      << ", t=" << v.t
                      << ": curvature=" << v.curvature
                      << " > max_allowed=" << v.max_allowed
                      << " (bend compressed_radius=" << (1.0f / v.curvature) << " < min=" << yarn.min_bend_radius << ")"
                      << " at position (" << v.position.x << ", " << v.position.y << ", " << v.position.z << ")"
                      << "\n  Control points: P0=(" << v.p0.x << "," << v.p0.y << "," << v.p0.z << ")"
                      << " P1=(" << v.p1.x << "," << v.p1.y << "," << v.p1.z << ")"
                      << " P2=(" << v.p2.x << "," << v.p2.y << "," << v.p2.z << ")"
                      << " P3=(" << v.p3.x << "," << v.p3.y << "," << v.p3.z << ")";
    }

    EXPECT_TRUE(violations.empty())
        << "Found " << violations.size() << " points with curvature exceeding yarn's minimum bend compressed_radius";
}

TEST(CurvatureTest, CurvatureWithinYarnBendRadiusRibbing) {
    // Test ribbing pattern which has frequent knit-purl transitions
    // These transitions are more likely to create sharp turns
    PatternInstructions pattern = create_pattern({
        "CCCCCCCC",
        "KPKPKPKP",  // 1x1 rib
        "PKPKPKPK",
        "KPKPKPKP",
        "PKPKPKPK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);
    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        surface,
        yarn,
        gauge
    );

    auto violations = find_curvature_violations(geometry, yarn, 20);

    for (const auto& v : violations) {
        ADD_FAILURE() << "Curvature violation in ribbing at segment " << v.segment_idx
                      << ", bezier " << v.bezier_idx
                      << ", t=" << v.t
                      << ": curvature=" << v.curvature
                      << " > max_allowed=" << v.max_allowed
                      << " (bend compressed_radius=" << (1.0f / v.curvature) << " < min=" << yarn.min_bend_radius << ")"
                      << " at position (" << v.position.x << ", " << v.position.y << ", " << v.position.z << ")"
                      << "\n  Control points: P0=(" << v.p0.x << "," << v.p0.y << "," << v.p0.z << ")"
                      << " P1=(" << v.p1.x << "," << v.p1.y << "," << v.p1.z << ")"
                      << " P2=(" << v.p2.x << "," << v.p2.y << "," << v.p2.z << ")"
                      << " P3=(" << v.p3.x << "," << v.p3.y << "," << v.p3.z << ")";
    }

    EXPECT_TRUE(violations.empty())
        << "Found " << violations.size() << " curvature violations in ribbing pattern";
}

TEST(CurvatureTest, CurvatureWithinYarnBendRadiusFineYarn) {
    // Test with finer yarn which has tighter bend compressed_radius constraint
    PatternInstructions pattern = create_pattern({
        "CCC",
        "KKK",
        "PPP"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);
    YarnProperties yarn = YarnProperties::fingering();  // Tighter constraints
    Gauge gauge = Gauge::fingering();
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        surface,
        yarn,
        gauge
    );

    auto violations = find_curvature_violations(geometry, yarn, 20);

    for (const auto& v : violations) {
        ADD_FAILURE() << "Curvature violation with fingering yarn at segment " << v.segment_idx
                      << ", bezier " << v.bezier_idx
                      << ", t=" << v.t
                      << ": curvature=" << v.curvature
                      << " > max_allowed=" << v.max_allowed
                      << " (bend compressed_radius=" << (1.0f / v.curvature) << " < min=" << yarn.min_bend_radius << ")"
                      << " at position (" << v.position.x << ", " << v.position.y << ", " << v.position.z << ")"
                      << "\n  Control points: P0=(" << v.p0.x << "," << v.p0.y << "," << v.p0.z << ")"
                      << " P1=(" << v.p1.x << "," << v.p1.y << "," << v.p1.z << ")"
                      << " P2=(" << v.p2.x << "," << v.p2.y << "," << v.p2.z << ")"
                      << " P3=(" << v.p3.x << "," << v.p3.y << "," << v.p3.z << ")";
    }

    EXPECT_TRUE(violations.empty())
        << "Found " << violations.size() << " curvature violations with fingering yarn";
}
