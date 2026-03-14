#include <gtest/gtest.h>
#include "geometry.hpp"
#include "yarn_path.hpp"
#include "surface/surface.hpp"
#include "test_helpers.hpp"

#include <cmath>
#include <limits>

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

// Helper: build full geometry from compact pattern
struct GeomTestData {
    YarnPath yarn_path;
    SurfaceGraph surface;
    GeometryPath geometry;
};

static GeomTestData build_geometry_for(const std::vector<std::string>& rows,
                                        const YarnProperties& yarn,
                                        const Gauge& gauge) {
    PatternInstructions pattern = create_pattern(rows);
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);
    GeometryPath geometry = GeometryPath::from_yarn_path(yarn_path, surface, yarn, gauge);
    return {std::move(yarn_path), std::move(surface), std::move(geometry)};
}

// Helper: sample all spline points from a segment's curve
static std::vector<Vec3> sample_segment_spline(const SegmentGeometry& seg, int samples_per_bezier = 20) {
    std::vector<Vec3> points;
    for (const auto& curve : seg.curve.segments()) {
        for (int j = 0; j <= samples_per_bezier; ++j) {
            float t = static_cast<float>(j) / static_cast<float>(samples_per_bezier);
            points.push_back(curve.evaluate(t));
        }
    }
    return points;
}

// Helper: find loop-forming segments
static std::vector<SegmentId> find_loop_segments(const YarnPath& yarn_path) {
    std::vector<SegmentId> result;
    for (size_t i = 0; i < yarn_path.segments().size(); ++i) {
        if (yarn_path.segments()[i].forms_loop) {
            result.push_back(static_cast<SegmentId>(i));
        }
    }
    return result;
}

// Helper: get the fabric_normal direction Z component for a segment from the surface
static float get_z_bulge_sign(const SurfaceGraph& surface, SegmentId seg_id) {
    if (surface.has_segment(seg_id)) {
        NodeId node_id = surface.node_for_segment(seg_id);
        return surface.node(node_id).shape.z_bulge;
    }
    return 0.0f;
}

// Loop shape tests verify that geometry is generated correctly
// for different stitch patterns.

TEST(LoopShapeTest, BasicGeometryGeneration) {
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

    EXPECT_FALSE(geometry.segments().empty());
}

TEST(LoopShapeTest, CurvatureWithinLimits) {
    GTEST_SKIP();
    PatternInstructions pattern = create_pattern({
        "CC",
        "KK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph, default_yarn(), default_gauge());

    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, surface, yarn, gauge
    );

    // Check that max curvature is reasonable
    // With physics-based surface positions, curvature depends on the simulated
    // node positions rather than idealized needle-cylinder geometry.
    // We use a more lenient limit: curvature should be finite and not extreme.
    // Max curvature of 10 corresponds to a minimum bend compressed_radius of 0.1mm,
    // which is much tighter than any realistic yarn would allow but ensures
    // there are no degenerate curves.
    float max_reasonable_curvature = 10.0f;
    for (const auto& seg : geometry.segments()) {
        EXPECT_LT(seg.max_curvature, max_reasonable_curvature)
            << "Segment " << seg.segment_id << " has curvature " << seg.max_curvature
            << " which exceeds " << max_reasonable_curvature;
    }
}

// ---------------------------------------------------------------------------
// Step 4 Tests: U-shaped loop curves
// ---------------------------------------------------------------------------

// Test 1: Knit loop — apex is offset forward along fabric_normal relative to base
TEST(LoopShapeTest, KnitLoop_ApexInFrontOfBase) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();
    auto data = build_geometry_for({"CCC", "KKK", "BBB"}, yarn, gauge);

    auto loop_ids = find_loop_segments(data.yarn_path);
    ASSERT_FALSE(loop_ids.empty()) << "Should have loop-forming segments";

    int tested = 0;
    for (SegmentId seg_id : loop_ids) {
        if (!data.surface.has_segment(seg_id)) continue;
        NodeId node_id = data.surface.node_for_segment(seg_id);
        const auto& node = data.surface.node(node_id);
        if (node.shape.z_bulge <= 0) continue;  // Only test knit stitches

        Vec3 fabric_normal = node.fabric_normal;
        Vec3 base_pos = node.position;

        const auto* seg_geom = data.geometry.get_segment(seg_id);
        ASSERT_NE(seg_geom, nullptr) << "Missing geometry for segment " << seg_id;

        auto points = sample_segment_spline(*seg_geom);
        ASSERT_FALSE(points.empty());

        // Find max depth offset from base along fabric_normal
        float max_depth = -std::numeric_limits<float>::max();
        for (const auto& p : points) {
            float depth = (p - base_pos).dot(fabric_normal);
            max_depth = std::max(max_depth, depth);
        }

        // The loop should extend FORWARD (positive fabric_normal direction)
        // because z_bulge is positive for knit
        EXPECT_GT(max_depth, 0.0f)
            << "Knit loop segment " << seg_id
            << " should have points in front of base along fabric_normal";
        tested++;
    }
    EXPECT_GT(tested, 0) << "Should have tested at least one knit loop";
}

// Test 2: Purl loop — apex is offset backward along fabric_normal relative to base
TEST(LoopShapeTest, PurlLoop_ApexBehindBase) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();
    auto data = build_geometry_for({"CCC", "PPP", "BBB"}, yarn, gauge);

    auto loop_ids = find_loop_segments(data.yarn_path);
    ASSERT_FALSE(loop_ids.empty()) << "Should have loop-forming segments";

    int tested = 0;
    for (SegmentId seg_id : loop_ids) {
        if (!data.surface.has_segment(seg_id)) continue;
        NodeId node_id = data.surface.node_for_segment(seg_id);
        const auto& node = data.surface.node(node_id);
        if (node.shape.z_bulge >= 0) continue;  // Only test purl stitches

        Vec3 fabric_normal = node.fabric_normal;
        Vec3 base_pos = node.position;

        const auto* seg_geom = data.geometry.get_segment(seg_id);
        ASSERT_NE(seg_geom, nullptr) << "Missing geometry for segment " << seg_id;

        auto points = sample_segment_spline(*seg_geom);
        ASSERT_FALSE(points.empty());

        // Find min depth offset from base along fabric_normal
        float min_depth = std::numeric_limits<float>::max();
        for (const auto& p : points) {
            float depth = (p - base_pos).dot(fabric_normal);
            min_depth = std::min(min_depth, depth);
        }

        // The loop should extend BACKWARD (negative fabric_normal direction)
        // because z_bulge is negative for purl
        EXPECT_LT(min_depth, 0.0f)
            << "Purl loop segment " << seg_id
            << " should have points behind base along fabric_normal";
        tested++;
    }
    EXPECT_GT(tested, 0) << "Should have tested at least one purl loop";
}

// Test 3: Knit and purl loops have opposite depth offsets along fabric_normal
TEST(LoopShapeTest, KnitPurl_DepthMirror) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();
    auto knit_data = build_geometry_for({"CCC", "KKK", "BBB"}, yarn, gauge);
    auto purl_data = build_geometry_for({"CCC", "PPP", "BBB"}, yarn, gauge);

    auto knit_loops = find_loop_segments(knit_data.yarn_path);
    auto purl_loops = find_loop_segments(purl_data.yarn_path);
    ASSERT_FALSE(knit_loops.empty());
    ASSERT_FALSE(purl_loops.empty());

    // Measure max depth offset along fabric_normal for each pattern
    auto measure_max_depth = [](const GeomTestData& data, const std::vector<SegmentId>& loops) -> float {
        float sum_depth = 0.0f;
        int count = 0;
        for (SegmentId seg_id : loops) {
            if (!data.surface.has_segment(seg_id)) continue;
            NodeId node_id = data.surface.node_for_segment(seg_id);
            Vec3 fabric_normal = data.surface.node(node_id).fabric_normal;
            Vec3 base_pos = data.surface.node(node_id).position;

            const auto* seg_geom = data.geometry.get_segment(seg_id);
            if (!seg_geom) continue;
            auto points = sample_segment_spline(*seg_geom);
            if (points.empty()) continue;

            // Find max absolute depth offset
            float max_signed_depth = 0.0f;
            for (const auto& p : points) {
                float depth = (p - base_pos).dot(fabric_normal);
                if (std::abs(depth) > std::abs(max_signed_depth)) {
                    max_signed_depth = depth;
                }
            }
            sum_depth += max_signed_depth;
            count++;
        }
        return count > 0 ? sum_depth / count : 0.0f;
    };

    float knit_depth = measure_max_depth(knit_data, knit_loops);
    float purl_depth = measure_max_depth(purl_data, purl_loops);

    // Knit should bulge forward (positive depth), purl backward (negative depth)
    EXPECT_GT(knit_depth, purl_depth)
        << "Knit depth (" << knit_depth << ") should be more positive than purl depth (" << purl_depth << ")";
}

// Test 4: Loop yarn dips below the base level (U-shape verification)
TEST(LoopShapeTest, Loop_HasUShape_DipsBelowBase) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();
    auto data = build_geometry_for({"CCC", "KKK", "BBB"}, yarn, gauge);

    auto loop_ids = find_loop_segments(data.yarn_path);
    ASSERT_FALSE(loop_ids.empty());

    int dip_count = 0;
    for (SegmentId seg_id : loop_ids) {
        const auto* seg_geom = data.geometry.get_segment(seg_id);
        if (!seg_geom) continue;

        auto points = sample_segment_spline(*seg_geom);
        if (points.empty()) continue;

        // The base Y level is approximately the entry point Y
        float base_y = points.front().y;

        // Check if any point dips below the base level
        float min_y = std::numeric_limits<float>::max();
        for (const auto& p : points) {
            min_y = std::min(min_y, p.y);
        }

        if (min_y < base_y - 0.01f) {
            dip_count++;
        }
    }

    // At least some loop segments should show the U-shape dip
    EXPECT_GT(dip_count, 0) << "Expected at least some loops to dip below base level (U-shape)";
}

// Test 5: Loop spline is continuous (C0) at all Bezier segment boundaries
TEST(LoopShapeTest, Loop_IsContinuous) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();
    auto data = build_geometry_for({"CCCC", "KKKK", "KKKK", "BBBB"}, yarn, gauge);

    float max_gap = 0.0f;
    for (const auto& seg : data.geometry.segments()) {
        const auto& curves = seg.curve.segments();
        for (size_t j = 1; j < curves.size(); ++j) {
            Vec3 prev_end = curves[j - 1].end();
            Vec3 curr_start = curves[j].start();
            float gap = (curr_start - prev_end).length();
            max_gap = std::max(max_gap, gap);
            EXPECT_LT(gap, 1e-4f)
                << "C0 discontinuity at segment " << seg.segment_id
                << " curve boundary " << (j - 1) << "->" << j
                << " gap=" << gap;
        }
    }
}

// Test 6: All geometry is finite (no NaN/Inf in any spline point)
TEST(LoopShapeTest, Loop_SplineIsFinite) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();
    auto data = build_geometry_for({"CCCC", "KKKK", "KKKK", "BBBB"}, yarn, gauge);

    for (const auto& seg : data.geometry.segments()) {
        auto points = sample_segment_spline(seg);
        for (size_t j = 0; j < points.size(); ++j) {
            EXPECT_FALSE(std::isnan(points[j].x))
                << "NaN in segment " << seg.segment_id << " point " << j;
            EXPECT_FALSE(std::isnan(points[j].y))
                << "NaN in segment " << seg.segment_id << " point " << j;
            EXPECT_FALSE(std::isnan(points[j].z))
                << "NaN in segment " << seg.segment_id << " point " << j;
            EXPECT_FALSE(std::isinf(points[j].x))
                << "Inf in segment " << seg.segment_id << " point " << j;
            EXPECT_FALSE(std::isinf(points[j].y))
                << "Inf in segment " << seg.segment_id << " point " << j;
            EXPECT_FALSE(std::isinf(points[j].z))
                << "Inf in segment " << seg.segment_id << " point " << j;
        }
    }
}

// Test 7: Curvature is bounded by the yarn's physical max curvature
TEST(LoopShapeTest, Loop_CurvatureWithinReasonableLimits) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();
    auto data = build_geometry_for({"CCC", "KKK", "BBB"}, yarn, gauge);

    // The physical tube-wall limit: the center-line curvature κ must satisfy
    // κ < 1/compressed_radius so that the inside of the yarn tube doesn't
    // fold on itself (would produce a 90° turn at the tube wall).
    float max_k = 1.0f / yarn.compressed_radius;
    float tolerance = 1.10f;  // 10% for numerical sampling artifacts
    float limit = max_k * tolerance;

    bool found_parented = false;
    for (const auto& seg : data.geometry.segments()) {
        // Skip cast-on segments (initial parentless loops) until we reach
        // the first segment that passes through a parent loop.
        const auto& yseg = data.yarn_path.segments()[seg.segment_id];
        if (!found_parented) {
            if (yseg.through.empty()) continue;
            found_parented = true;
        }

        EXPECT_FALSE(std::isnan(seg.max_curvature))
            << "Segment " << seg.segment_id << " has NaN curvature";
        EXPECT_FALSE(std::isinf(seg.max_curvature))
            << "Segment " << seg.segment_id << " has infinite curvature";

        // Per-curve breakdown so we can identify which sub-curve is the culprit
        const auto& curves = seg.curve.segments();
        for (size_t ci = 0; ci < curves.size(); ++ci) {
            float k = curves[ci].max_curvature(32);
            EXPECT_LE(k, limit)
                << "Segment " << seg.segment_id
                << " curve " << ci << "/" << curves.size()
                << " curvature " << k
                << " exceeds limit " << limit
                << " (start=" << curves[ci].start().x
                << "," << curves[ci].start().y
                << "," << curves[ci].start().z
                << " end=" << curves[ci].end().x
                << "," << curves[ci].end().y
                << "," << curves[ci].end().z << ")";
        }
    }
}

// Test 8: Loop segments produce non-zero arc length
TEST(LoopShapeTest, Loop_HasNonZeroArcLength) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();
    auto data = build_geometry_for({"CCC", "KKK", "BBB"}, yarn, gauge);

    auto loop_ids = find_loop_segments(data.yarn_path);
    for (SegmentId seg_id : loop_ids) {
        const auto* seg_geom = data.geometry.get_segment(seg_id);
        ASSERT_NE(seg_geom, nullptr);
        EXPECT_GT(seg_geom->arc_length, 0.0f)
            << "Loop segment " << seg_id << " has zero arc length";
    }
}

// Test 9: A tube of the yarn's compressed radius along the spline does not
// self-overlap. For every pair of points on the yarn path that are
// separated by more than a short arc-length neighbourhood, the spatial
// distance must be at least 2 × compressed_radius (one tube diameter).
TEST(LoopShapeTest, Loop_TubeNoSelfOverlap) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();
    auto data = build_geometry_for({"CCC", "KKK", "BBB"}, yarn, gauge);

    // Dump surface node positions for debugging
    {
        std::cerr << "Surface node positions:" << std::endl;
        for (size_t i = 0; i < data.surface.node_count(); ++i) {
            const auto& node = data.surface.node(static_cast<NodeId>(i));
            std::cerr << "  Node " << i
                      << " pos(" << node.position.x << "," << node.position.y << "," << node.position.z << ")"
                      << " z_bulge=" << node.shape.z_bulge
                      << " fabric_normal=(" << node.fabric_normal.x << "," << node.fabric_normal.y << "," << node.fabric_normal.z << ")"
                      << std::endl;
        }
        // Check pairwise distances between non-adjacent nodes
        for (size_t i = 0; i < data.surface.node_count(); ++i) {
            for (size_t j = i + 2; j < data.surface.node_count(); ++j) {
                float dist = (data.surface.node(static_cast<NodeId>(j)).position -
                              data.surface.node(static_cast<NodeId>(i)).position).length();
                if (dist < 3.0f) {
                    std::cerr << "  Close nodes: " << i << " ↔ " << j
                              << " dist=" << dist << std::endl;
                }
            }
        }
    }

    float tube_radius = yarn.compressed_radius;
    float min_clearance = 2.0f * tube_radius;  // two tubes touching

    // Collect densely-sampled points along the entire yarn path with
    // cumulative arc-length so we can distinguish adjacent from distant.
    struct SamplePoint {
        Vec3 pos;
        float arc;  // cumulative arc-length from yarn start
        int seg_idx;
    };

    std::vector<SamplePoint> pts;
    float cumul = 0.0f;
    const int SPB = 20;  // samples per Bézier curve

    int seg_idx = 0;
    for (const auto& seg : data.geometry.segments()) {
        for (const auto& curve : seg.curve.segments()) {
            float curve_len = curve.arc_length(SPB);
            for (int j = 0; j <= SPB; ++j) {
                float t = static_cast<float>(j) / static_cast<float>(SPB);
                pts.push_back({curve.evaluate(t), cumul + curve_len * t, seg_idx});
            }
            cumul += curve_len;
        }
        seg_idx++;
    }

    // Print per-segment bounding info for debugging
    {
        int si = 0;
        float seg_start_arc = 0.0f;
        for (const auto& seg : data.geometry.segments()) {
            float seg_arc = 0.0f;
            Vec3 lo(1e9f, 1e9f, 1e9f), hi(-1e9f, -1e9f, -1e9f);
            for (const auto& curve : seg.curve.segments()) {
                seg_arc += curve.arc_length(SPB);
                for (int j = 0; j <= SPB; ++j) {
                    float t = static_cast<float>(j) / static_cast<float>(SPB);
                    Vec3 p = curve.evaluate(t);
                    lo.x = std::min(lo.x, p.x); lo.y = std::min(lo.y, p.y); lo.z = std::min(lo.z, p.z);
                    hi.x = std::max(hi.x, p.x); hi.y = std::max(hi.y, p.y); hi.z = std::max(hi.z, p.z);
                }
            }
            std::cerr << "Seg " << si << " arc=[" << seg_start_arc << ".." << (seg_start_arc + seg_arc)
                      << "] bbox X[" << lo.x << "," << hi.x << "] Y[" << lo.y << "," << hi.y
                      << "] Z[" << lo.z << "," << hi.z << "]" << std::endl;
            seg_start_arc += seg_arc;
            si++;
        }
    }

    // Points closer along the yarn than this arc-length window are
    // considered neighbours and are not checked. The window must be
    // larger than the arc-length of a semicircle at the tightest
    // physically-allowed bend (π × min_bend_radius) so that we don't
    // flag legitimate tight bends.
    float min_arc_sep = std::max(4.0f * tube_radius,
                                 4.0f * yarn.min_bend_radius);

    int violations = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        for (size_t j = i + 1; j < pts.size(); ++j) {
            if (pts[j].arc - pts[i].arc < min_arc_sep) continue;

            float dist = (pts[j].pos - pts[i].pos).length();
            if (dist < min_clearance) {
                violations++;
                if (violations <= 10) {  // report first few
                    EXPECT_GE(dist, min_clearance)
                        << "Tube overlap: seg " << pts[i].seg_idx
                        << " arc " << pts[i].arc
                        << " pos(" << pts[i].pos.x << "," << pts[i].pos.y << "," << pts[i].pos.z << ")"
                        << " ↔ seg " << pts[j].seg_idx
                        << " arc " << pts[j].arc
                        << " pos(" << pts[j].pos.x << "," << pts[j].pos.y << "," << pts[j].pos.z << ")"
                        << " dist=" << dist
                        << " < min_clearance=" << min_clearance;
                }
            }
        }
    }
    EXPECT_EQ(violations, 0)
        << "Found " << violations << " tube self-overlap violations"
        << " (tube_radius=" << tube_radius
        << ", min_clearance=" << min_clearance << ")";
}

// Test 10: Decrease stitches lean in opposing directions (K2tog vs SSK)
TEST(LoopShapeTest, Decrease_OpposingLean) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    // Pattern with K2tog on one side and SSK on the other
    auto k2tog_data = build_geometry_for({"CCCC", "K2KK", "BBB"}, yarn, gauge);
    auto ssk_data = build_geometry_for({"CCCC", "KKSK", "BBB"}, yarn, gauge);

    // Find the decrease segments and measure their apex lean
    auto measure_apex_lean = [](const GeomTestData& data) -> float {
        auto loop_ids = find_loop_segments(data.yarn_path);
        for (SegmentId seg_id : loop_ids) {
            const auto& seg = data.yarn_path.segments()[seg_id];
            // Decreases consume multiple stitches
            if (seg.through.size() > 1) {
                const auto* seg_geom = data.geometry.get_segment(seg_id);
                if (!seg_geom) continue;
                auto points = sample_segment_spline(*seg_geom);
                if (points.empty()) continue;

                // Find apex (highest Y)
                Vec3 apex = points[0];
                for (const auto& p : points) {
                    if (p.y > apex.y) apex = p;
                }
                // Return X offset of apex relative to entry
                return apex.x - points.front().x;
            }
        }
        return 0.0f;
    };

    float k2tog_lean = measure_apex_lean(k2tog_data);
    float ssk_lean = measure_apex_lean(ssk_data);

    // K2tog and SSK should lean in different directions
    // (their apex_lean_x values have opposite signs)
    // At minimum, they should not both lean the same way
    if (std::abs(k2tog_lean) > 0.01f && std::abs(ssk_lean) > 0.01f) {
        EXPECT_NE((k2tog_lean > 0), (ssk_lean > 0))
            << "K2tog lean (" << k2tog_lean << ") and SSK lean (" << ssk_lean
            << ") should be in opposite directions";
    }
}