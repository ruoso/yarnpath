#include <gtest/gtest.h>
#include <geometry/position_resolver.hpp>
#include <surface/surface_builder.hpp>
#include <surface/surface_solver.hpp>
#include "test_helpers.hpp"

#include <cmath>

using namespace yarnpath;
using namespace yarnpath::test;

// ---------------------------------------------------------------------------
// Helper: build a full pipeline up to resolved segment frames
// ---------------------------------------------------------------------------
struct FrameTestData {
    YarnPath yarn_path;
    SurfaceGraph surface;
    std::vector<SegmentFrame> frames;
};

static FrameTestData build_frames(const std::vector<std::string>& rows) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    PatternInstructions pattern = create_pattern(rows);
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);

    SurfaceBuildConfig build_config;
    build_config.random_seed = 42;
    SurfaceGraph surface = SurfaceBuilder::from_yarn_path(yarn_path, yarn, gauge, build_config);

    SolveConfig solve_config;
    solve_config.max_iterations = 1000;
    solve_config.convergence_threshold = 1e-4f;
    SurfaceSolver::solve(surface, yarn, gauge, solve_config);

    // Build parent and children maps (same logic as geometry_builder)
    const auto& segments = yarn_path.segments();
    std::map<SegmentId, std::vector<SegmentId>> parent_map;
    std::map<SegmentId, std::vector<SegmentId>> children_map;
    for (size_t i = 0; i < segments.size(); ++i) {
        for (SegmentId parent_id : segments[i].through) {
            parent_map[i].push_back(parent_id);
            children_map[parent_id].push_back(i);
        }
    }

    auto frames = resolve_segment_frames(yarn_path, surface, yarn, parent_map, children_map);

    return FrameTestData{std::move(yarn_path), std::move(surface), std::move(frames)};
}

// ---------------------------------------------------------------------------
// Test 1: Positions match those stored in the surface graph
// ---------------------------------------------------------------------------
TEST(SegmentFrameTest, PositionMatchesSurface) {
    auto data = build_frames({"CCCC", "KKKK", "KKKK", "BBBB"});

    // For segments that have parents (children of loops), position may be adjusted.
    // For parentless root segments, position comes straight from the surface.
    // We just verify that every frame has a finite position.
    for (size_t i = 0; i < data.frames.size(); ++i) {
        const auto& f = data.frames[i];
        EXPECT_FALSE(std::isnan(f.position.x)) << "Frame " << i << " has NaN position.x";
        EXPECT_FALSE(std::isnan(f.position.y)) << "Frame " << i << " has NaN position.y";
        EXPECT_FALSE(std::isnan(f.position.z)) << "Frame " << i << " has NaN position.z";
    }

    // Segments that are in the surface should have positions matching their surface node
    // (before parentless adjustment). Check the non-parentless ones match exactly.
    const auto& segments = data.yarn_path.segments();
    for (size_t i = 0; i < segments.size(); ++i) {
        SegmentId seg_id = static_cast<SegmentId>(i);
        if (!segments[i].through.empty() && data.surface.has_segment(seg_id)) {
            // Has parents — no Y adjustment, should match surface exactly
            NodeId node_id = data.surface.node_for_segment(seg_id);
            const auto& node = data.surface.node(node_id);
            EXPECT_FLOAT_EQ(data.frames[i].position.x, node.position.x) << "Seg " << i;
            EXPECT_FLOAT_EQ(data.frames[i].position.y, node.position.y) << "Seg " << i;
            EXPECT_FLOAT_EQ(data.frames[i].position.z, node.position.z) << "Seg " << i;
        }
    }
}

// ---------------------------------------------------------------------------
// Test 2: stitch_axis matches the surface node value
// ---------------------------------------------------------------------------
TEST(SegmentFrameTest, StitchAxisMatchesSurface) {
    auto data = build_frames({"CCCC", "KKKK", "KKKK", "BBBB"});

    for (size_t i = 0; i < data.frames.size(); ++i) {
        SegmentId seg_id = static_cast<SegmentId>(i);
        if (data.surface.has_segment(seg_id)) {
            NodeId node_id = data.surface.node_for_segment(seg_id);
            const auto& node = data.surface.node(node_id);
            EXPECT_FLOAT_EQ(data.frames[i].stitch_axis.x, node.stitch_axis.x) << "Seg " << i;
            EXPECT_FLOAT_EQ(data.frames[i].stitch_axis.y, node.stitch_axis.y) << "Seg " << i;
            EXPECT_FLOAT_EQ(data.frames[i].stitch_axis.z, node.stitch_axis.z) << "Seg " << i;
        }
    }
}

// ---------------------------------------------------------------------------
// Test 3: fabric_normal matches the surface node value
// ---------------------------------------------------------------------------
TEST(SegmentFrameTest, FabricNormalMatchesSurface) {
    auto data = build_frames({"CCCC", "KKKK", "KKKK", "BBBB"});

    for (size_t i = 0; i < data.frames.size(); ++i) {
        SegmentId seg_id = static_cast<SegmentId>(i);
        if (data.surface.has_segment(seg_id)) {
            NodeId node_id = data.surface.node_for_segment(seg_id);
            const auto& node = data.surface.node(node_id);
            EXPECT_FLOAT_EQ(data.frames[i].fabric_normal.x, node.fabric_normal.x) << "Seg " << i;
            EXPECT_FLOAT_EQ(data.frames[i].fabric_normal.y, node.fabric_normal.y) << "Seg " << i;
            EXPECT_FLOAT_EQ(data.frames[i].fabric_normal.z, node.fabric_normal.z) << "Seg " << i;
        }
    }
}

// ---------------------------------------------------------------------------
// Test 4: wale_axis is orthogonal to both stitch_axis and fabric_normal
// ---------------------------------------------------------------------------
TEST(SegmentFrameTest, WaleAxisIsOrthogonal) {
    auto data = build_frames({"CCCC", "KKKK", "KKKK", "BBBB"});

    for (size_t i = 0; i < data.frames.size(); ++i) {
        const auto& f = data.frames[i];
        float dot_stitch = f.wale_axis.dot(f.stitch_axis);
        float dot_normal = f.wale_axis.dot(f.fabric_normal);
        EXPECT_NEAR(dot_stitch, 0.0f, 0.05f)
            << "Frame " << i << ": wale_axis not orthogonal to stitch_axis (dot=" << dot_stitch << ")";
        EXPECT_NEAR(dot_normal, 0.0f, 0.05f)
            << "Frame " << i << ": wale_axis not orthogonal to fabric_normal (dot=" << dot_normal << ")";
    }
}

// ---------------------------------------------------------------------------
// Test 5: Frame forms a right-handed coordinate system
// wale_axis ≈ fabric_normal × stitch_axis
// ---------------------------------------------------------------------------
TEST(SegmentFrameTest, FrameIsRightHanded) {
    auto data = build_frames({"CCCC", "KKKK", "KKKK", "BBBB"});

    for (size_t i = 0; i < data.frames.size(); ++i) {
        const auto& f = data.frames[i];
        Vec3 expected_wale = f.fabric_normal.cross(f.stitch_axis);
        float len = expected_wale.length();
        if (len < 1e-6f) continue;  // degenerate, skip
        expected_wale = expected_wale * (1.0f / len);

        float dot = f.wale_axis.dot(expected_wale);
        EXPECT_GT(dot, 0.9f)
            << "Frame " << i << ": wale_axis not consistent with fabric_normal x stitch_axis (dot=" << dot << ")";
    }
}

// ---------------------------------------------------------------------------
// Test 6: Parentless segments are adjusted Y downward below their children
// ---------------------------------------------------------------------------
TEST(SegmentFrameTest, ParentlessAdjustment) {
    auto data = build_frames({"CCCC", "KKKK", "KKKK", "BBBB"});

    const auto& segments = data.yarn_path.segments();

    // Build children map
    std::map<SegmentId, std::vector<SegmentId>> children_map;
    for (size_t i = 0; i < segments.size(); ++i) {
        for (SegmentId parent_id : segments[i].through) {
            children_map[parent_id].push_back(i);
        }
    }

    // Find segments that have no parents but do have children
    for (size_t i = 0; i < segments.size(); ++i) {
        bool has_parents = !segments[i].through.empty();
        auto children_it = children_map.find(static_cast<SegmentId>(i));
        bool has_children = (children_it != children_map.end() && !children_it->second.empty());

        if (!has_parents && has_children) {
            // This segment should have been adjusted downward
            // Its Y should be below the minimum child Y
            float min_child_y = std::numeric_limits<float>::max();
            for (SegmentId child_id : children_it->second) {
                min_child_y = std::min(min_child_y, data.frames[child_id].position.y);
            }
            EXPECT_LT(data.frames[i].position.y, min_child_y)
                << "Parentless segment " << i << " should be below its children";
        }
    }
}

// ---------------------------------------------------------------------------
// Test 7: Shape params match the surface node
// ---------------------------------------------------------------------------
TEST(SegmentFrameTest, ShapeMatchesSurface) {
    auto data = build_frames({"CCCC", "KKKK", "KKKK", "BBBB"});

    for (size_t i = 0; i < data.frames.size(); ++i) {
        SegmentId seg_id = static_cast<SegmentId>(i);
        if (data.surface.has_segment(seg_id)) {
            NodeId node_id = data.surface.node_for_segment(seg_id);
            const auto& node = data.surface.node(node_id);
            EXPECT_FLOAT_EQ(data.frames[i].shape.z_bulge, node.shape.z_bulge) << "Seg " << i;
            EXPECT_FLOAT_EQ(data.frames[i].shape.loop_height, node.shape.loop_height) << "Seg " << i;
            EXPECT_FLOAT_EQ(data.frames[i].shape.loop_width, node.shape.loop_width) << "Seg " << i;
        }
    }
}

// ---------------------------------------------------------------------------
// Test 8: Knit vs Purl segments have different z_bulge sign
// ---------------------------------------------------------------------------
TEST(SegmentFrameTest, KnitVsPurl_DifferentZBulge) {
    // Pattern with knit and purl stitches
    auto data = build_frames({"CCCC", "KKPP", "KKPP", "BBBB"});

    const auto& segments = data.yarn_path.segments();

    // Gather z_bulge values for knit-like and purl-like loop segments
    std::vector<float> knit_zbulge;
    std::vector<float> purl_zbulge;

    for (size_t i = 0; i < segments.size(); ++i) {
        if (!segments[i].forms_loop) continue;
        float zb = data.frames[i].shape.z_bulge;
        if (zb > 0) {
            knit_zbulge.push_back(zb);
        } else if (zb < 0) {
            purl_zbulge.push_back(zb);
        }
    }

    // We should have found both knit (positive z_bulge) and purl (negative z_bulge) stitches
    EXPECT_FALSE(knit_zbulge.empty()) << "Expected some knit stitches with positive z_bulge";
    EXPECT_FALSE(purl_zbulge.empty()) << "Expected some purl stitches with negative z_bulge";

    // Verify the sign difference
    for (float zb : knit_zbulge) {
        EXPECT_GT(zb, 0.0f) << "Knit z_bulge should be positive";
    }
    for (float zb : purl_zbulge) {
        EXPECT_LT(zb, 0.0f) << "Purl z_bulge should be negative";
    }
}
