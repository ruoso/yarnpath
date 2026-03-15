#include <gtest/gtest.h>
#include <geometry/position_resolver.hpp>
#include <surface/surface_builder.hpp>
#include <surface/surface_solver.hpp>
#include "test_helpers.hpp"

#include <cmath>
#include <map>

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
// Test 5: The three frame axes are mutually orthogonal
// wale_axis is topology-derived (always parent→child) and independent of
// stitch_axis sign, so fabric_normal × stitch_axis may differ in sign from
// wale_axis on WS rows.  We check that wale_axis lies in the plane
// perpendicular to stitch_axis (which it shares with fabric_normal × stitch_axis,
// up to a sign).
// ---------------------------------------------------------------------------
TEST(SegmentFrameTest, FrameAxesMutuallyOrthogonal) {
    auto data = build_frames({"CCCC", "KKKK", "KKKK", "BBBB"});

    for (size_t i = 0; i < data.frames.size(); ++i) {
        const auto& f = data.frames[i];
        // wale ⊥ stitch
        EXPECT_NEAR(std::abs(f.wale_axis.dot(f.stitch_axis)), 0.0f, 0.05f)
            << "Frame " << i << ": wale_axis not perpendicular to stitch_axis";
        // wale ⊥ normal
        EXPECT_NEAR(std::abs(f.wale_axis.dot(f.fabric_normal)), 0.0f, 0.05f)
            << "Frame " << i << ": wale_axis not perpendicular to fabric_normal";
        // normal ⊥ stitch
        EXPECT_NEAR(std::abs(f.fabric_normal.dot(f.stitch_axis)), 0.0f, 0.05f)
            << "Frame " << i << ": fabric_normal not perpendicular to stitch_axis";
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

    // Categorize by actual stitch type (LoopOrientation), not by z_bulge sign
    std::vector<float> knit_zbulge;
    std::vector<float> purl_zbulge;

    for (size_t i = 0; i < segments.size(); ++i) {
        if (!segments[i].forms_loop) continue;
        float zb = data.frames[i].shape.z_bulge;
        if (segments[i].orientation == LoopOrientation::Front) {
            knit_zbulge.push_back(zb);
        } else if (segments[i].orientation == LoopOrientation::Back) {
            purl_zbulge.push_back(zb);
        }
    }

    EXPECT_FALSE(knit_zbulge.empty()) << "Expected Front-oriented (knit) loops";
    EXPECT_FALSE(purl_zbulge.empty()) << "Expected Back-oriented (purl) loops";

    // Knit (Front) should have positive z_bulge, purl (Back) negative
    for (float zb : knit_zbulge) {
        EXPECT_GT(zb, 0.0f) << "Knit (Front) z_bulge should be positive";
    }
    for (float zb : purl_zbulge) {
        EXPECT_LT(zb, 0.0f) << "Purl (Back) z_bulge should be negative";
    }
}

// ---------------------------------------------------------------------------
// Test 9: All frame axes are unit length (orthonormal, not just orthogonal)
// ---------------------------------------------------------------------------
TEST(SegmentFrameTest, AxesAreUnitLength) {
    auto data = build_frames({"CCCC", "KKKK"});

    for (size_t i = 0; i < data.frames.size(); ++i) {
        const auto& f = data.frames[i];
        EXPECT_NEAR(f.stitch_axis.length(), 1.0f, 0.01f)
            << "Frame " << i << ": stitch_axis not unit length";
        EXPECT_NEAR(f.fabric_normal.length(), 1.0f, 0.01f)
            << "Frame " << i << ": fabric_normal not unit length";
        EXPECT_NEAR(f.wale_axis.length(), 1.0f, 0.01f)
            << "Frame " << i << ": wale_axis not unit length";
    }
}

// ---------------------------------------------------------------------------
// Test 10: Fabric normals are consistent across flat stockinette
// For a single-layer flat fabric, all fabric normals should point in
// roughly the same direction.
// ---------------------------------------------------------------------------
TEST(SegmentFrameTest, FabricNormalsConsistentForFlatFabric) {
    auto data = build_frames({"CCCC", "KKKK", "KKKK"});

    // Compute average fabric normal
    Vec3 avg_normal = Vec3::zero();
    int count = 0;
    for (const auto& f : data.frames) {
        avg_normal += f.fabric_normal;
        count++;
    }
    ASSERT_GT(count, 0);
    avg_normal = avg_normal * (1.0f / count);
    float avg_len = avg_normal.length();
    ASSERT_GT(avg_len, 0.1f) << "Average normal near zero — normals are not consistent";
    avg_normal = avg_normal * (1.0f / avg_len);

    // Each normal should be roughly aligned with the average
    for (size_t i = 0; i < data.frames.size(); ++i) {
        float dot = data.frames[i].fabric_normal.dot(avg_normal);
        EXPECT_GT(dot, 0.8f)
            << "Frame " << i << ": fabric_normal deviates significantly from average"
            << " (dot=" << dot << ")";
    }
}

// ---------------------------------------------------------------------------
// Test 11: Stitch axis is perpendicular to the parent→child direction
// The stitch_axis (course direction) should be roughly perpendicular to
// the wale direction. We derive the physical wale direction from the
// parent→child position vector rather than trusting the frame's wale_axis.
// ---------------------------------------------------------------------------
TEST(SegmentFrameTest, StitchAxisPerpendicularToWaleDirection) {
    auto data = build_frames({"CCCC", "KKKK"});
    const auto& segments = data.yarn_path.segments();

    // Build children map
    std::map<SegmentId, std::vector<SegmentId>> children_map;
    for (size_t i = 0; i < segments.size(); ++i) {
        for (SegmentId pid : segments[i].through) {
            children_map[pid].push_back(static_cast<SegmentId>(i));
        }
    }

    int checked = 0;
    for (const auto& [parent_id, children] : children_map) {
        Vec3 parent_pos = data.frames[parent_id].position;
        Vec3 avg_child = Vec3::zero();
        for (SegmentId cid : children) avg_child += data.frames[cid].position;
        avg_child = avg_child * (1.0f / children.size());

        Vec3 wale_dir = avg_child - parent_pos;
        float len = wale_dir.length();
        if (len < 1e-4f) continue;
        wale_dir = wale_dir * (1.0f / len);

        // Parent's stitch_axis should be roughly perpendicular to wale
        float dot = std::abs(data.frames[parent_id].stitch_axis.dot(wale_dir));
        EXPECT_LT(dot, 0.4f)
            << "Segment " << parent_id
            << ": stitch_axis should be perpendicular to parent→child direction"
            << " (|dot|=" << dot << ")";

        // Children's stitch axes should also be roughly perpendicular
        for (SegmentId cid : children) {
            float cdot = std::abs(data.frames[cid].stitch_axis.dot(wale_dir));
            EXPECT_LT(cdot, 0.4f)
                << "Segment " << cid << " (child of " << parent_id
                << "): stitch_axis should be perpendicular to wale direction"
                << " (|dot|=" << cdot << ")";
        }
        checked++;
    }
    EXPECT_GT(checked, 0) << "Expected at least one parent with children";
}

// ---------------------------------------------------------------------------
// Test 12: Wale axis aligns with the parent-to-child direction
// The wale axis represents the row-stacking direction. For any loop that
// has children, the vector from the parent's position to its children's
// average position defines the physical wale direction. The frame's
// wale_axis should have a positive projection onto this direction.
// ---------------------------------------------------------------------------
TEST(SegmentFrameTest, WaleAxisPointsParentToChild) {
    auto data = build_frames({"CCCC", "KKKK"});
    const auto& segments = data.yarn_path.segments();

    // Build children map
    std::map<SegmentId, std::vector<SegmentId>> children_map;
    for (size_t i = 0; i < segments.size(); ++i) {
        for (SegmentId pid : segments[i].through) {
            children_map[pid].push_back(static_cast<SegmentId>(i));
        }
    }

    int checked = 0;
    for (const auto& [parent_id, children] : children_map) {
        if (children.empty()) continue;

        Vec3 parent_pos = data.frames[parent_id].position;
        Vec3 avg_child = Vec3::zero();
        for (SegmentId cid : children) avg_child += data.frames[cid].position;
        avg_child = avg_child * (1.0f / children.size());

        Vec3 parent_to_child = avg_child - parent_pos;
        float len = parent_to_child.length();
        if (len < 1e-4f) continue;

        float proj = parent_to_child.dot(data.frames[parent_id].wale_axis);
        EXPECT_GT(proj, 0.0f)
            << "Segment " << parent_id << ": wale_axis should point toward children"
            << " (proj=" << proj << ", |parent_to_child|=" << len << ")";
        checked++;
    }
    EXPECT_GT(checked, 0) << "Expected at least one loop with children";
}

// ---------------------------------------------------------------------------
// Test 13: Children are further along wale than their parents
// For every parent→child relationship, the child's position should have a
// larger wale projection than the parent's, using the parent's own wale_axis.
// This is the per-edge version of "row progression."
// ---------------------------------------------------------------------------
TEST(SegmentFrameTest, ChildrenFurtherAlongWaleThanParents) {
    auto data = build_frames({"CCCC", "KKKK", "KKKK"});
    const auto& segments = data.yarn_path.segments();

    int checked = 0;
    for (size_t i = 0; i < segments.size(); ++i) {
        for (SegmentId pid : segments[i].through) {
            Vec3 wale = data.frames[pid].wale_axis;
            float parent_proj = data.frames[pid].position.dot(wale);
            float child_proj = data.frames[i].position.dot(wale);
            EXPECT_GT(child_proj, parent_proj)
                << "Child " << i << " should be further along wale than parent " << pid
                << " (child_proj=" << child_proj << ", parent_proj=" << parent_proj << ")";
            checked++;
        }
    }
    EXPECT_GT(checked, 0) << "Expected at least one parent-child pair";
}
