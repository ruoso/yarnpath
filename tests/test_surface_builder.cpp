#include <gtest/gtest.h>
#include <surface/surface_builder.hpp>
#include <surface/surface_graph.hpp>
#include "yarn_path.hpp"
#include "stitch_node.hpp"
#include "stitch_instruction.hpp"
#include "row_instruction.hpp"
#include <cmath>
#include <map>

using namespace yarnpath;

class SurfaceBuilderTest : public ::testing::Test {
protected:
    YarnProperties yarn;
    Gauge gauge;

    void SetUp() override {
        yarn = YarnProperties::worsted();
        gauge = Gauge::worsted();
    }

    // Helper to create a simple yarn path for testing
    YarnPath create_simple_yarn_path() {
        // Create a simple 3-stitch pattern: cast on 3, then knit 3
        PatternInstructions pattern;

        RowInstruction row0;
        row0.side = RowSide::RS;
        row0.stitches = {instruction::CastOn{3}};
        pattern.rows.push_back(row0);

        RowInstruction row1;
        row1.side = RowSide::RS;
        instruction::Repeat rep;
        rep.instructions = {instruction::Knit{}};
        rep.times = 3;
        row1.stitches = {rep};
        pattern.rows.push_back(row1);

        StitchGraph graph = StitchGraph::from_instructions(pattern);
        return YarnPath::from_stitch_graph(graph, yarn, gauge);
    }

    YarnPath create_cast_on_only() {
        PatternInstructions pattern;

        RowInstruction row0;
        row0.side = RowSide::RS;
        row0.stitches = {instruction::CastOn{5}};
        pattern.rows.push_back(row0);

        StitchGraph graph = StitchGraph::from_instructions(pattern);
        return YarnPath::from_stitch_graph(graph, yarn, gauge);
    }
};

TEST_F(SurfaceBuilderTest, CreatesNodesForEachSegment) {
    YarnPath path = create_simple_yarn_path();

    SurfaceGraph graph = SurfaceBuilder::from_yarn_path(path, yarn, gauge);

    EXPECT_EQ(graph.node_count(), path.segment_count());
}

TEST_F(SurfaceBuilderTest, CreatesCorrectSegmentMapping) {
    YarnPath path = create_simple_yarn_path();

    SurfaceGraph graph = SurfaceBuilder::from_yarn_path(path, yarn, gauge);

    // Each segment should map to a node
    for (size_t i = 0; i < path.segment_count(); ++i) {
        SegmentId seg_id = static_cast<SegmentId>(i);
        EXPECT_TRUE(graph.has_segment(seg_id));

        NodeId node_id = graph.node_for_segment(seg_id);
        EXPECT_EQ(graph.node(node_id).segment_id, seg_id);
    }
}

TEST_F(SurfaceBuilderTest, CreatesContinuityEdges) {
    YarnPath path = create_cast_on_only();

    SurfaceGraph graph = SurfaceBuilder::from_yarn_path(path, yarn, gauge);

    // Should have (segment_count - 1) continuity edges
    size_t continuity_count = 0;
    for (const auto& edge : graph.edges()) {
        if (edge.type == EdgeType::YarnContinuity) {
            ++continuity_count;

            // Check that continuity edges connect consecutive segments
            EXPECT_EQ(edge.node_b, edge.node_a + 1);
        }
    }

    EXPECT_EQ(continuity_count, path.segment_count() - 1);
}

TEST_F(SurfaceBuilderTest, CreatesPassthroughEdges) {
    YarnPath path = create_simple_yarn_path();

    SurfaceGraph graph = SurfaceBuilder::from_yarn_path(path, yarn, gauge);

    // Count passthrough edges
    size_t passthrough_count = 0;
    for (const auto& edge : graph.edges()) {
        if (edge.type == EdgeType::PassThrough) {
            ++passthrough_count;
        }
    }

    // With a knit row, each knit stitch passes through a cast-on loop
    // So we should have passthrough edges
    EXPECT_GT(passthrough_count, 0);
}

TEST_F(SurfaceBuilderTest, GridBasedInitialPositions) {
    YarnPath path = create_cast_on_only();

    SurfaceBuildConfig config;
    config.random_seed = 42;
    config.position_noise = 0.1f;

    SurfaceGraph graph = SurfaceBuilder::from_yarn_path(path, yarn, gauge, config);

    // Nodes should have non-zero positions
    bool has_nonzero = false;
    for (const auto& node : graph.nodes()) {
        if (node.position.length() > 1e-6f) {
            has_nonzero = true;
            break;
        }
    }

    EXPECT_TRUE(has_nonzero);

    // Positions should be roughly in a row (small Y variation for cast-on only)
    float min_y = graph.node(0).position.y;
    float max_y = min_y;
    for (const auto& node : graph.nodes()) {
        min_y = std::min(min_y, node.position.y);
        max_y = std::max(max_y, node.position.y);
    }

    // Y spread: two-pass cast-on has foundation + return rows, so up to 2 row heights
    float y_spread = max_y - min_y;
    EXPECT_LT(y_spread, 2.0f * gauge.loop_height(yarn.compressed_radius));
}


TEST_F(SurfaceBuilderTest, SameSeedSamePositions) {
    YarnPath path = create_cast_on_only();

    SurfaceBuildConfig config;
    config.random_seed = 42;

    SurfaceGraph graph1 = SurfaceBuilder::from_yarn_path(path, yarn, gauge, config);
    SurfaceGraph graph2 = SurfaceBuilder::from_yarn_path(path, yarn, gauge, config);

    // All positions should be the same
    for (size_t i = 0; i < graph1.node_count(); ++i) {
        EXPECT_EQ(graph1.node(i).position, graph2.node(i).position);
    }
}

TEST_F(SurfaceBuilderTest, FormsLoopFlagCopied) {
    YarnPath path = create_simple_yarn_path();

    SurfaceGraph graph = SurfaceBuilder::from_yarn_path(path, yarn, gauge);

    // Check that forms_loop flags match the yarn path
    for (size_t i = 0; i < path.segment_count(); ++i) {
        SegmentId seg_id = static_cast<SegmentId>(i);
        NodeId node_id = graph.node_for_segment(seg_id);

        bool yarn_forms_loop = path.segments()[i].forms_loop;
        bool node_forms_loop = graph.node(node_id).forms_loop;

        EXPECT_EQ(node_forms_loop, yarn_forms_loop);
    }
}


TEST_F(SurfaceBuilderTest, OrientationZOffsetApplied) {
    // Create a pattern with both knit (front) and purl (back) stitches
    // Cast on 4, then knit 2, purl 2
    PatternInstructions pattern;

    RowInstruction row0;
    row0.side = RowSide::RS;
    row0.stitches = {instruction::CastOn{4}};
    pattern.rows.push_back(row0);

    RowInstruction row1;
    row1.side = RowSide::RS;
    instruction::Repeat knits;
    knits.instructions = {instruction::Knit{}};
    knits.times = 2;
    instruction::Repeat purls;
    purls.instructions = {instruction::Purl{}};
    purls.times = 2;
    row1.stitches = {knits, purls};
    pattern.rows.push_back(row1);

    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath path = YarnPath::from_stitch_graph(graph, yarn, gauge);

    // Build surface - z-offsets now come from compute_stitch_shape (yarn + gauge derived)
    SurfaceBuildConfig config;

    SurfaceGraph surface = SurfaceBuilder::from_yarn_path(path, yarn, gauge, config);

    // Collect Z positions by orientation
    bool found_front = false;
    bool found_back = false;

    for (size_t i = 0; i < path.segment_count(); ++i) {
        const auto& segment = path.segments()[i];
        SegmentId seg_id = static_cast<SegmentId>(i);
        NodeId node_id = surface.node_for_segment(seg_id);
        float node_z = surface.node(node_id).position.z;
        float expected_z = surface.node(node_id).shape.z_bulge;

        if (segment.orientation == YarnSegment::LoopOrientation::Front) {
            // Front-facing nodes should have positive z_bulge
            EXPECT_GT(expected_z, 0.0f);
            EXPECT_FLOAT_EQ(node_z, expected_z);
            found_front = true;
        } else if (segment.orientation == YarnSegment::LoopOrientation::Back) {
            // Back-facing nodes should have negative z_bulge
            EXPECT_LT(expected_z, 0.0f);
            EXPECT_FLOAT_EQ(node_z, expected_z);
            found_back = true;
        }
    }

    // Verify we found both orientations
    EXPECT_TRUE(found_front);
    EXPECT_TRUE(found_back);
}

// --- Widen loops for crossovers tests ---

TEST_F(SurfaceBuilderTest, SingleChildWidening) {
    // Cast on 2, K2: each cast-on parent has 1 child.
    // Each child forms a loop → 2 child slots per parent.
    // Each parent forms a loop → 2 parent passes.
    // Total cross-sections = 4 → min_width = 4 × compressed_diameter.
    PatternInstructions pattern;
    {
        RowInstruction row;
        row.side = RowSide::RS;
        row.stitches = {instruction::CastOn{2}};
        pattern.rows.push_back(row);
    }
    {
        RowInstruction row;
        row.side = RowSide::RS;
        instruction::Repeat rep;
        rep.instructions = {instruction::Knit{}};
        rep.times = 2;
        row.stitches = {rep};
        pattern.rows.push_back(row);
    }

    StitchGraph sg = StitchGraph::from_instructions(pattern);
    YarnPath path = YarnPath::from_stitch_graph(sg, yarn, gauge);
    SurfaceGraph graph = SurfaceBuilder::from_yarn_path(path, yarn, gauge);

    float compressed_diameter = yarn.compressed_radius * 2.0f;

    // Find parent nodes (segments with children passing through them)
    const auto& segments = path.segments();
    std::map<SegmentId, int> slots_per_parent;
    for (size_t i = 0; i < segments.size(); ++i) {
        for (SegmentId parent_id : segments[i].through) {
            slots_per_parent[parent_id] += segments[i].forms_loop ? 2 : 1;
        }
    }

    for (auto& [parent_id, num_slots] : slots_per_parent) {
        int parent_passes = segments[parent_id].forms_loop ? 2 : 1;
        int total_cross_sections = num_slots + parent_passes;
        float min_width = static_cast<float>(total_cross_sections) * compressed_diameter;

        EXPECT_GE(graph.node(parent_id).shape.loop_width, min_width)
            << "Parent node " << parent_id << " not wide enough for "
            << total_cross_sections << " cross-sections";
    }
}

TEST_F(SurfaceBuilderTest, DecreaseWidening) {
    // Cast on 3, K2tog K: the K2tog parent consumes 2 loops.
    PatternInstructions pattern;
    {
        RowInstruction row;
        row.side = RowSide::RS;
        row.stitches = {instruction::CastOn{3}};
        pattern.rows.push_back(row);
    }
    {
        RowInstruction row;
        row.side = RowSide::RS;
        row.stitches = {instruction::K2tog{}, instruction::Knit{}};
        pattern.rows.push_back(row);
    }

    StitchGraph sg = StitchGraph::from_instructions(pattern);
    YarnPath path = YarnPath::from_stitch_graph(sg, yarn, gauge);
    SurfaceGraph graph = SurfaceBuilder::from_yarn_path(path, yarn, gauge);

    float compressed_diameter = yarn.compressed_radius * 2.0f;

    const auto& segments = path.segments();
    std::map<SegmentId, int> slots_per_parent;
    for (size_t i = 0; i < segments.size(); ++i) {
        for (SegmentId parent_id : segments[i].through) {
            slots_per_parent[parent_id] += segments[i].forms_loop ? 2 : 1;
        }
    }

    // Verify widening was applied for parents with children
    for (auto& [parent_id, num_slots] : slots_per_parent) {
        int parent_passes = segments[parent_id].forms_loop ? 2 : 1;
        int total_cross_sections = num_slots + parent_passes;
        float min_width = static_cast<float>(total_cross_sections) * compressed_diameter;

        EXPECT_GE(graph.node(parent_id).shape.loop_width, min_width)
            << "Parent node " << parent_id << " with " << num_slots
            << " child slots needs widening";
    }
}

TEST_F(SurfaceBuilderTest, NoWideningNeeded) {
    // Cast on 5 — two-pass produces 5 foundation + 5 return segments.
    // Return segments (nodes 5-9) have no children, so should not be widened.
    YarnPath path = create_cast_on_only();

    auto worked_shape = compute_stitch_shape(yarn, gauge,
        LoopOrientation::Neutral, WrapDirection::None, WorkType::Worked);

    SurfaceGraph graph = SurfaceBuilder::from_yarn_path(path, yarn, gauge);

    // Return segments (no children) should have original loop_width
    for (size_t i = 5; i < graph.node_count(); ++i) {
        EXPECT_FLOAT_EQ(graph.node(i).shape.loop_width, worked_shape.loop_width)
            << "Return node " << i << " was unexpectedly widened";
    }
}

// --- Continuity edge min-width enforcement ---

TEST_F(SurfaceBuilderTest, ContinuityRestLengthMinWidth) {
    // After widen_loops_for_crossovers, continuity edge rest_length should be
    // >= half_width_a + half_width_b for adjacent widened nodes.
    YarnPath path = create_simple_yarn_path();
    SurfaceGraph graph = SurfaceBuilder::from_yarn_path(path, yarn, gauge);

    for (const auto& edge : graph.edges()) {
        if (edge.type == EdgeType::YarnContinuity) {
            float half_width_a = graph.node(edge.node_a).shape.loop_width * 0.5f;
            float half_width_b = graph.node(edge.node_b).shape.loop_width * 0.5f;
            float min_rest = half_width_a + half_width_b;

            EXPECT_GE(edge.rest_length, min_rest - 1e-6f)
                << "Continuity edge " << edge.node_a << "->" << edge.node_b
                << " rest_length=" << edge.rest_length
                << " < min_rest=" << min_rest;
        }
    }
}

// --- Passthrough edge 3D rest length ---

TEST_F(SurfaceBuilderTest, PassthroughRestLength3D) {
    // Build Cast-on + Knit pattern.
    YarnPath path = create_simple_yarn_path();
    SurfaceGraph graph = SurfaceBuilder::from_yarn_path(path, yarn, gauge);

    // Expected formula: sqrt(y² + z²) × passthrough_rest_length_factor
    float loop_height = gauge.loop_height(yarn.relaxed_radius);
    float comfortable_clearance = yarn.min_clearance() * 1.5f;
    float y_component = loop_height + comfortable_clearance;

    for (const auto& edge : graph.edges()) {
        if (edge.type == EdgeType::PassThrough) {
            // The z_component uses the parent node's z_bulge
            float parent_z_bulge = std::abs(graph.node(edge.node_b).shape.z_bulge);
            float z_component = parent_z_bulge + yarn.compressed_radius;

            float expected_rest = std::sqrt(y_component * y_component + z_component * z_component);

            // Default passthrough_rest_length_factor = 1.0
            EXPECT_NEAR(edge.rest_length, expected_rest, expected_rest * 0.01f)
                << "Passthrough edge " << edge.node_a << "->" << edge.node_b
                << " rest_length doesn't match 3D formula";
        }
    }
}

// --- Slip stitch passthrough stiffness reduction ---

TEST_F(SurfaceBuilderTest, SlipPassthroughStiffnessReduced) {
    // Build Cast-on 2, then Knit + Slip
    PatternInstructions pattern;
    {
        RowInstruction row;
        row.side = RowSide::RS;
        row.stitches = {instruction::CastOn{2}};
        pattern.rows.push_back(row);
    }
    {
        RowInstruction row;
        row.side = RowSide::RS;
        row.stitches = {instruction::Knit{}, instruction::Slip{}};
        pattern.rows.push_back(row);
    }

    StitchGraph sg = StitchGraph::from_instructions(pattern);
    YarnPath path = YarnPath::from_stitch_graph(sg, yarn, gauge);
    SurfaceGraph graph = SurfaceBuilder::from_yarn_path(path, yarn, gauge);

    const auto& segments = path.segments();

    // Find passthrough edges for knit vs slip children
    float knit_stiffness = 0.0f;
    float slip_stiffness = 0.0f;
    bool found_knit = false, found_slip = false;

    for (const auto& edge : graph.edges()) {
        if (edge.type == EdgeType::PassThrough) {
            // node_a is the child segment
            const auto& child_seg = segments[edge.node_a];
            if (child_seg.work_type == WorkType::Transferred) {
                slip_stiffness = edge.stiffness;
                found_slip = true;
            } else if (child_seg.work_type == WorkType::Worked) {
                knit_stiffness = edge.stiffness;
                found_knit = true;
            }
        }
    }

    ASSERT_TRUE(found_knit) << "No knit passthrough edge found";
    ASSERT_TRUE(found_slip) << "No slip passthrough edge found";

    // Slip stiffness should be 0.3× the knit stiffness (within tolerance for
    // different yarn densities between the two segments)
    // The 0.3 factor is applied multiplicatively to the base stiffness
    EXPECT_LT(slip_stiffness, knit_stiffness)
        << "Slip stiffness should be less than knit stiffness";

    // The ratio should be approximately 0.3 (but yarn density differences
    // between the two segments may cause some deviation)
    if (knit_stiffness > 0.0f) {
        float ratio = slip_stiffness / knit_stiffness;
        EXPECT_LT(ratio, 0.6f)
            << "Slip/knit stiffness ratio " << ratio << " should be close to 0.3";
    }
}
