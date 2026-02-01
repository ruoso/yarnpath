#include <gtest/gtest.h>
#include <surface/surface_builder.hpp>
#include <surface/surface_graph.hpp>
#include "yarn_path.hpp"
#include "stitch_node.hpp"
#include "stitch_instruction.hpp"
#include "row_instruction.hpp"

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
        return YarnPath::from_stitch_graph(graph);
    }

    YarnPath create_cast_on_only() {
        PatternInstructions pattern;

        RowInstruction row0;
        row0.side = RowSide::RS;
        row0.stitches = {instruction::CastOn{5}};
        pattern.rows.push_back(row0);

        StitchGraph graph = StitchGraph::from_instructions(pattern);
        return YarnPath::from_stitch_graph(graph);
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

TEST_F(SurfaceBuilderTest, CreatesConstraints) {
    YarnPath path = create_simple_yarn_path();

    SurfaceGraph graph = SurfaceBuilder::from_yarn_path(path, yarn, gauge);

    // Should have constraints for both continuity (MaxStretch) and passthrough (MinDistance)
    EXPECT_GT(graph.constraint_count(), 0);

    // Check we have both types
    size_t max_stretch_count = 0;
    size_t min_distance_count = 0;
    for (const auto& constraint : graph.constraints()) {
        if (constraint.type == ConstraintType::MaxStretch) {
            max_stretch_count++;
        } else if (constraint.type == ConstraintType::MinDistance) {
            min_distance_count++;
        }
    }

    EXPECT_GT(max_stretch_count, 0);
    EXPECT_GT(min_distance_count, 0);
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

    // Y spread should be small (just noise) for a single row
    float y_spread = max_y - min_y;
    EXPECT_LT(y_spread, gauge.loop_height(yarn.compressed_radius));  // Should be less than one row height
}

TEST_F(SurfaceBuilderTest, DifferentSeedsDifferentPositions) {
    YarnPath path = create_cast_on_only();

    SurfaceBuildConfig config1;
    config1.random_seed = 42;

    SurfaceBuildConfig config2;
    config2.random_seed = 123;

    SurfaceGraph graph1 = SurfaceBuilder::from_yarn_path(path, yarn, gauge, config1);
    SurfaceGraph graph2 = SurfaceBuilder::from_yarn_path(path, yarn, gauge, config2);

    // At least one position should differ (due to noise)
    bool positions_differ = false;
    for (size_t i = 0; i < graph1.node_count(); ++i) {
        if (graph1.node(i).position != graph2.node(i).position) {
            positions_differ = true;
            break;
        }
    }

    EXPECT_TRUE(positions_differ);
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

TEST_F(SurfaceBuilderTest, RestLengthsBasedOnYarnProperties) {
    YarnPath path = create_simple_yarn_path();

    SurfaceGraph graph = SurfaceBuilder::from_yarn_path(path, yarn, gauge);

    // Passthrough edges have rest length = loop_height + min_clearance
    float loop_height = gauge.loop_height(yarn.compressed_radius);
    float expected_passthrough = loop_height + yarn.min_clearance();

    for (const auto& edge : graph.edges()) {
        if (edge.type == EdgeType::PassThrough) {
            // Rest length should be approximately loop_height + min_clearance
            EXPECT_NEAR(edge.rest_length, expected_passthrough, expected_passthrough * 0.1f);
        }
    }

    // Continuity edges have rest length based on yarn compressed_radius
    float expected_continuity = yarn.compressed_radius * 2.0f * (1.0f + (1.0f - yarn.tension) * 0.25f);

    for (const auto& edge : graph.edges()) {
        if (edge.type == EdgeType::YarnContinuity) {
            // Rest length should be approximately 2*compressed_radius with tension adjustment
            EXPECT_NEAR(edge.rest_length, expected_continuity, expected_continuity * 0.1f);
        }
    }
}
