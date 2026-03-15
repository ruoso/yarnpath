#include <gtest/gtest.h>
#include <surface/surface_solver.hpp>
#include <surface/surface_builder.hpp>
#include <surface/surface_graph.hpp>
#include "yarn_path.hpp"
#include "stitch_node.hpp"
#include "stitch_instruction.hpp"
#include "row_instruction.hpp"
#include "test_helpers.hpp"
#include <cmath>
#include <map>
#include <set>
#include <iostream>

using namespace yarnpath;
using namespace yarnpath::test;

class SurfaceSolverTest : public ::testing::Test {
protected:
    SurfaceGraph graph;
    YarnProperties yarn;

    void SetUp() override {
        yarn = YarnProperties::worsted();
    }
};

TEST_F(SurfaceSolverTest, ConvergesForSimpleSpring) {
    // Two nodes connected by a spring, starting displaced
    SurfaceNode node1, node2;
    node1.segment_id = 0;
    node1.position = Vec3(0.0f, 0.0f, 0.0f);
    node2.segment_id = 1;
    node2.position = Vec3(5.0f, 0.0f, 0.0f);  // Far apart

    graph.add_node(node1);
    graph.add_node(node2);

    SurfaceEdge edge;
    edge.node_a = 0;
    edge.node_b = 1;
    edge.type = EdgeType::YarnContinuity;
    edge.rest_length = 2.0f;
    edge.stiffness = 10.0f;

    graph.add_edge(edge);

    SolveConfig config;
    config.dt = 0.01f;
    config.max_iterations = 1000;
    config.convergence_threshold = 1e-4f;
    config.force_config.damping = 0.8f;

    Gauge gauge = Gauge::worsted();
    SolveResult result = SurfaceSolver::solve(graph, yarn, gauge, config);

    EXPECT_TRUE(result.converged);
    EXPECT_LT(result.final_energy, result.initial_energy);

    // Final distance should be close to rest length
    float final_dist = graph.node(0).position.distance_to(graph.node(1).position);
    EXPECT_NEAR(final_dist, 2.0f, 0.1f);
}

TEST_F(SurfaceSolverTest, PinnedNodeDoesNotMove) {
    SurfaceNode node1, node2;
    node1.segment_id = 0;
    node1.position = Vec3(0.0f, 0.0f, 0.0f);
    node1.is_pinned = true;
    node2.segment_id = 1;
    node2.position = Vec3(5.0f, 0.0f, 0.0f);

    graph.add_node(node1);
    graph.add_node(node2);

    SurfaceEdge edge;
    edge.node_a = 0;
    edge.node_b = 1;
    edge.type = EdgeType::YarnContinuity;
    edge.rest_length = 2.0f;
    edge.stiffness = 10.0f;

    graph.add_edge(edge);

    Vec3 original_pos = graph.node(0).position;

    SolveConfig config;
    config.max_iterations = 100;
    config.force_config.damping = 0.8f;

    Gauge gauge = Gauge::worsted();
    SurfaceSolver::solve(graph, yarn, gauge, config);

    // Pinned node should not have moved
    EXPECT_EQ(graph.node(0).position.x, original_pos.x);
    EXPECT_EQ(graph.node(0).position.y, original_pos.y);
    EXPECT_EQ(graph.node(0).position.z, original_pos.z);
}

TEST_F(SurfaceSolverTest, EnergyDecreases) {
    // Create a system that starts with high energy
    SurfaceNode node1, node2, node3;
    node1.segment_id = 0;
    node1.position = Vec3(0.0f, 0.0f, 0.0f);
    node2.segment_id = 1;
    node2.position = Vec3(10.0f, 0.0f, 0.0f);  // Very far
    node3.segment_id = 2;
    node3.position = Vec3(20.0f, 0.0f, 0.0f);  // Even further

    graph.add_node(node1);
    graph.add_node(node2);
    graph.add_node(node3);

    SurfaceEdge edge1, edge2;
    edge1.node_a = 0;
    edge1.node_b = 1;
    edge1.rest_length = 2.0f;
    edge1.stiffness = 5.0f;
    edge2.node_a = 1;
    edge2.node_b = 2;
    edge2.rest_length = 2.0f;
    edge2.stiffness = 5.0f;

    graph.add_edge(edge1);
    graph.add_edge(edge2);

    float initial_energy = graph.compute_energy();

    SolveConfig config;
    config.max_iterations = 500;
    config.force_config.damping = 0.8f;

    Gauge gauge = Gauge::worsted();
    SolveResult result = SurfaceSolver::solve(graph, yarn, gauge, config);

    EXPECT_LT(result.final_energy, initial_energy);
}

TEST_F(SurfaceSolverTest, MaxStretchConstraintEnforced) {
    SurfaceNode node1, node2;
    node1.segment_id = 0;
    node1.position = Vec3(0.0f, 0.0f, 0.0f);
    node1.is_pinned = true;  // Pin node 0
    node2.segment_id = 1;
    node2.position = Vec3(10.0f, 0.0f, 0.0f);  // Start very far

    graph.add_node(node1);
    graph.add_node(node2);

    SurfaceEdge edge;
    edge.node_a = 0;
    edge.node_b = 1;
    edge.type = EdgeType::YarnContinuity;
    edge.rest_length = 2.0f;
    edge.stiffness = 1.0f;  // Low stiffness - constraint should dominate

    graph.add_edge(edge);

    SurfaceConstraint constraint;
    constraint.type = ConstraintType::MaxStretch;
    constraint.node_a = 0;
    constraint.node_b = 1;
    constraint.limit = 3.0f;  // Max stretch to 3.0

    graph.add_constraint(constraint);

    SolveConfig config;
    config.max_iterations = 100;
    config.constraint_iterations = 5;
    config.force_config.damping = 0.9f;

    Gauge gauge = Gauge::worsted();
    SurfaceSolver::solve(graph, yarn, gauge, config);

    // Distance should be at most the constraint limit
    float final_dist = graph.node(0).position.distance_to(graph.node(1).position);
    EXPECT_LE(final_dist, 3.1f);  // Small tolerance for numerical precision
}

TEST_F(SurfaceSolverTest, SingleStepDoesNotCrash) {
    SurfaceNode node1, node2;
    node1.segment_id = 0;
    node1.position = Vec3(0.0f, 0.0f, 0.0f);
    node2.segment_id = 1;
    node2.position = Vec3(3.0f, 0.0f, 0.0f);

    graph.add_node(node1);
    graph.add_node(node2);

    SurfaceEdge edge;
    edge.node_a = 0;
    edge.node_b = 1;
    edge.rest_length = 2.0f;
    edge.stiffness = 1.0f;

    graph.add_edge(edge);

    SolveConfig config;
    Gauge gauge = Gauge::worsted();

    // Should not crash
    EXPECT_NO_THROW(SurfaceSolver::step(graph, yarn, gauge, config));

    // Positions should have changed
    EXPECT_NE(graph.node(1).position.x, 3.0f);
}

// After solving, every parent loop node must have enough space for its
// crossover slots.  The minimum width is (num_child_slots + parent_passes)
// × compressed_diameter.  We verify this by checking that consecutive
// nodes along the same row are at least that far apart in the stitch
// direction.
TEST(SurfaceSolverIntegration, CrossoverWidthSatisfiedAfterSolve) {
    YarnProperties yarn;
    yarn.relaxed_radius = 1.75f;
    yarn.compressed_radius = 0.75f;
    yarn.min_bend_radius = 2.25f;
    yarn.stiffness = 0.8f;
    yarn.elasticity = 0.3f;
    yarn.tension = 0.5f;
    Gauge gauge{5.0f};
    float compressed_diameter = yarn.compressed_radius * 2.0f;

    // Build a 3-row stockinette (CCC / KKK / BBB)
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
        instruction::Repeat rep;
        rep.instructions = {instruction::Knit{}};
        rep.times = 3;
        row.stitches = {rep};
        pattern.rows.push_back(row);
    }
    {
        RowInstruction row;
        row.side = RowSide::RS;
        row.stitches = {instruction::BindOff{3}};
        pattern.rows.push_back(row);
    }

    StitchGraph stitch_graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(stitch_graph, yarn, gauge);

    SurfaceBuildConfig build_config;
    build_config.random_seed = 42;
    SurfaceGraph surface = SurfaceBuilder::from_yarn_path(yarn_path, yarn, gauge, build_config);

    SolveConfig solve_config;
    solve_config.max_iterations = 1000;
    solve_config.convergence_threshold = 1e-4f;
    SurfaceSolver::solve(surface, yarn, gauge, solve_config);

    const auto& segments = yarn_path.segments();

    // Count crossover slots per parent (same logic as widen_loops_for_crossovers)
    std::map<SegmentId, int> slots_per_parent;
    for (size_t i = 0; i < segments.size(); ++i) {
        for (SegmentId parent_id : segments[i].through) {
            slots_per_parent[parent_id] += segments[i].forms_loop ? 2 : 1;
        }
    }

    // For each parent with children, check that the solved loop_width
    // (distance between continuity neighbors along the stitch axis)
    // is at least the required minimum.
    int violations = 0;
    for (auto& [parent_id, num_slots] : slots_per_parent) {
        int parent_passes = segments[parent_id].forms_loop ? 2 : 1;
        int total_cross_sections = num_slots + parent_passes;
        float min_width = static_cast<float>(total_cross_sections) * compressed_diameter;

        // Find the continuity neighbors (prev and next in yarn order)
        // Only check neighbors that are actually different nodes
        float available = std::numeric_limits<float>::max();

        if (parent_id > 0) {
            float dist_prev = (surface.node(parent_id).position -
                               surface.node(parent_id - 1).position).length();
            available = std::min(available, dist_prev);
        }
        if (parent_id + 1 < static_cast<SegmentId>(surface.node_count())) {
            float dist_next = (surface.node(parent_id + 1).position -
                               surface.node(parent_id).position).length();
            available = std::min(available, dist_next);
        }

        if (available == std::numeric_limits<float>::max()) continue;

        // Allow 10% tolerance for numerical relaxation
        if (available < min_width * 0.9f) {
            violations++;
            EXPECT_GE(available, min_width * 0.9f)
                << "Node " << parent_id
                << " needs " << total_cross_sections << " cross-sections"
                << " (" << num_slots << " slots + " << parent_passes << " parent passes)"
                << " → min_width=" << min_width
                << " but available=" << available;
        }
    }
    EXPECT_EQ(violations, 0)
        << "Found " << violations << " nodes with insufficient crossover width";
}

// --- Fabric normal tests ---

static SurfaceGraph build_and_solve_stockinette_2row(
    const YarnProperties& yarn, const Gauge& gauge) {
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
        instruction::Repeat rep;
        rep.instructions = {instruction::Knit{}};
        rep.times = 3;
        row.stitches = {rep};
        pattern.rows.push_back(row);
    }

    StitchGraph sg = StitchGraph::from_instructions(pattern);
    YarnPath path = YarnPath::from_stitch_graph(sg, yarn, gauge);

    SurfaceBuildConfig build_config;
    SurfaceGraph surface = SurfaceBuilder::from_yarn_path(path, yarn, gauge, build_config);

    SolveConfig solve_config;
    solve_config.max_iterations = 500;
    solve_config.convergence_threshold = 1e-4f;
    SurfaceSolver::solve(surface, yarn, gauge, solve_config);

    return surface;
}

TEST(SurfaceSolverFabricNormal, FabricNormalComputedAfterSolve) {
    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();

    SurfaceGraph surface = build_and_solve_stockinette_2row(yarn, gauge);

    // All nodes should have non-zero fabric_normal after solve
    for (size_t i = 0; i < surface.node_count(); ++i) {
        Vec3 normal = surface.node(i).fabric_normal;
        EXPECT_GT(normal.length(), 0.5f)
            << "Node " << i << " has near-zero fabric_normal";
    }
}

TEST(SurfaceSolverFabricNormal, FabricNormalConsistentDirection) {
    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();

    SurfaceGraph surface = build_and_solve_stockinette_2row(yarn, gauge);

    // All fabric normals should point in the same hemisphere
    // (consistent orientation propagation)
    Vec3 ref_normal = surface.node(0).fabric_normal;
    for (size_t i = 1; i < surface.node_count(); ++i) {
        float dot = surface.node(i).fabric_normal.dot(ref_normal);
        EXPECT_GT(dot, 0.0f)
            << "Node " << i << " fabric_normal is in opposite hemisphere from node 0";
    }
}

TEST(SurfaceSolverFabricNormal, FabricNormalPerpendicularity) {
    YarnProperties yarn = YarnProperties::worsted();
    Gauge gauge = Gauge::worsted();

    SurfaceGraph surface = build_and_solve_stockinette_2row(yarn, gauge);

    // For interior nodes, fabric_normal should be roughly perpendicular to stitch_axis
    // (dot product close to 0)
    for (size_t i = 1; i + 1 < surface.node_count(); ++i) {
        Vec3 normal = surface.node(i).fabric_normal;
        Vec3 axis = surface.node(i).stitch_axis;
        float dot = std::abs(normal.dot(axis));

        // Allow generous tolerance since stitch_axis and normal are derived independently
        EXPECT_LT(dot, 0.5f)
            << "Node " << i << " fabric_normal not perpendicular to stitch_axis"
            << " (|dot|=" << dot << ")";
    }
}

// ---------------------------------------------------------------------------
// Helper: build full pipeline (pattern → solve) returning both surface and
// yarn_path so tests can use PassThrough topology.
// ---------------------------------------------------------------------------
struct SolverTestData {
    YarnPath yarn_path;
    SurfaceGraph surface;
};

static SolverTestData build_and_solve(const std::vector<std::string>& rows) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    PatternInstructions pattern = create_pattern(rows);
    StitchGraph sg = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(sg, yarn, gauge);

    SurfaceBuildConfig build_config;
    build_config.random_seed = 42;
    SurfaceGraph surface = SurfaceBuilder::from_yarn_path(yarn_path, yarn, gauge, build_config);

    SolveConfig solve_config;
    solve_config.max_iterations = 1000;
    solve_config.convergence_threshold = 1e-4f;
    SurfaceSolver::solve(surface, yarn, gauge, solve_config);

    return SolverTestData{std::move(yarn_path), std::move(surface)};
}

// ---------------------------------------------------------------------------
// Physical invariant: stitch_axis should be perpendicular to the
// parent→child (wale) direction derived from PassThrough topology.
// Uses 3 rows so the middle row has both parents and children.
// ---------------------------------------------------------------------------
TEST(SurfaceSolverFabricNormal, StitchAxisPerpendicularToWaleAxis) {
    auto data = build_and_solve({"CCCC", "KKKK", "KKKK"});
    const auto& surface = data.surface;

    // Every node's stitch_axis should be perpendicular to its stored wale_axis.
    // The wale_axis is computed from PassThrough topology in the solver,
    // and stitch_axis is orthogonalized against it.
    int checked = 0;
    for (size_t i = 0; i < surface.node_count(); ++i) {
        const auto& node = surface.node(i);
        float dot = std::abs(node.stitch_axis.dot(node.wale_axis));
        EXPECT_LT(dot, 0.01f)
            << "Node " << i << " stitch_axis not perpendicular to wale_axis"
            << " (|dot|=" << dot << ")";
        checked++;
    }
    EXPECT_GT(checked, 0);
}

// ---------------------------------------------------------------------------
// Physical invariant: for flat stockinette, all fabric normals should be
// closely aligned (not just in the same hemisphere — actually similar).
// ---------------------------------------------------------------------------
TEST(SurfaceSolverFabricNormal, FabricNormalsCloselyAlignedForFlatFabric) {
    auto data = build_and_solve({"CCCC", "KKKK", "KKKK"});
    const auto& surface = data.surface;

    // Compute average normal
    Vec3 avg_normal = Vec3::zero();
    for (size_t i = 0; i < surface.node_count(); ++i) {
        avg_normal += surface.node(i).fabric_normal;
    }
    avg_normal = avg_normal * (1.0f / surface.node_count());
    float avg_len = avg_normal.length();
    ASSERT_GT(avg_len, 0.1f) << "Average normal is near-zero (normals cancel out)";
    avg_normal = avg_normal * (1.0f / avg_len);

    // Debug: print all node frames for diagnosis
    for (size_t i = 0; i < surface.node_count(); ++i) {
        const auto& n = surface.node(i);
        std::cerr << "  Node " << i << " seg=" << n.segment_id
            << " pos=(" << n.position.x << "," << n.position.y << "," << n.position.z << ")"
            << " fn=(" << n.fabric_normal.x << "," << n.fabric_normal.y << "," << n.fabric_normal.z << ")"
            << " sa=(" << n.stitch_axis.x << "," << n.stitch_axis.y << "," << n.stitch_axis.z << ")"
            << " wa=(" << n.wale_axis.x << "," << n.wale_axis.y << "," << n.wale_axis.z << ")"
            << " fn·avg=" << n.fabric_normal.dot(avg_normal)
            << "\n";
    }
    std::cerr << "  Edges: ";
    for (size_t i = 0; i < surface.edge_count(); ++i) {
        const auto& e = surface.edge(i);
        std::cerr << e.node_a << (e.type == EdgeType::PassThrough ? "-PT->" : "-YC->") << e.node_b << " ";
    }
    std::cerr << "\n";

    // Each node's normal should be close to the average
    for (size_t i = 0; i < surface.node_count(); ++i) {
        float dot = surface.node(i).fabric_normal.dot(avg_normal);
        EXPECT_GT(dot, 0.8f)
            << "Node " << i << " fabric_normal deviates too much from average"
            << " (dot=" << dot << ")";
    }
}

// ---------------------------------------------------------------------------
// Physical invariant: the stored wale_axis should point from parents toward
// children (positive projection onto the parent→child vector from PassThrough
// topology).
// ---------------------------------------------------------------------------
TEST(SurfaceSolverFabricNormal, WaleAxisPointsParentToChild) {
    auto data = build_and_solve({"CCCC", "KKKK", "KKKK"});
    const auto& surface = data.surface;

    // Build parent→children map from PassThrough edges
    std::map<NodeId, std::vector<NodeId>> node_children;
    for (const auto& edge : surface.edges()) {
        if (edge.type == EdgeType::PassThrough) {
            node_children[edge.node_b].push_back(edge.node_a);
        }
    }

    int checked = 0;
    for (const auto& [parent_id, children] : node_children) {
        Vec3 parent_pos = surface.node(parent_id).position;
        Vec3 avg_child = Vec3::zero();
        for (NodeId cid : children) avg_child += surface.node(cid).position;
        avg_child = avg_child * (1.0f / children.size());

        Vec3 parent_to_child = avg_child - parent_pos;
        float len = parent_to_child.length();
        if (len < 1e-4f) continue;

        // The stored wale_axis should point toward children
        float proj = surface.node(parent_id).wale_axis.dot(parent_to_child);
        EXPECT_GT(proj, 0.0f)
            << "Node " << parent_id << " wale_axis points away from children"
            << " (proj=" << proj << ")";
        checked++;
    }
    EXPECT_GT(checked, 0) << "No parent-child pairs found via PassThrough edges";
}
