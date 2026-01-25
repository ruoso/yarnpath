#include <gtest/gtest.h>
#include <surface/surface_solver.hpp>
#include <surface/surface_graph.hpp>
#include <cmath>

using namespace yarnpath;

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

    SolveResult result = SurfaceSolver::solve(graph, yarn, config);

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

    SurfaceSolver::solve(graph, yarn, config);

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

    SolveResult result = SurfaceSolver::solve(graph, yarn, config);

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

    SurfaceSolver::solve(graph, yarn, config);

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

    // Should not crash
    EXPECT_NO_THROW(SurfaceSolver::step(graph, yarn, config));

    // Positions should have changed
    EXPECT_NE(graph.node(1).position.x, 3.0f);
}
