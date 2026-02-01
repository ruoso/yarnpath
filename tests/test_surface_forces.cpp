#include <gtest/gtest.h>
#include <surface/surface_forces.hpp>
#include <surface/surface_graph.hpp>
#include <cmath>

using namespace yarnpath;

class SurfaceForcesTest : public ::testing::Test {
protected:
    SurfaceGraph graph;
    YarnProperties yarn;
    Gauge gauge;

    void SetUp() override {
        yarn = YarnProperties::worsted();
        gauge = Gauge::worsted();
    }

    void create_two_node_spring(float distance, float rest_length, float stiffness) {
        SurfaceNode node1, node2;
        node1.segment_id = 0;
        node1.position = Vec3(0.0f, 0.0f, 0.0f);
        node2.segment_id = 1;
        node2.position = Vec3(distance, 0.0f, 0.0f);

        graph.add_node(node1);
        graph.add_node(node2);

        SurfaceEdge edge;
        edge.node_a = 0;
        edge.node_b = 1;
        edge.type = EdgeType::YarnContinuity;
        edge.rest_length = rest_length;
        edge.stiffness = stiffness;

        graph.add_edge(edge);
    }
};

TEST_F(SurfaceForcesTest, SpringForceExtension) {
    // Spring extended beyond rest length - should pull nodes together
    create_two_node_spring(3.0f, 2.0f, 1.0f);

    compute_spring_forces(graph);

    // Node 0 should have positive x force (pulled toward node 1)
    // Node 1 should have negative x force (pulled toward node 0)
    EXPECT_GT(graph.node(0).force.x, 0.0f);
    EXPECT_LT(graph.node(1).force.x, 0.0f);

    // Forces should be equal and opposite
    EXPECT_FLOAT_EQ(graph.node(0).force.x, -graph.node(1).force.x);

    // Force magnitude should be k * displacement = 1 * (3 - 2) = 1
    EXPECT_FLOAT_EQ(graph.node(0).force.x, 1.0f);
}

TEST_F(SurfaceForcesTest, SpringForceCompression) {
    // Spring compressed below rest length - should push nodes apart
    create_two_node_spring(1.0f, 2.0f, 1.0f);

    compute_spring_forces(graph);

    // Node 0 should have negative x force (pushed away from node 1)
    // Node 1 should have positive x force (pushed away from node 0)
    EXPECT_LT(graph.node(0).force.x, 0.0f);
    EXPECT_GT(graph.node(1).force.x, 0.0f);

    // Force magnitude should be k * displacement = 1 * (1 - 2) = -1
    EXPECT_FLOAT_EQ(graph.node(0).force.x, -1.0f);
}

TEST_F(SurfaceForcesTest, SpringForceAtRest) {
    // Spring at rest length - no force
    create_two_node_spring(2.0f, 2.0f, 1.0f);

    compute_spring_forces(graph);

    EXPECT_FLOAT_EQ(graph.node(0).force.x, 0.0f);
    EXPECT_FLOAT_EQ(graph.node(1).force.x, 0.0f);
}

TEST_F(SurfaceForcesTest, SpringForceStiffness) {
    // Higher stiffness = stronger force
    create_two_node_spring(3.0f, 2.0f, 5.0f);

    compute_spring_forces(graph);

    // Force magnitude should be k * displacement = 5 * (3 - 2) = 5
    EXPECT_FLOAT_EQ(graph.node(0).force.x, 5.0f);
}

TEST_F(SurfaceForcesTest, DampingReducesForce) {
    create_two_node_spring(3.0f, 2.0f, 1.0f);

    // Give node 0 some velocity
    graph.node(0).velocity = Vec3(2.0f, 0.0f, 0.0f);

    compute_spring_forces(graph);

    apply_damping(graph, 0.5f);

    // Force should be reduced by damping * velocity
    // New force = 1.0 - 0.5 * 2.0 = 0.0
    EXPECT_FLOAT_EQ(graph.node(0).force.x, 0.0f);
}

TEST_F(SurfaceForcesTest, PassthroughTension) {
    SurfaceNode node1, node2;
    node1.segment_id = 0;
    node1.position = Vec3(0.0f, 0.0f, 0.0f);
    node2.segment_id = 1;
    node2.position = Vec3(2.0f, 0.0f, 0.0f);

    graph.add_node(node1);
    graph.add_node(node2);

    SurfaceEdge edge;
    edge.node_a = 0;
    edge.node_b = 1;
    edge.type = EdgeType::PassThrough;
    edge.rest_length = 1.0f;
    edge.stiffness = 1.0f;

    graph.add_edge(edge);

    // Set yarn tension
    yarn.tension = 0.5f;

    compute_passthrough_tension(graph, yarn, gauge, 2.0f);

    // Node 0 should be pulled toward node 1
    // Force = tension * tension_factor = 0.5 * 2.0 = 1.0
    EXPECT_FLOAT_EQ(graph.node(0).force.x, 1.0f);
    EXPECT_FLOAT_EQ(graph.node(1).force.x, -1.0f);
}

TEST_F(SurfaceForcesTest, ComputeForcesIntegration) {
    create_two_node_spring(3.0f, 2.0f, 1.0f);

    ForceConfig config;
    config.damping = 0.1f;
    config.passthrough_tension_factor = 1.0f;

    compute_forces(graph, yarn, gauge, config);

    // Forces should be computed and non-zero
    EXPECT_NE(graph.node(0).force.x, 0.0f);
    EXPECT_NE(graph.node(1).force.x, 0.0f);
}

TEST_F(SurfaceForcesTest, LoopCurvatureForce) {
    // Create 3 nodes in a line: prev, current (loop), next
    SurfaceNode prev, curr, next;
    prev.segment_id = 0;
    prev.position = Vec3(0.0f, 0.0f, 0.0f);
    curr.segment_id = 1;
    curr.position = Vec3(1.0f, 0.0f, 0.0f);  // In line, no curve
    curr.forms_loop = true;
    next.segment_id = 2;
    next.position = Vec3(2.0f, 0.0f, 0.0f);

    graph.add_node(prev);
    graph.add_node(curr);
    graph.add_node(next);

    // Continuity edges
    SurfaceEdge edge1, edge2;
    edge1.node_a = 0;
    edge1.node_b = 1;
    edge1.type = EdgeType::YarnContinuity;
    edge2.node_a = 1;
    edge2.node_b = 2;
    edge2.type = EdgeType::YarnContinuity;

    graph.add_edge(edge1);
    graph.add_edge(edge2);

    // Loop curvature should push the middle node up (to form a loop)
    compute_loop_curvature_forces(graph, yarn, gauge, 1.0f);

    // The middle node should have positive Y force (pushed up to form loop)
    EXPECT_GT(graph.node(1).force.y, 0.0f);
}
