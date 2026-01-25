#include <gtest/gtest.h>
#include <surface/surface_graph.hpp>

using namespace yarnpath;

TEST(SurfaceGraph, AddAndRetrieveNode) {
    SurfaceGraph graph;

    SurfaceNode node;
    node.segment_id = 42;
    node.position = Vec3(1.0f, 2.0f, 3.0f);
    node.forms_loop = true;

    NodeId id = graph.add_node(node);
    EXPECT_EQ(id, 0);
    EXPECT_EQ(graph.node_count(), 1);

    const auto& retrieved = graph.node(id);
    EXPECT_EQ(retrieved.id, 0);
    EXPECT_EQ(retrieved.segment_id, 42);
    EXPECT_EQ(retrieved.position.x, 1.0f);
    EXPECT_EQ(retrieved.position.y, 2.0f);
    EXPECT_EQ(retrieved.position.z, 3.0f);
    EXPECT_TRUE(retrieved.forms_loop);
}

TEST(SurfaceGraph, AddMultipleNodes) {
    SurfaceGraph graph;

    for (int i = 0; i < 5; ++i) {
        SurfaceNode node;
        node.segment_id = static_cast<SegmentId>(i);
        graph.add_node(node);
    }

    EXPECT_EQ(graph.node_count(), 5);

    for (int i = 0; i < 5; ++i) {
        EXPECT_EQ(graph.node(i).segment_id, static_cast<SegmentId>(i));
    }
}

TEST(SurfaceGraph, AddAndRetrieveEdge) {
    SurfaceGraph graph;

    SurfaceNode node1, node2;
    node1.segment_id = 0;
    node2.segment_id = 1;
    graph.add_node(node1);
    graph.add_node(node2);

    SurfaceEdge edge;
    edge.node_a = 0;
    edge.node_b = 1;
    edge.type = EdgeType::YarnContinuity;
    edge.rest_length = 5.0f;
    edge.stiffness = 10.0f;

    EdgeId id = graph.add_edge(edge);
    EXPECT_EQ(id, 0);
    EXPECT_EQ(graph.edge_count(), 1);

    const auto& retrieved = graph.edge(id);
    EXPECT_EQ(retrieved.node_a, 0);
    EXPECT_EQ(retrieved.node_b, 1);
    EXPECT_EQ(retrieved.type, EdgeType::YarnContinuity);
    EXPECT_FLOAT_EQ(retrieved.rest_length, 5.0f);
    EXPECT_FLOAT_EQ(retrieved.stiffness, 10.0f);
}

TEST(SurfaceGraph, AddAndRetrieveConstraint) {
    SurfaceGraph graph;

    SurfaceNode node1, node2;
    node1.segment_id = 0;
    node2.segment_id = 1;
    graph.add_node(node1);
    graph.add_node(node2);

    SurfaceConstraint constraint;
    constraint.type = ConstraintType::MaxStretch;
    constraint.node_a = 0;
    constraint.node_b = 1;
    constraint.limit = 7.5f;

    ConstraintId id = graph.add_constraint(constraint);
    EXPECT_EQ(id, 0);
    EXPECT_EQ(graph.constraint_count(), 1);

    const auto& retrieved = graph.constraint(id);
    EXPECT_EQ(retrieved.type, ConstraintType::MaxStretch);
    EXPECT_EQ(retrieved.node_a, 0);
    EXPECT_EQ(retrieved.node_b, 1);
    EXPECT_FLOAT_EQ(retrieved.limit, 7.5f);
}

TEST(SurfaceGraph, NodeForSegment) {
    SurfaceGraph graph;

    SurfaceNode node1, node2, node3;
    node1.segment_id = 10;
    node2.segment_id = 20;
    node3.segment_id = 30;

    graph.add_node(node1);
    graph.add_node(node2);
    graph.add_node(node3);

    EXPECT_EQ(graph.node_for_segment(10), 0);
    EXPECT_EQ(graph.node_for_segment(20), 1);
    EXPECT_EQ(graph.node_for_segment(30), 2);

    EXPECT_TRUE(graph.has_segment(10));
    EXPECT_TRUE(graph.has_segment(20));
    EXPECT_TRUE(graph.has_segment(30));
    EXPECT_FALSE(graph.has_segment(40));
}

TEST(SurfaceGraph, EdgesForNode) {
    SurfaceGraph graph;

    // Create 3 nodes
    for (int i = 0; i < 3; ++i) {
        SurfaceNode node;
        node.segment_id = static_cast<SegmentId>(i);
        graph.add_node(node);
    }

    // Create edges: 0-1, 1-2
    SurfaceEdge edge1, edge2;
    edge1.node_a = 0;
    edge1.node_b = 1;
    edge2.node_a = 1;
    edge2.node_b = 2;

    graph.add_edge(edge1);
    graph.add_edge(edge2);

    auto edges_for_0 = graph.edges_for_node(0);
    EXPECT_EQ(edges_for_0.size(), 1);
    EXPECT_EQ(edges_for_0[0], 0);

    auto edges_for_1 = graph.edges_for_node(1);
    EXPECT_EQ(edges_for_1.size(), 2);

    auto edges_for_2 = graph.edges_for_node(2);
    EXPECT_EQ(edges_for_2.size(), 1);
    EXPECT_EQ(edges_for_2[0], 1);
}

TEST(SurfaceGraph, ComputeEnergy) {
    SurfaceGraph graph;

    // Two nodes at distance 3, spring with rest length 2, stiffness 1
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

    // Energy = 0.5 * k * (x - rest)^2 = 0.5 * 1 * (3 - 2)^2 = 0.5
    float energy = graph.compute_energy();
    EXPECT_FLOAT_EQ(energy, 0.5f);
}

TEST(SurfaceGraph, ClearAllForces) {
    SurfaceGraph graph;

    SurfaceNode node;
    node.segment_id = 0;
    node.force = Vec3(1.0f, 2.0f, 3.0f);
    graph.add_node(node);

    EXPECT_NE(graph.node(0).force.x, 0.0f);

    graph.clear_all_forces();

    EXPECT_FLOAT_EQ(graph.node(0).force.x, 0.0f);
    EXPECT_FLOAT_EQ(graph.node(0).force.y, 0.0f);
    EXPECT_FLOAT_EQ(graph.node(0).force.z, 0.0f);
}
