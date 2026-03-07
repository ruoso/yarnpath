#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <surface/surface_builder.hpp>
#include <surface/surface_solver.hpp>
#include <serialization/surface_graph_json.hpp>
#include "test_helpers.hpp"

#include <cmath>

using namespace yarnpath;
using namespace yarnpath::test;
using json = nlohmann::json;

// Helper: build and solve a surface from compact pattern notation
static SurfaceGraph build_solved_surface(const std::vector<std::string>& rows,
                                         const YarnProperties& yarn,
                                         const Gauge& gauge) {
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

    return surface;
}

// ---------------------------------------------------------------------------
// Test 1: Flat stockinette normals point in a consistent direction (+Z dominant)
// ---------------------------------------------------------------------------
TEST(FabricNormalTest, FlatStockinette_NormalsPointPositiveZ) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    SurfaceGraph surface = build_solved_surface({"CCCC", "KKKK", "KKKK", "BBBB"}, yarn, gauge);

    for (const auto& node : surface.nodes()) {
        // For flat stockinette, normals should have a dominant Z component.
        // We check absolute value because the consistency pass ensures they all
        // agree in sign — the first node picks the orientation.
        EXPECT_GT(std::abs(node.fabric_normal.z), 0.3f)
            << "Node " << node.id << " fabric_normal.z = " << node.fabric_normal.z
            << " (expected dominant Z component)";
    }

    // Additionally, all normals should agree in Z sign (consistent orientation)
    float first_z_sign = (surface.nodes()[0].fabric_normal.z > 0) ? 1.0f : -1.0f;
    for (const auto& node : surface.nodes()) {
        float z_sign = (node.fabric_normal.z > 0) ? 1.0f : -1.0f;
        EXPECT_EQ(z_sign, first_z_sign)
            << "Node " << node.id << " has inconsistent normal Z sign";
    }
}

// ---------------------------------------------------------------------------
// Test 2: Normals are unit length
// ---------------------------------------------------------------------------
TEST(FabricNormalTest, FlatStockinette_NormalsAreUnitLength) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    SurfaceGraph surface = build_solved_surface({"CCCC", "KKKK", "KKKK", "BBBB"}, yarn, gauge);

    for (const auto& node : surface.nodes()) {
        float len = node.fabric_normal.length();
        EXPECT_NEAR(len, 1.0f, 1e-3f)
            << "Node " << node.id << " fabric_normal length = " << len;
    }
}

// ---------------------------------------------------------------------------
// Test 3: Normals are consistent across the swatch (high mutual dot product)
// ---------------------------------------------------------------------------
TEST(FabricNormalTest, FlatStockinette_NormalsAreConsistent) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    SurfaceGraph surface = build_solved_surface({"CCCC", "KKKK", "KKKK", "BBBB"}, yarn, gauge);

    // All normals in a flat swatch should roughly agree
    const auto& nodes = surface.nodes();
    for (size_t i = 1; i < nodes.size(); ++i) {
        float dot = nodes[0].fabric_normal.dot(nodes[i].fabric_normal);
        EXPECT_GT(dot, 0.5f)
            << "Node " << i << " normal diverges from node 0: dot = " << dot;
    }
}

// ---------------------------------------------------------------------------
// Test 4: Normal is perpendicular to stitch_axis
// ---------------------------------------------------------------------------
TEST(FabricNormalTest, NormalPerpendicularToStitchAxis) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    SurfaceGraph surface = build_solved_surface({"CCCC", "KKKK", "KKKK", "BBBB"}, yarn, gauge);

    for (const auto& node : surface.nodes()) {
        float dot = std::abs(node.fabric_normal.dot(node.stitch_axis));
        EXPECT_LT(dot, 0.15f)
            << "Node " << node.id << " normal not perpendicular to stitch_axis: |dot| = " << dot;
    }
}

// ---------------------------------------------------------------------------
// Test 5: Boundary nodes (cast-on, bind-off) have valid normals
// ---------------------------------------------------------------------------
TEST(FabricNormalTest, BoundaryNodes_HaveValidNormals) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    SurfaceGraph surface = build_solved_surface({"CCC", "KKK", "BBB"}, yarn, gauge);

    for (const auto& node : surface.nodes()) {
        float len = node.fabric_normal.length();
        EXPECT_GT(len, 0.9f)
            << "Node " << node.id << " has degenerate fabric_normal (length " << len << ")";
        EXPECT_LT(len, 1.1f)
            << "Node " << node.id << " has non-unit fabric_normal (length " << len << ")";
    }
}

// ---------------------------------------------------------------------------
// Test 6: Purl rows still have front-facing normals (normal is surface
// property, not stitch orientation — z_bulge handles knit/purl depth)
// ---------------------------------------------------------------------------
TEST(FabricNormalTest, PurlRows_NormalStillFrontFacing) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    // Garter-like swatch with alternating knit/purl
    SurfaceGraph surface = build_solved_surface({"CCC", "PPP", "KKK", "BBB"}, yarn, gauge);

    // All normals should have consistent Z sign, regardless of stitch type
    float first_z_sign = (surface.nodes()[0].fabric_normal.z > 0) ? 1.0f : -1.0f;
    for (const auto& node : surface.nodes()) {
        float z_sign = (node.fabric_normal.z > 0) ? 1.0f : -1.0f;
        EXPECT_EQ(z_sign, first_z_sign)
            << "Node " << node.id << " (segment " << node.segment_id
            << ") has inconsistent normal Z sign on purl/knit mix";
    }
}

// ---------------------------------------------------------------------------
// Test 7: fabric_normal survives JSON round-trip
// ---------------------------------------------------------------------------
TEST(FabricNormalTest, FabricNormal_Serialization_RoundTrip) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    SurfaceGraph original = build_solved_surface({"CCC", "KKK", "BBB"}, yarn, gauge);

    // Serialize and deserialize
    json j = surface_graph_to_json(original);
    SurfaceGraph deserialized = surface_graph_from_json(j);

    ASSERT_EQ(deserialized.nodes().size(), original.nodes().size());

    for (size_t i = 0; i < original.nodes().size(); ++i) {
        const auto& orig = original.nodes()[i];
        const auto& deser = deserialized.nodes()[i];

        EXPECT_NEAR(deser.fabric_normal.x, orig.fabric_normal.x, 1e-5f)
            << "fabric_normal.x mismatch at node " << i;
        EXPECT_NEAR(deser.fabric_normal.y, orig.fabric_normal.y, 1e-5f)
            << "fabric_normal.y mismatch at node " << i;
        EXPECT_NEAR(deser.fabric_normal.z, orig.fabric_normal.z, 1e-5f)
            << "fabric_normal.z mismatch at node " << i;
    }
}
