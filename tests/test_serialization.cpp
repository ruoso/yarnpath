#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <serialization/surface_graph_json.hpp>
#include <surface/surface_builder.hpp>
#include <surface/surface_solver.hpp>
#include "test_helpers.hpp"

using namespace yarnpath;
using namespace yarnpath::test;
using json = nlohmann::json;

// ---------------------------------------------------------------------------
// StitchShapeParams serialization
// ---------------------------------------------------------------------------

TEST(SerializationTest, LoopShapeParams_Serialization_AllFields) {
    StitchShapeParams original;
    original.z_bulge = -2.5f;
    original.loop_height = 3.14f;
    original.loop_width = 2.71f;
    original.z_bulge_factor = -1.0f;
    original.apex_lean_x = 0.3f;
    original.apex_height_factor = 0.8f;
    original.symmetric_exit = false;
    original.entry_tangent_scale = 1.2f;
    original.width_multiplier = 1.4f;
    original.height_multiplier = 1.8f;

    json j = original;
    StitchShapeParams deserialized = j.get<StitchShapeParams>();

    EXPECT_NEAR(deserialized.z_bulge, original.z_bulge, 1e-6f);
    EXPECT_NEAR(deserialized.loop_height, original.loop_height, 1e-6f);
    EXPECT_NEAR(deserialized.loop_width, original.loop_width, 1e-6f);
    EXPECT_NEAR(deserialized.z_bulge_factor, original.z_bulge_factor, 1e-6f);
    EXPECT_NEAR(deserialized.apex_lean_x, original.apex_lean_x, 1e-6f);
    EXPECT_NEAR(deserialized.apex_height_factor, original.apex_height_factor, 1e-6f);
    EXPECT_EQ(deserialized.symmetric_exit, original.symmetric_exit);
    EXPECT_NEAR(deserialized.entry_tangent_scale, original.entry_tangent_scale, 1e-6f);
    EXPECT_NEAR(deserialized.width_multiplier, original.width_multiplier, 1e-6f);
    EXPECT_NEAR(deserialized.height_multiplier, original.height_multiplier, 1e-6f);
}

TEST(SerializationTest, LoopShapeParams_DefaultValues_RoundTrip) {
    StitchShapeParams original;  // all defaults

    json j = original;
    StitchShapeParams deserialized = j.get<StitchShapeParams>();

    EXPECT_NEAR(deserialized.z_bulge, 0.0f, 1e-6f);
    EXPECT_NEAR(deserialized.loop_height, 0.0f, 1e-6f);
    EXPECT_NEAR(deserialized.loop_width, 0.0f, 1e-6f);
    EXPECT_NEAR(deserialized.z_bulge_factor, 1.0f, 1e-6f);
    EXPECT_NEAR(deserialized.apex_lean_x, 0.0f, 1e-6f);
    EXPECT_NEAR(deserialized.apex_height_factor, 1.0f, 1e-6f);
    EXPECT_TRUE(deserialized.symmetric_exit);
    EXPECT_NEAR(deserialized.entry_tangent_scale, 1.0f, 1e-6f);
    EXPECT_NEAR(deserialized.width_multiplier, 1.0f, 1e-6f);
    EXPECT_NEAR(deserialized.height_multiplier, 1.0f, 1e-6f);
}

// ---------------------------------------------------------------------------
// SurfaceNode stitch_axis + shape serialization
// ---------------------------------------------------------------------------

TEST(SerializationTest, SurfaceNode_StitchAxis_RoundTrip) {
    SurfaceNode original;
    original.id = 7;
    original.segment_id = 3;
    original.position = Vec3(1.0f, 2.0f, 3.0f);
    original.velocity = Vec3::zero();
    original.force = Vec3::zero();
    original.mass = 0.5f;
    original.is_pinned = false;
    original.forms_loop = true;
    original.stitch_axis = Vec3(0.6f, 0.8f, 0.0f);

    json j = original;
    SurfaceNode deserialized = j.get<SurfaceNode>();

    EXPECT_NEAR(deserialized.stitch_axis.x, 0.6f, 1e-6f);
    EXPECT_NEAR(deserialized.stitch_axis.y, 0.8f, 1e-6f);
    EXPECT_NEAR(deserialized.stitch_axis.z, 0.0f, 1e-6f);
}

TEST(SerializationTest, SurfaceNode_Shape_RoundTrip) {
    SurfaceNode original;
    original.id = 5;
    original.segment_id = 2;
    original.position = Vec3(0.0f, 1.0f, 0.0f);
    original.velocity = Vec3::zero();
    original.force = Vec3::zero();
    original.mass = 1.0f;
    original.is_pinned = false;
    original.forms_loop = true;

    original.shape.z_bulge = -2.5f;
    original.shape.loop_height = 3.14f;
    original.shape.loop_width = 2.71f;
    original.shape.z_bulge_factor = -1.0f;
    original.shape.apex_lean_x = 0.3f;
    original.shape.apex_height_factor = 0.8f;
    original.shape.symmetric_exit = false;
    original.shape.entry_tangent_scale = 1.2f;
    original.shape.width_multiplier = 1.4f;
    original.shape.height_multiplier = 1.8f;

    json j = original;
    SurfaceNode deserialized = j.get<SurfaceNode>();

    EXPECT_NEAR(deserialized.shape.z_bulge, original.shape.z_bulge, 1e-6f);
    EXPECT_NEAR(deserialized.shape.loop_height, original.shape.loop_height, 1e-6f);
    EXPECT_NEAR(deserialized.shape.loop_width, original.shape.loop_width, 1e-6f);
    EXPECT_NEAR(deserialized.shape.z_bulge_factor, original.shape.z_bulge_factor, 1e-6f);
    EXPECT_NEAR(deserialized.shape.apex_lean_x, original.shape.apex_lean_x, 1e-6f);
    EXPECT_NEAR(deserialized.shape.apex_height_factor, original.shape.apex_height_factor, 1e-6f);
    EXPECT_EQ(deserialized.shape.symmetric_exit, original.shape.symmetric_exit);
    EXPECT_NEAR(deserialized.shape.entry_tangent_scale, original.shape.entry_tangent_scale, 1e-6f);
    EXPECT_NEAR(deserialized.shape.width_multiplier, original.shape.width_multiplier, 1e-6f);
    EXPECT_NEAR(deserialized.shape.height_multiplier, original.shape.height_multiplier, 1e-6f);
}

// ---------------------------------------------------------------------------
// Full SurfaceGraph round-trip preserving frame data
// ---------------------------------------------------------------------------

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

TEST(SerializationTest, SurfaceGraph_FullRoundTrip_PreservesFrameData) {
    YarnProperties yarn = default_yarn();
    Gauge gauge = default_gauge();

    // Build a small stockinette: cast on 3, knit 3, bind off 3
    PatternInstructions pattern = create_pattern({"CCC", "KKK", "BBB"});
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);
    SurfaceGraph original = build_test_surface(yarn_path, yarn, gauge);

    // Serialize
    json j = surface_graph_to_json(original);

    // Deserialize
    SurfaceGraph deserialized = surface_graph_from_json(j);

    // Verify same number of nodes
    ASSERT_EQ(deserialized.nodes().size(), original.nodes().size());

    // Verify stitch_axis and shape survive the round trip for every node
    for (size_t i = 0; i < original.nodes().size(); ++i) {
        const auto& orig = original.nodes()[i];
        const auto& deser = deserialized.nodes()[i];

        // stitch_axis
        EXPECT_NEAR(deser.stitch_axis.x, orig.stitch_axis.x, 1e-5f)
            << "stitch_axis.x mismatch at node " << i;
        EXPECT_NEAR(deser.stitch_axis.y, orig.stitch_axis.y, 1e-5f)
            << "stitch_axis.y mismatch at node " << i;
        EXPECT_NEAR(deser.stitch_axis.z, orig.stitch_axis.z, 1e-5f)
            << "stitch_axis.z mismatch at node " << i;

        // shape fields
        EXPECT_NEAR(deser.shape.z_bulge, orig.shape.z_bulge, 1e-5f)
            << "shape.z_bulge mismatch at node " << i;
        EXPECT_NEAR(deser.shape.loop_height, orig.shape.loop_height, 1e-5f)
            << "shape.loop_height mismatch at node " << i;
        EXPECT_NEAR(deser.shape.loop_width, orig.shape.loop_width, 1e-5f)
            << "shape.loop_width mismatch at node " << i;
        EXPECT_NEAR(deser.shape.z_bulge_factor, orig.shape.z_bulge_factor, 1e-5f)
            << "shape.z_bulge_factor mismatch at node " << i;
        EXPECT_NEAR(deser.shape.apex_lean_x, orig.shape.apex_lean_x, 1e-5f)
            << "shape.apex_lean_x mismatch at node " << i;
        EXPECT_NEAR(deser.shape.apex_height_factor, orig.shape.apex_height_factor, 1e-5f)
            << "shape.apex_height_factor mismatch at node " << i;
        EXPECT_EQ(deser.shape.symmetric_exit, orig.shape.symmetric_exit)
            << "shape.symmetric_exit mismatch at node " << i;
        EXPECT_NEAR(deser.shape.entry_tangent_scale, orig.shape.entry_tangent_scale, 1e-5f)
            << "shape.entry_tangent_scale mismatch at node " << i;
        EXPECT_NEAR(deser.shape.width_multiplier, orig.shape.width_multiplier, 1e-5f)
            << "shape.width_multiplier mismatch at node " << i;
        EXPECT_NEAR(deser.shape.height_multiplier, orig.shape.height_multiplier, 1e-5f)
            << "shape.height_multiplier mismatch at node " << i;
    }
}
