#include <gtest/gtest.h>
#include "geometry.hpp"
#include "yarn_path.hpp"
#include "test_helpers.hpp"

using namespace yarnpath;
using namespace yarnpath::test;

// Topology-geometry integration tests verify that the topology
// (parent-child relationships) maps correctly to geometry.

TEST(TopologyGeometryTest, ParentChildInTopology) {
    PatternInstructions pattern = create_pattern({
        "CC",
        "KK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    // Count loop segments with parents
    size_t with_parents = 0;
    size_t without_parents = 0;

    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop) {
            if (seg.through.empty()) {
                without_parents++;
            } else {
                with_parents++;
            }
        }
    }

    EXPECT_EQ(without_parents, 2u);  // 2 cast-on
    EXPECT_EQ(with_parents, 2u);     // 2 knit
}

TEST(TopologyGeometryTest, GeometryFromTopology) {
    PatternInstructions pattern = create_pattern({
        "CCC",
        "KKK",
        "PPP"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted()
    );

    // Geometry should have same number of segments as topology
    EXPECT_EQ(geometry.segments().size(), yarn_path.segments().size());
}

TEST(TopologyGeometryTest, VerticalStacking) {
    PatternInstructions pattern = create_pattern({
        "CCC",
        "KKK",
        "KKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted()
    );

    // Get bounding box - should have vertical extent
    auto [min_pt, max_pt] = geometry.bounding_box();
    EXPECT_GT(max_pt.y - min_pt.y, 0.0f);  // Non-zero height
}
