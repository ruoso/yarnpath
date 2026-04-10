#include <gtest/gtest.h>

#include "geometry.hpp"
#include "surface/surface.hpp"
#include "test_helpers.hpp"
#include "yarn_path.hpp"

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

using namespace yarnpath;
using namespace yarnpath::test;

namespace {

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

static GeometryPath build_geometry_for_pattern(const std::vector<std::string>& rows,
                                               const YarnProperties& yarn,
                                               const Gauge& gauge) {
    PatternInstructions pattern = create_pattern(rows);
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph, yarn, gauge);
    SurfaceGraph surface = build_test_surface(yarn_path, yarn, gauge);
    return GeometryPath::from_yarn_path(yarn_path, surface, yarn, gauge);
}

static void expect_no_duplicate_consecutive_polyline_points(const GeometryPath& geometry,
                                                            float eps = 1e-6f) {
    const std::vector<Vec3> polyline = geometry.to_polyline_fixed(12);
    ASSERT_GT(polyline.size(), 1u);

    for (size_t i = 1; i < polyline.size(); ++i) {
        const float d = (polyline[i] - polyline[i - 1]).length();
        EXPECT_GT(d, eps) << "duplicate consecutive polyline points at index " << i;
    }
}

static void expect_no_zero_length_segments(const GeometryPath& geometry,
                                           float eps = 1e-6f) {
    ASSERT_FALSE(geometry.segments().empty());

    for (const auto& seg : geometry.segments()) {
        EXPECT_FALSE(seg.curve.empty())
            << "segment " << seg.segment_id << " has empty curve";
        EXPECT_GT(seg.arc_length, eps)
            << "segment " << seg.segment_id << " has zero arc_length";
        const float chord = (seg.curve.end() - seg.curve.start()).length();
        EXPECT_GT(chord, eps)
            << "segment " << seg.segment_id << " has zero-length chord";
    }
}

static void expect_valid_obj_export(const GeometryPath& geometry) {
    const std::string obj = geometry.to_obj(12);
    ASSERT_FALSE(obj.empty());

    std::istringstream in(obj);
    std::string line;

    size_t vertex_count = 0;
    size_t line_count = 0;

    while (std::getline(in, line)) {
        if (line.rfind("v ", 0) == 0) {
            std::istringstream ls(line);
            char tag = '\0';
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            ls >> tag >> x >> y >> z;
            ASSERT_TRUE(static_cast<bool>(ls)) << "invalid OBJ vertex record: " << line;
            EXPECT_TRUE(std::isfinite(x)) << "OBJ vertex x is non-finite";
            EXPECT_TRUE(std::isfinite(y)) << "OBJ vertex y is non-finite";
            EXPECT_TRUE(std::isfinite(z)) << "OBJ vertex z is non-finite";
            ++vertex_count;
            continue;
        }

        if (line.rfind("l ", 0) == 0) {
            std::istringstream ls(line);
            char tag = '\0';
            ls >> tag;
            ASSERT_EQ(tag, 'l');

            std::vector<int> indices;
            int idx = 0;
            while (ls >> idx) {
                indices.push_back(idx);
            }

            ASSERT_GE(indices.size(), 2u) << "OBJ line record must reference >= 2 vertices";
            for (int v_idx : indices) {
                EXPECT_GE(v_idx, 1) << "OBJ line index must be 1-based";
                EXPECT_LE(static_cast<size_t>(v_idx), vertex_count)
                    << "OBJ line index out of vertex range";
            }
            ++line_count;
        }
    }

    EXPECT_GT(vertex_count, 1u) << "OBJ export must include at least two vertices";
    EXPECT_GT(line_count, 0u) << "OBJ export must include at least one line record";
}

static void expect_geometry_output_validity(const GeometryPath& geometry) {
    expect_no_duplicate_consecutive_polyline_points(geometry);
    expect_no_zero_length_segments(geometry);
    expect_valid_obj_export(geometry);
}

}  // namespace

TEST(GeometryRefOutputValidity, StockinettePanelShouldProduceStructurallyValidOutputGeometry) {
    const YarnProperties yarn = default_yarn();
    const Gauge gauge = default_gauge();

    const GeometryPath geometry = build_geometry_for_pattern({"CCCC", "KKKK", "PPPP", "KKKK"}, yarn, gauge);
    expect_geometry_output_validity(geometry);
}

TEST(GeometryRefOutputValidity, AlternatingKnitPurlRowShouldProduceStructurallyValidOutputGeometry) {
    const YarnProperties yarn = default_yarn();
    const Gauge gauge = default_gauge();

    const GeometryPath geometry = build_geometry_for_pattern({"CCCC", "KPKP"}, yarn, gauge);
    expect_geometry_output_validity(geometry);
}

TEST(GeometryRefOutputValidity, YarnOverRowShouldProduceStructurallyValidOutputGeometry) {
    const YarnProperties yarn = default_yarn();
    const Gauge gauge = default_gauge();

    const GeometryPath geometry = build_geometry_for_pattern({"CCC", "KOK"}, yarn, gauge);
    expect_geometry_output_validity(geometry);
}

TEST(GeometryRefOutputValidity, OpposingDecreasesRowShouldProduceStructurallyValidOutputGeometry) {
    const YarnProperties yarn = default_yarn();
    const Gauge gauge = default_gauge();

    const GeometryPath geometry = build_geometry_for_pattern({"CCCCCC", "K2SK"}, yarn, gauge);
    expect_geometry_output_validity(geometry);
}
