#include <gtest/gtest.h>
#include "geometry.hpp"
#include "yarn_path.hpp"
#include "physical_loop.hpp"
#include "test_helpers.hpp"
#include <cmath>
#include <limits>
#include <iostream>

using namespace yarnpath;
using namespace yarnpath::test;

// ============================================
// Loop Shape Detection Tests
// These tests verify that when we create a "loop" in the yarn path,
// the spline actually forms a visible loop shape (not just a smooth curve).
// ============================================

// Helper: Detect loop shapes in a polyline
// A "loop" is a section where the Y coordinate rises to a peak and falls back
struct DetectedLoop {
    size_t start_idx;
    size_t peak_idx;
    size_t end_idx;
    float peak_y;
    float base_y;
    Vec3 peak_pos;
};

static std::vector<DetectedLoop> detect_loops_in_polyline(
    const std::vector<Vec3>& polyline,
    float min_height = 0.01f) {

    std::vector<DetectedLoop> loops;

    if (polyline.size() < 3) return loops;

    enum State { LOOKING, RISING, FALLING };
    State state = LOOKING;

    size_t start_idx = 0;
    size_t peak_idx = 0;
    float peak_y = 0.0f;
    float base_y = 0.0f;

    for (size_t i = 1; i < polyline.size(); ++i) {
        float prev_y = polyline[i - 1].y;
        float curr_y = polyline[i].y;
        float dy = curr_y - prev_y;

        switch (state) {
            case LOOKING:
                if (dy > 0.001f) {
                    state = RISING;
                    start_idx = i - 1;
                    base_y = prev_y;
                    peak_y = curr_y;
                    peak_idx = i;
                }
                break;

            case RISING:
                if (curr_y > peak_y) {
                    peak_y = curr_y;
                    peak_idx = i;
                } else if (dy < -0.001f) {
                    state = FALLING;
                }
                break;

            case FALLING:
                if (dy > 0.001f) {
                    float height = peak_y - base_y;
                    if (height >= min_height) {
                        DetectedLoop loop;
                        loop.start_idx = start_idx;
                        loop.peak_idx = peak_idx;
                        loop.end_idx = i - 1;
                        loop.peak_y = peak_y;
                        loop.base_y = base_y;
                        loop.peak_pos = polyline[peak_idx];
                        loops.push_back(loop);
                    }
                    state = RISING;
                    start_idx = i - 1;
                    base_y = prev_y;
                    peak_y = curr_y;
                    peak_idx = i;
                } else if (curr_y < base_y - 0.01f) {
                    float height = peak_y - base_y;
                    if (height >= min_height) {
                        DetectedLoop loop;
                        loop.start_idx = start_idx;
                        loop.peak_idx = peak_idx;
                        loop.end_idx = i;
                        loop.peak_y = peak_y;
                        loop.base_y = base_y;
                        loop.peak_pos = polyline[peak_idx];
                        loops.push_back(loop);
                    }
                    state = LOOKING;
                }
                break;
        }
    }

    if (state == FALLING) {
        float height = peak_y - base_y;
        if (height >= min_height) {
            DetectedLoop loop;
            loop.start_idx = start_idx;
            loop.peak_idx = peak_idx;
            loop.end_idx = polyline.size() - 1;
            loop.peak_y = peak_y;
            loop.base_y = base_y;
            loop.peak_pos = polyline[peak_idx];
            loops.push_back(loop);
        }
    }

    return loops;
}

// ============================================
// Physical Loop Geometry Tests
// ============================================

// Helper: Calculate expected loop dimensions from physical properties
struct ExpectedLoopDimensions {
    float opening_diameter;
    float loop_height;
    float loop_width;
    float yarn_length;

    static ExpectedLoopDimensions from_properties(const Gauge& gauge, const YarnProperties& yarn) {
        ExpectedLoopDimensions dim;

        dim.opening_diameter = gauge.needle_diameter - 2.0f * yarn.radius;
        if (dim.opening_diameter < yarn.radius) {
            dim.opening_diameter = yarn.radius;
        }

        dim.opening_diameter *= yarn.loop_size_factor();

        float wrap_circumference = 3.14159f * (gauge.needle_diameter + 2.0f * yarn.radius);
        dim.loop_height = wrap_circumference * 0.5f * yarn.loop_aspect_ratio;
        dim.loop_width = dim.opening_diameter + 2.0f * yarn.radius;
        dim.yarn_length = wrap_circumference * (1.0f + yarn.loop_slack);

        return dim;
    }
};

TEST(PhysicalLoopTest, LoopOpeningDeterminedByNeedle) {
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    auto expected = ExpectedLoopDimensions::from_properties(gauge, yarn);

    EXPECT_GT(expected.opening_diameter, 0.0f);
    EXPECT_GT(expected.opening_diameter, yarn.radius);
    EXPECT_LT(expected.opening_diameter, gauge.needle_diameter);
}

TEST(PhysicalLoopTest, TighterTensionMakesSmallerLoops) {
    Gauge gauge = Gauge::worsted();

    YarnProperties loose_yarn = YarnProperties::worsted();
    loose_yarn.tension = 0.2f;

    YarnProperties tight_yarn = YarnProperties::worsted();
    tight_yarn.tension = 0.8f;

    auto loose_dim = ExpectedLoopDimensions::from_properties(gauge, loose_yarn);
    auto tight_dim = ExpectedLoopDimensions::from_properties(gauge, tight_yarn);

    EXPECT_LT(tight_dim.opening_diameter, loose_dim.opening_diameter);
}

TEST(PhysicalLoopTest, BiggerNeedleMakesBiggerLoops) {
    YarnProperties yarn = YarnProperties::worsted();

    Gauge small_needle = Gauge::worsted();
    small_needle.needle_diameter = 3.5f;

    Gauge large_needle = Gauge::worsted();
    large_needle.needle_diameter = 6.0f;

    auto small_dim = ExpectedLoopDimensions::from_properties(small_needle, yarn);
    auto large_dim = ExpectedLoopDimensions::from_properties(large_needle, yarn);

    EXPECT_GT(large_dim.opening_diameter, small_dim.opening_diameter);
}

TEST(PhysicalLoopTest, LoopMustFitYarnThrough) {
    std::vector<std::pair<Gauge, YarnProperties>> combinations = {
        {Gauge::fingering(), YarnProperties::fingering()},
        {Gauge::worsted(), YarnProperties::worsted()},
        {Gauge::bulky(), YarnProperties::bulky()},
    };

    for (const auto& [gauge, yarn] : combinations) {
        auto dim = ExpectedLoopDimensions::from_properties(gauge, yarn);
        EXPECT_GE(dim.opening_diameter, yarn.radius);
    }
}

// ============================================
// Topology-Driven Geometry Tests
// ============================================

TEST(TopologyGeometryTest, KnitLoopPassesThroughParent) {
    PatternInstructions pattern = create_pattern({
        "C",
        "K"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    const Loop* knit_loop = nullptr;
    const Loop* cast_on_loop = nullptr;
    for (const auto& loop : yarn_path.loops()) {
        if (loop.kind == FormKind::CastOn) cast_on_loop = &loop;
        if (loop.kind == FormKind::Knit) knit_loop = &loop;
    }

    ASSERT_NE(cast_on_loop, nullptr);
    ASSERT_NE(knit_loop, nullptr);
    ASSERT_EQ(knit_loop->parent_loops.size(), 1u);
    EXPECT_EQ(knit_loop->parent_loops[0], cast_on_loop->id);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    auto polyline = geometry.to_polyline_fixed(50);
    auto dim = ExpectedLoopDimensions::from_properties(gauge, yarn);

    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    for (const auto& pt : polyline) {
        min_y = std::min(min_y, pt.y);
        max_y = std::max(max_y, pt.y);
    }

    float total_height = max_y - min_y;

    EXPECT_GT(total_height, dim.loop_height * 0.5f);
}

TEST(TopologyGeometryTest, YarnFormsTwoDistinctLoops) {
    PatternInstructions pattern = create_pattern({
        "C",
        "K"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    auto polyline = geometry.to_polyline_fixed(100);
    auto dim = ExpectedLoopDimensions::from_properties(gauge, yarn);

    float detection_threshold = dim.loop_height * 0.2f;
    auto detected = detect_loops_in_polyline(polyline, detection_threshold);

    EXPECT_GE(detected.size(), 2u);
}

TEST(TopologyGeometryTest, LoopHeightMatchesPhysicalExpectation) {
    PatternInstructions pattern = create_pattern({
        "CCC",
        "KKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    auto polyline = geometry.to_polyline_fixed(50);
    auto dim = ExpectedLoopDimensions::from_properties(gauge, yarn);

    auto detected = detect_loops_in_polyline(polyline, dim.loop_height * 0.1f);

    int loops_with_correct_height = 0;
    for (const auto& loop : detected) {
        float height = loop.peak_y - loop.base_y;
        if (height >= dim.loop_height * 0.3f && height <= dim.loop_height * 2.0f) {
            loops_with_correct_height++;
        }
    }

    EXPECT_GE(loops_with_correct_height, 3);
}

TEST(TopologyGeometryTest, InterlockingStructureIsVisible) {
    PatternInstructions pattern = create_pattern({
        "CC",
        "KK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    auto polyline = geometry.to_polyline_fixed(100);
    auto dim = ExpectedLoopDimensions::from_properties(gauge, yarn);

    auto detected = detect_loops_in_polyline(polyline, dim.loop_height * 0.1f);

    std::vector<DetectedLoop> lower_loops, upper_loops;
    float y_midpoint = 0.0f;
    for (const auto& loop : detected) {
        y_midpoint += loop.peak_y;
    }
    if (!detected.empty()) {
        y_midpoint /= detected.size();
    }

    for (const auto& loop : detected) {
        if (loop.peak_y < y_midpoint) {
            lower_loops.push_back(loop);
        } else {
            upper_loops.push_back(loop);
        }
    }

    EXPECT_GE(lower_loops.size(), 2u);
    EXPECT_GE(upper_loops.size(), 2u);

    if (!lower_loops.empty() && !upper_loops.empty()) {
        float lower_avg_y = 0.0f;
        for (const auto& l : lower_loops) lower_avg_y += l.peak_y;
        lower_avg_y /= lower_loops.size();

        float upper_avg_y = 0.0f;
        for (const auto& l : upper_loops) upper_avg_y += l.peak_y;
        upper_avg_y /= upper_loops.size();

        float level_gap = upper_avg_y - lower_avg_y;

        EXPECT_GT(level_gap, dim.loop_height * 0.3f);
    }
}
