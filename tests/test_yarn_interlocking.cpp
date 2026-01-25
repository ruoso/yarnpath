#include <gtest/gtest.h>
#include <algorithm>
#include "geometry.hpp"
#include "yarn_path.hpp"
#include "test_helpers.hpp"

using namespace yarnpath;
using namespace yarnpath::test;

// Yarn interlocking tests verify that the topology correctly
// represents how yarn passes through existing loops.

TEST(YarnInterlockingTest, KnitPassesThrough) {
    PatternInstructions pattern = create_pattern({
        "CC",
        "KK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    // Find knit loops - they should pass through cast-on loops
    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop && !seg.through.empty()) {
            // This is a worked loop - verify it passes through a valid parent
            for (SegmentId parent : seg.through) {
                EXPECT_LT(parent, yarn_path.segments().size());
                EXPECT_TRUE(yarn_path.segments()[parent].forms_loop);
            }
        }
    }
}

TEST(YarnInterlockingTest, ThroughVectorCorrect) {
    PatternInstructions pattern = create_pattern({
        "CCC",
        "KKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    // Each knit should have exactly one parent
    size_t knit_count = 0;
    for (const auto& seg : yarn_path.segments()) {
        if (seg.forms_loop && !seg.through.empty()) {
            knit_count++;
            EXPECT_EQ(seg.through.size(), 1u);
        }
    }
    EXPECT_EQ(knit_count, 3u);
}

TEST(YarnInterlockingTest, MultiRowInterlocking) {
    PatternInstructions pattern = create_pattern({
        "CC",
        "KK",
        "KK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    // Count loops at each level (by counting parents)
    size_t level0 = 0;  // No parents (cast-on)
    size_t level1 = 0;  // Parents are level 0
    size_t level2 = 0;  // Parents are level 1

    // First pass: identify cast-ons
    std::vector<SegmentId> level0_ids;
    std::vector<SegmentId> level1_ids;

    for (size_t i = 0; i < yarn_path.segments().size(); ++i) {
        const auto& seg = yarn_path.segments()[i];
        if (seg.forms_loop && seg.through.empty()) {
            level0++;
            level0_ids.push_back(static_cast<SegmentId>(i));
        }
    }

    // Second pass: identify loops whose parents are in level0
    for (size_t i = 0; i < yarn_path.segments().size(); ++i) {
        const auto& seg = yarn_path.segments()[i];
        if (seg.forms_loop && !seg.through.empty()) {
            bool parent_is_level0 = false;
            for (SegmentId p : seg.through) {
                if (std::find(level0_ids.begin(), level0_ids.end(), p) != level0_ids.end()) {
                    parent_is_level0 = true;
                    break;
                }
            }
            if (parent_is_level0) {
                level1++;
                level1_ids.push_back(static_cast<SegmentId>(i));
            }
        }
    }

    // Third pass: identify loops whose parents are in level1
    for (size_t i = 0; i < yarn_path.segments().size(); ++i) {
        const auto& seg = yarn_path.segments()[i];
        if (seg.forms_loop && !seg.through.empty()) {
            bool parent_is_level1 = false;
            for (SegmentId p : seg.through) {
                if (std::find(level1_ids.begin(), level1_ids.end(), p) != level1_ids.end()) {
                    parent_is_level1 = true;
                    break;
                }
            }
            if (parent_is_level1) {
                level2++;
            }
        }
    }

    EXPECT_EQ(level0, 2u);  // 2 cast-on
    EXPECT_EQ(level1, 2u);  // 2 knit in row 1
    EXPECT_EQ(level2, 2u);  // 2 knit in row 2
}
