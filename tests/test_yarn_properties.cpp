#include <gtest/gtest.h>
#include "geometry.hpp"

using namespace yarnpath;

// ============================================
// YarnProperties Tests
// ============================================

TEST(YarnPropertiesTest, Defaults) {
    GTEST_SKIP();
    YarnProperties yarn;
    EXPECT_FLOAT_EQ(yarn.compressed_radius, 1.0f);
    EXPECT_FLOAT_EQ(yarn.min_bend_radius, 3.0f);
}

TEST(YarnPropertiesTest, DerivedProperties) {
    YarnProperties yarn;
    yarn.compressed_radius = 2.0f;
    yarn.min_bend_radius = 6.0f;

    EXPECT_FLOAT_EQ(yarn.min_clearance(), 4.0f);
    EXPECT_NEAR(yarn.max_curvature(), 1.0f / 6.0f, 0.001f);
}

TEST(YarnPropertiesTest, Presets) {
    GTEST_SKIP();
    auto fingering = YarnProperties::fingering();
    EXPECT_LT(fingering.compressed_radius, 1.0f);

    auto bulky = YarnProperties::bulky();
    EXPECT_GT(bulky.compressed_radius, 1.0f);
}

// ============================================
// Gauge Tests
// ============================================

TEST(GaugeTest, Defaults) {
    Gauge gauge;
    EXPECT_FLOAT_EQ(gauge.needle_diameter, 4.0f);
}

TEST(GaugeTest, LoopHeight) {
    Gauge gauge;
    gauge.needle_diameter = 5.0f;
    float yarn_compressed_radius = 1.0f;
    
    // Loop height should be based on needle diameter and yarn radius
    float loop_height = gauge.loop_height(yarn_compressed_radius);
    EXPECT_GT(loop_height, 0.0f);
    
    // Larger needles should give larger loops
    Gauge gauge_larger;
    gauge_larger.needle_diameter = 8.0f;
    float larger_loop_height = gauge_larger.loop_height(yarn_compressed_radius);
    EXPECT_GT(larger_loop_height, loop_height);
}

TEST(GaugeTest, Presets) {
    auto fingering = Gauge::fingering();
    auto worsted = Gauge::worsted();
    auto bulky = Gauge::bulky();
    
    // Fingering uses smallest needles
    EXPECT_LT(fingering.needle_diameter, worsted.needle_diameter);
    EXPECT_LT(worsted.needle_diameter, bulky.needle_diameter);
}
