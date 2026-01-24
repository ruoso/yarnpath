#include <gtest/gtest.h>
#include "geometry.hpp"
#include <numbers>

using namespace yarnpath;

// ============================================
// FabricSurface Tests
// ============================================

TEST(PlaneSurfaceTest, Default) {
    PlaneSurface surface;

    Vec3 pos = surface.position(1.0f, 2.0f);
    EXPECT_FLOAT_EQ(pos.x, 1.0f);
    EXPECT_FLOAT_EQ(pos.y, 2.0f);
    EXPECT_FLOAT_EQ(pos.z, 0.0f);
}

TEST(PlaneSurfaceTest, Normal) {
    PlaneSurface surface;

    Vec3 n = surface.normal(0.0f, 0.0f);
    EXPECT_FLOAT_EQ(n.z, 1.0f);
}

TEST(PlaneSurfaceTest, LocalToWorld) {
    PlaneSurface surface;

    Vec3 world = surface.local_to_world(1.0f, 2.0f, 0.1f, 0.2f, 0.3f);
    EXPECT_FLOAT_EQ(world.x, 1.1f);
    EXPECT_FLOAT_EQ(world.y, 2.2f);
    EXPECT_FLOAT_EQ(world.z, 0.3f);
}

TEST(CylinderSurfaceTest, Position) {
    CylinderSurface surface(1.0f, 2.0f * std::numbers::pi_v<float>);

    // At u=0, should be at (1, 0, 0)
    Vec3 pos0 = surface.position(0.0f, 0.0f);
    EXPECT_NEAR(pos0.x, 1.0f, 0.001f);
    EXPECT_NEAR(pos0.y, 0.0f, 0.001f);
    EXPECT_NEAR(pos0.z, 0.0f, 0.001f);

    // At u=pi (half circumference), should be at (-1, 0, 0)
    Vec3 pos_half = surface.position(std::numbers::pi_v<float>, 0.0f);
    EXPECT_NEAR(pos_half.x, -1.0f, 0.001f);
    EXPECT_NEAR(pos_half.z, 0.0f, 0.001f);
}
