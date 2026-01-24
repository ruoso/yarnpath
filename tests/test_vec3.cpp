#include <gtest/gtest.h>
#include "geometry.hpp"

using namespace yarnpath;

TEST(Vec3Test, DefaultConstruction) {
    Vec3 v;
    EXPECT_FLOAT_EQ(v.x, 0.0f);
    EXPECT_FLOAT_EQ(v.y, 0.0f);
    EXPECT_FLOAT_EQ(v.z, 0.0f);
}

TEST(Vec3Test, ValueConstruction) {
    Vec3 v(1.0f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(v.x, 1.0f);
    EXPECT_FLOAT_EQ(v.y, 2.0f);
    EXPECT_FLOAT_EQ(v.z, 3.0f);
}

TEST(Vec3Test, Addition) {
    Vec3 a(1.0f, 2.0f, 3.0f);
    Vec3 b(4.0f, 5.0f, 6.0f);
    Vec3 c = a + b;
    EXPECT_FLOAT_EQ(c.x, 5.0f);
    EXPECT_FLOAT_EQ(c.y, 7.0f);
    EXPECT_FLOAT_EQ(c.z, 9.0f);
}

TEST(Vec3Test, DotProduct) {
    Vec3 a(1.0f, 0.0f, 0.0f);
    Vec3 b(0.0f, 1.0f, 0.0f);
    EXPECT_FLOAT_EQ(a.dot(b), 0.0f);

    Vec3 c(1.0f, 2.0f, 3.0f);
    Vec3 d(4.0f, 5.0f, 6.0f);
    EXPECT_FLOAT_EQ(c.dot(d), 32.0f);
}

TEST(Vec3Test, CrossProduct) {
    Vec3 x = vec3::unit_x();
    Vec3 y = vec3::unit_y();
    Vec3 z = x.cross(y);
    EXPECT_FLOAT_EQ(z.x, 0.0f);
    EXPECT_FLOAT_EQ(z.y, 0.0f);
    EXPECT_FLOAT_EQ(z.z, 1.0f);
}

TEST(Vec3Test, Length) {
    Vec3 v(3.0f, 4.0f, 0.0f);
    EXPECT_FLOAT_EQ(v.length(), 5.0f);
}

TEST(Vec3Test, Normalized) {
    Vec3 v(3.0f, 4.0f, 0.0f);
    Vec3 n = v.normalized();
    EXPECT_FLOAT_EQ(n.length(), 1.0f);
    EXPECT_FLOAT_EQ(n.x, 0.6f);
    EXPECT_FLOAT_EQ(n.y, 0.8f);
}

TEST(Vec3Test, Lerp) {
    Vec3 a(0.0f, 0.0f, 0.0f);
    Vec3 b(10.0f, 10.0f, 10.0f);
    Vec3 mid = lerp(a, b, 0.5f);
    EXPECT_FLOAT_EQ(mid.x, 5.0f);
    EXPECT_FLOAT_EQ(mid.y, 5.0f);
    EXPECT_FLOAT_EQ(mid.z, 5.0f);
}
