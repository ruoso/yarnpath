#ifndef YARNPATH_MATH_VEC3_HPP
#define YARNPATH_MATH_VEC3_HPP

#include <cmath>
#include <array>

namespace yarnpath {

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    // Arithmetic operators
    constexpr Vec3 operator+(const Vec3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    constexpr Vec3 operator-(const Vec3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }

    constexpr Vec3 operator*(float scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    constexpr Vec3 operator/(float scalar) const {
        return {x / scalar, y / scalar, z / scalar};
    }

    constexpr Vec3 operator-() const {
        return {-x, -y, -z};
    }

    // Compound assignment
    constexpr Vec3& operator+=(const Vec3& other) {
        x += other.x; y += other.y; z += other.z;
        return *this;
    }

    constexpr Vec3& operator-=(const Vec3& other) {
        x -= other.x; y -= other.y; z -= other.z;
        return *this;
    }

    constexpr Vec3& operator*=(float scalar) {
        x *= scalar; y *= scalar; z *= scalar;
        return *this;
    }

    constexpr Vec3& operator/=(float scalar) {
        x /= scalar; y /= scalar; z /= scalar;
        return *this;
    }

    // Dot product
    constexpr float dot(const Vec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    // Cross product
    constexpr Vec3 cross(const Vec3& other) const {
        return {
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        };
    }

    // Magnitude squared (no sqrt)
    constexpr float length_squared() const {
        return x * x + y * y + z * z;
    }

    // Magnitude
    float length() const {
        return std::sqrt(length_squared());
    }

    // Normalized vector
    Vec3 normalized() const {
        float len = length();
        if (len > 0.0f) {
            return *this / len;
        }
        return {0.0f, 0.0f, 0.0f};
    }

    // Distance to another point
    float distance_to(const Vec3& other) const {
        return (*this - other).length();
    }

    // Comparison (exact)
    constexpr bool operator==(const Vec3& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    constexpr bool operator!=(const Vec3& other) const {
        return !(*this == other);
    }

    // Array access
    constexpr float& operator[](size_t i) {
        if (i == 0) return x;
        if (i == 1) return y;
        return z;
    }

    constexpr float operator[](size_t i) const {
        if (i == 0) return x;
        if (i == 1) return y;
        return z;
    }
};

// Scalar * Vec3
constexpr Vec3 operator*(float scalar, const Vec3& v) {
    return v * scalar;
}

// Linear interpolation
constexpr Vec3 lerp(const Vec3& a, const Vec3& b, float t) {
    return a * (1.0f - t) + b * t;
}

// Common constants
namespace vec3 {
    constexpr Vec3 zero() { return {0.0f, 0.0f, 0.0f}; }
    constexpr Vec3 unit_x() { return {1.0f, 0.0f, 0.0f}; }
    constexpr Vec3 unit_y() { return {0.0f, 1.0f, 0.0f}; }
    constexpr Vec3 unit_z() { return {0.0f, 0.0f, 1.0f}; }
}

}  // namespace yarnpath

#endif // YARNPATH_MATH_VEC3_HPP
