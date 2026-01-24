#ifndef YARNPATH_GEOMETRY_FABRIC_SURFACE_HPP
#define YARNPATH_GEOMETRY_FABRIC_SURFACE_HPP

#include "vec3.hpp"
#include <cmath>
#include <numbers>

namespace yarnpath {

// Abstract embedding surface for fabric coordinates → 3D
class FabricSurface {
public:
    virtual ~FabricSurface() = default;

    // Map fabric coordinates (u, v) to 3D position
    virtual Vec3 position(float u, float v) const = 0;

    // Surface normal at (u, v)
    virtual Vec3 normal(float u, float v) const = 0;

    // Tangent in u direction at (u, v)
    virtual Vec3 tangent_u(float u, float v) const = 0;

    // Tangent in v direction at (u, v)
    virtual Vec3 tangent_v(float u, float v) const = 0;

    // Transform a local offset (du, dv, dz) to world coordinates at (u, v)
    // where dz is perpendicular to the surface
    Vec3 local_to_world(float u, float v, float du, float dv, float dz) const {
        return position(u, v) +
               tangent_u(u, v) * du +
               tangent_v(u, v) * dv +
               normal(u, v) * dz;
    }
};

// Flat knitting surface (XY plane)
class PlaneSurface : public FabricSurface {
public:
    PlaneSurface() = default;
    PlaneSurface(const Vec3& origin, const Vec3& u_axis, const Vec3& v_axis)
        : origin_(origin), u_axis_(u_axis.normalized()), v_axis_(v_axis.normalized()) {
        normal_ = u_axis_.cross(v_axis_).normalized();
    }

    Vec3 position(float u, float v) const override {
        return origin_ + u_axis_ * u + v_axis_ * v;
    }

    Vec3 normal(float u, float v) const override {
        (void)u; (void)v;
        return normal_;
    }

    Vec3 tangent_u(float u, float v) const override {
        (void)u; (void)v;
        return u_axis_;
    }

    Vec3 tangent_v(float u, float v) const override {
        (void)u; (void)v;
        return v_axis_;
    }

private:
    Vec3 origin_ = vec3::zero();
    Vec3 u_axis_ = vec3::unit_x();
    Vec3 v_axis_ = vec3::unit_y();
    Vec3 normal_ = vec3::unit_z();
};

// Knitting in the round (cylinder with axis along Y)
class CylinderSurface : public FabricSurface {
public:
    // circumference: total u-coordinate distance for one wrap around
    // height_per_round: v-coordinate increase per circumference (for spiral tubes)
    CylinderSurface(float radius, float circumference, float height_per_round = 0.0f)
        : radius_(radius)
        , circumference_(circumference)
        , height_per_round_(height_per_round)
        , angular_scale_(2.0f * std::numbers::pi_v<float> / circumference) {}

    Vec3 position(float u, float v) const override {
        float theta = u * angular_scale_;
        float height = v + (height_per_round_ / circumference_) * u;
        return Vec3(
            radius_ * std::cos(theta),
            height,
            radius_ * std::sin(theta)
        );
    }

    Vec3 normal(float u, float v) const override {
        (void)v;
        float theta = u * angular_scale_;
        return Vec3(std::cos(theta), 0.0f, std::sin(theta));
    }

    Vec3 tangent_u(float u, float v) const override {
        (void)v;
        float theta = u * angular_scale_;
        // Derivative with respect to u: -sin(theta) * angular_scale, etc.
        // Normalized to unit length
        return Vec3(-std::sin(theta), 0.0f, std::cos(theta));
    }

    Vec3 tangent_v(float u, float v) const override {
        (void)u; (void)v;
        return vec3::unit_y();
    }

    float radius() const { return radius_; }
    float circumference() const { return circumference_; }

private:
    float radius_;
    float circumference_;
    float height_per_round_;
    float angular_scale_;
};

// Cone surface (for shaped knitting)
class ConeSurface : public FabricSurface {
public:
    // base_radius: radius at v=0
    // taper_rate: change in radius per unit v (negative for narrowing cone)
    // circumference: u-coordinate distance for one wrap at base
    ConeSurface(float base_radius, float taper_rate, float circumference)
        : base_radius_(base_radius)
        , taper_rate_(taper_rate)
        , circumference_(circumference)
        , angular_scale_(2.0f * std::numbers::pi_v<float> / circumference) {}

    Vec3 position(float u, float v) const override {
        float r = std::max(0.0f, base_radius_ + taper_rate_ * v);
        float theta = u * angular_scale_;
        return Vec3(
            r * std::cos(theta),
            v,
            r * std::sin(theta)
        );
    }

    Vec3 normal(float u, float v) const override {
        (void)v;  // Cone normal is independent of v
        float theta = u * angular_scale_;
        // For a cone, normal has both radial and vertical components
        // depending on the taper angle
        float cos_t = std::cos(theta);
        float sin_t = std::sin(theta);

        // Slope angle from taper rate
        float slope = std::atan(taper_rate_);
        float cos_slope = std::cos(slope);
        float sin_slope = std::sin(slope);

        return Vec3(
            cos_t * cos_slope,
            -sin_slope,
            sin_t * cos_slope
        ).normalized();
    }

    Vec3 tangent_u(float u, float v) const override {
        (void)v;
        float theta = u * angular_scale_;
        return Vec3(-std::sin(theta), 0.0f, std::cos(theta));
    }

    Vec3 tangent_v(float u, float v) const override {
        (void)u; (void)v;
        // Tangent along the cone slope
        float slope = std::atan(taper_rate_);
        return Vec3(0.0f, std::cos(slope), 0.0f).normalized();
    }

private:
    float base_radius_;
    float taper_rate_;
    float circumference_;
    float angular_scale_;
};

}  // namespace yarnpath

#endif // YARNPATH_GEOMETRY_FABRIC_SURFACE_HPP
