#ifndef YARNPATH_SERIALIZATION_CONFIG_JSON_HPP
#define YARNPATH_SERIALIZATION_CONFIG_JSON_HPP

#include <nlohmann/json.hpp>
#include <math/vec3.hpp>
#include <yarn/yarn_properties.hpp>
#include <yarn/gauge.hpp>
#include <surface/surface_builder.hpp>
#include <surface/surface_solver.hpp>
#include <surface/surface_forces.hpp>

namespace yarnpath {

// Vec3 serialization
inline void to_json(nlohmann::json& j, const Vec3& v) {
    j = nlohmann::json::array({v.x, v.y, v.z});
}

inline void from_json(const nlohmann::json& j, Vec3& v) {
    v.x = j[0].get<float>();
    v.y = j[1].get<float>();
    v.z = j[2].get<float>();
}

// YarnProperties serialization
inline void to_json(nlohmann::json& j, const YarnProperties& yarn) {
    j = {
        {"relaxed_radius", yarn.relaxed_radius},
        {"compressed_radius", yarn.compressed_radius},
        {"min_bend_radius", yarn.min_bend_radius},
        {"loop_slack", yarn.loop_slack},
        {"stiffness", yarn.stiffness},
        {"friction", yarn.friction},
        {"tension", yarn.tension},
        {"elasticity", yarn.elasticity},
        {"linear_density", yarn.linear_density}
    };
}

inline void from_json(const nlohmann::json& j, YarnProperties& yarn) {
    yarn.relaxed_radius = j.value("relaxed_radius", 1.0f);
    yarn.compressed_radius = j.value("compressed_radius", 0.1f);
    yarn.min_bend_radius = j.value("min_bend_radius", 0.3f);
    yarn.loop_slack = j.value("loop_slack", 0.1f);
    yarn.stiffness = j.value("stiffness", 0.5f);
    yarn.friction = j.value("friction", 0.3f);
    yarn.tension = j.value("tension", 0.5f);
    yarn.elasticity = j.value("elasticity", 0.3f);
    yarn.linear_density = j.value("linear_density", 0.003f);
}

// Gauge serialization
inline void to_json(nlohmann::json& j, const Gauge& gauge) {
    j = {
        {"needle_diameter", gauge.needle_diameter}
    };
}

inline void from_json(const nlohmann::json& j, Gauge& gauge) {
    gauge.needle_diameter = j.value("needle_diameter", 4.0f);
}

// SurfaceBuildConfig serialization
inline void to_json(nlohmann::json& j, const SurfaceBuildConfig& config) {
    j = {
        {"random_seed", config.random_seed},
        {"position_noise", config.position_noise},
        {"continuity_rest_length_factor", config.continuity_rest_length_factor},
        {"passthrough_rest_length_factor", config.passthrough_rest_length_factor},
        {"continuity_stiffness_factor", config.continuity_stiffness_factor},
        {"passthrough_stiffness_factor", config.passthrough_stiffness_factor}
    };
}

inline void from_json(const nlohmann::json& j, SurfaceBuildConfig& config) {
    config.random_seed = j.value("random_seed", 42u);
    config.position_noise = j.value("position_noise", 0.1f);
    config.continuity_rest_length_factor = j.value("continuity_rest_length_factor", 1.0f);
    config.passthrough_rest_length_factor = j.value("passthrough_rest_length_factor", 1.0f);
    config.continuity_stiffness_factor = j.value("continuity_stiffness_factor", 1.0f);
    config.passthrough_stiffness_factor = j.value("passthrough_stiffness_factor", 0.5f);
}

// ForceConfig serialization
inline void to_json(nlohmann::json& j, const ForceConfig& config) {
    j = {
        {"damping", config.damping},
        {"passthrough_tension_factor", config.passthrough_tension_factor},
        {"loop_curvature_strength", config.loop_curvature_strength},
        {"gravity_strength", config.gravity_strength},
        {"gravity_direction", config.gravity_direction},
        {"enable_gravity", config.enable_gravity},
        {"floor_position", config.floor_position},
        {"enable_floor", config.enable_floor},
        {"enable_collision", config.enable_collision},
        {"collision_strength", config.collision_strength}
    };
}

inline void from_json(const nlohmann::json& j, ForceConfig& config) {
    config.damping = j.value("damping", 0.5f);
    config.passthrough_tension_factor = j.value("passthrough_tension_factor", 1.0f);
    config.loop_curvature_strength = j.value("loop_curvature_strength", 0.1f);
    config.gravity_strength = j.value("gravity_strength", 9.8f);
    if (j.contains("gravity_direction")) {
        config.gravity_direction = j["gravity_direction"].get<Vec3>();
    }
    config.enable_gravity = j.value("enable_gravity", true);
    config.floor_position = j.value("floor_position", 0.0f);
    config.enable_floor = j.value("enable_floor", false);
    config.enable_collision = j.value("enable_collision", true);
    config.collision_strength = j.value("collision_strength", 100.0f);
}

// SolveConfig serialization (without frame_callback)
inline void to_json(nlohmann::json& j, const SolveConfig& config) {
    j = {
        {"dt", config.dt},
        {"max_iterations", config.max_iterations},
        {"convergence_threshold", config.convergence_threshold},
        {"constraint_iterations", config.constraint_iterations},
        {"pre_solve_iterations", config.pre_solve_iterations},
        {"force_config", config.force_config},
        {"num_threads", config.num_threads}
    };
}

inline void from_json(const nlohmann::json& j, SolveConfig& config) {
    config.dt = j.value("dt", 0.01f);
    config.max_iterations = j.value("max_iterations", 1000);
    config.convergence_threshold = j.value("convergence_threshold", 1e-4f);
    config.constraint_iterations = j.value("constraint_iterations", 30);
    config.pre_solve_iterations = j.value("pre_solve_iterations", 1000);
    if (j.contains("force_config")) {
        config.force_config = j["force_config"].get<ForceConfig>();
    }
    config.num_threads = j.value("num_threads", 0);
    // Note: frame_callback cannot be serialized
}

// SolveResult serialization
inline void to_json(nlohmann::json& j, const SolveResult& result) {
    j = {
        {"converged", result.converged},
        {"iterations", result.iterations},
        {"final_energy", result.final_energy},
        {"initial_energy", result.initial_energy}
    };
}

inline void from_json(const nlohmann::json& j, SolveResult& result) {
    result.converged = j.value("converged", false);
    result.iterations = j.value("iterations", 0);
    result.final_energy = j.value("final_energy", 0.0f);
    result.initial_energy = j.value("initial_energy", 0.0f);
}

}  // namespace yarnpath

#endif // YARNPATH_SERIALIZATION_CONFIG_JSON_HPP
