#ifndef YARNPATH_YARN_YARN_PROPERTIES_HPP
#define YARNPATH_YARN_YARN_PROPERTIES_HPP

namespace yarnpath {

// Physical yarn characteristics that affect geometry generation
struct YarnProperties {
    // Core dimension
    float relaxed_radius = 1.0f;
    float compressed_radius = 0.1f;              // Yarn compressed_radius (half diameter)

    // Bending constraints
    float min_bend_radius = 0.3f;     // Minimum curvature compressed_radius (typically 2-4x compressed_radius)

    // Loop formation
    float loop_slack = 0.1f;          // Extra length factor [0-1]

    // Interaction properties
    float stiffness = 0.5f;           // 0=flexible, 1=stiff (affects repulsion)
    float friction = 0.3f;            // Yarn-yarn contact coefficient (future use)

    // Tension/tightness properties
    float tension = 0.5f;             // 0=loose, 1=tight knitting style
    float elasticity = 0.3f;          // How much the yarn stretches under tension

    // Mass properties
    float linear_density = 0.003f;    // Mass per unit length (g/mm) - worsted ~3g per meter = 0.003g/mm

    // Derived properties
    float min_clearance() const {
        return 2.0f * compressed_radius;
    }

    float max_curvature() const {
        return 1.0f / min_bend_radius;
    }

    // Estimate yarn length for a connector segment (mm)
    // Just the distance between nodes plus some slack
    float connector_yarn_length(float distance) const {
        return distance * (1.0f + loop_slack * 0.5f);
    }

    // Mass of a loop segment (grams)
    float loop_mass(float loop_width) const {
        return loop_width * linear_density;
    }

    // Mass of a connector segment (grams)
    float connector_mass(float distance) const {
        return connector_yarn_length(distance) * linear_density;
    }

    // Knot tightness factor: how much the loop contracts from its relaxed state
    // Higher tension + lower elasticity = tighter knots
    float knot_tightness() const {
        return tension * (1.0f - elasticity * 0.5f);
    }

    // Effective loop size multiplier (1.0 = relaxed, < 1.0 = tighter)
    float loop_size_factor() const {
        return 1.0f - knot_tightness() * 0.3f;  // Up to 30% smaller when very tight
    }

    // Factory methods for common yarn types
    static YarnProperties fingering() {
        return YarnProperties{
            .compressed_radius = 0.5f,
            .min_bend_radius = 1.5f,
            .loop_slack = 0.08f,
            .stiffness = 0.4f,
            .friction = 0.3f
        };
    }

    static YarnProperties worsted() {
        return YarnProperties{
            .compressed_radius = 1.0f,
            .min_bend_radius = 3.0f,
            .loop_slack = 0.1f,
            .stiffness = 0.5f,
            .friction = 0.3f
        };
    }

    static YarnProperties bulky() {
        return YarnProperties{
            .compressed_radius = 2.0f,
            .min_bend_radius = 6.0f,
            .loop_slack = 0.12f,
            .stiffness = 0.6f,
            .friction = 0.35f
        };
    }

    static YarnProperties thin() {
        return YarnProperties{
            .compressed_radius = 0.01f,
            .loop_slack = 0.07f,
            .stiffness = 0.35f,
            .friction = 0.25f
        };
    }
};

}  // namespace yarnpath

#endif // YARNPATH_YARN_YARN_PROPERTIES_HPP
