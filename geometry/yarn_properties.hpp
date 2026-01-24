#ifndef YARNPATH_GEOMETRY_YARN_PROPERTIES_HPP
#define YARNPATH_GEOMETRY_YARN_PROPERTIES_HPP

namespace yarnpath {

// Physical yarn characteristics that affect geometry generation
struct YarnProperties {
    // Core dimension
    float radius = 1.0f;              // Yarn radius (half diameter)

    // Bending constraints
    float min_bend_radius = 3.0f;     // Minimum curvature radius (typically 2-4x radius)

    // Loop formation
    float loop_aspect_ratio = 1.5f;   // Height/width of relaxed loop
    float loop_slack = 0.1f;          // Extra length factor [0-1]

    // Interaction properties
    float stiffness = 0.5f;           // 0=flexible, 1=stiff (affects repulsion)
    float friction = 0.3f;            // Yarn-yarn contact coefficient (future use)

    // Derived properties
    float min_clearance() const {
        return 2.0f * radius;
    }

    float max_curvature() const {
        return 1.0f / min_bend_radius;
    }

    // Factory methods for common yarn types
    static YarnProperties fingering() {
        return YarnProperties{
            .radius = 0.5f,
            .min_bend_radius = 1.5f,
            .loop_aspect_ratio = 1.4f,
            .loop_slack = 0.08f,
            .stiffness = 0.4f,
            .friction = 0.3f
        };
    }

    static YarnProperties worsted() {
        return YarnProperties{
            .radius = 1.0f,
            .min_bend_radius = 3.0f,
            .loop_aspect_ratio = 1.5f,
            .loop_slack = 0.1f,
            .stiffness = 0.5f,
            .friction = 0.3f
        };
    }

    static YarnProperties bulky() {
        return YarnProperties{
            .radius = 2.0f,
            .min_bend_radius = 6.0f,
            .loop_aspect_ratio = 1.6f,
            .loop_slack = 0.12f,
            .stiffness = 0.6f,
            .friction = 0.35f
        };
    }
};

}  // namespace yarnpath

#endif // YARNPATH_GEOMETRY_YARN_PROPERTIES_HPP
