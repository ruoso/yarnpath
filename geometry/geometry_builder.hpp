#ifndef YARNPATH_GEOMETRY_BUILDER_HPP
#define YARNPATH_GEOMETRY_BUILDER_HPP

#include "geometry_path.hpp"
#include "yarn_path.hpp"
#include "yarn_properties.hpp"
#include "gauge.hpp"
#include "fabric_surface.hpp"
#include "physical_loop.hpp"
#include <map>
#include <set>
#include <queue>

namespace yarnpath {

// Builder class that assembles geometry from YarnPath using physical properties
class GeometryBuilder {
public:
    GeometryBuilder(
        const YarnPath& yarn_path,
        const YarnProperties& yarn,
        const Gauge& gauge,
        const FabricSurface& surface
    );

    // Build the complete geometry
    GeometryPath build();

private:
    const YarnPath& yarn_path_;
    YarnProperties yarn_;
    Gauge gauge_;
    const FabricSurface& surface_;

    // Working state
    std::map<LoopId, LoopPosition> loop_positions_;

    // Helper: Get Z offset for pass mode
    float get_z_offset(PassMode mode) const;

    // Helper: Transform position to 3D
    Vec3 to_world_position(float u, float v, float z) const;
};

}  // namespace yarnpath

#endif // YARNPATH_GEOMETRY_BUILDER_HPP
