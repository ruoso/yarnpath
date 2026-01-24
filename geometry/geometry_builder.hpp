#ifndef YARNPATH_GEOMETRY_BUILDER_HPP
#define YARNPATH_GEOMETRY_BUILDER_HPP

#include "geometry_path.hpp"
#include "stitch_glyph.hpp"
#include "yarn_path.hpp"
#include "yarn_properties.hpp"
#include "gauge.hpp"
#include "fabric_surface.hpp"
#include <map>
#include <set>
#include <queue>

namespace yarnpath {

// Builder class that assembles geometry from YarnPath
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
    GlyphFactory glyph_factory_;

    // Working state
    std::map<LoopId, LoopPosition> loop_positions_;
    std::map<AnchorId, AnchorGeometry> anchor_geometries_;
    std::map<LoopId, StitchGlyph> loop_glyphs_;

    // Phase 1: Initialize cast-on positions
    void initialize_cast_on_positions();

    // Phase 2: Propagate positions through topology (BFS from cast-on)
    void propagate_positions();

    // Phase 3: Generate glyph geometry for each loop
    void generate_glyph_geometries();

    // Phase 4: Create anchor geometries from glyphs
    void create_anchor_geometries();

    // Phase 5: Connect segments between anchors
    std::vector<SegmentGeometry> connect_segments();

    // Phase 6: Apply constraints (curvature, clearance)
    void apply_constraints(std::vector<SegmentGeometry>& segments);

    // Helper: Get loop parents
    std::vector<LoopId> get_loop_parents(LoopId loop_id) const;

    // Helper: Check if all parents have positions
    bool all_parents_positioned(LoopId loop_id) const;

    // Helper: Compute child position from parents
    LoopPosition compute_child_position(const Loop& loop) const;

    // Helper: Get Z offset for pass mode
    float get_z_offset(PassMode mode) const;

    // Helper: Transform position to 3D
    Vec3 to_world_position(float u, float v, float z) const;

    // Helper: Create connecting curve between anchors
    BezierSpline create_connecting_curve(
        const AnchorGeometry& from,
        const AnchorGeometry& to,
        const YarnSegment& segment) const;
};

}  // namespace yarnpath

#endif // YARNPATH_GEOMETRY_BUILDER_HPP
