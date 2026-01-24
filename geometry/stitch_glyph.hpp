#ifndef YARNPATH_GEOMETRY_STITCH_GLYPH_HPP
#define YARNPATH_GEOMETRY_STITCH_GLYPH_HPP

#include "vec3.hpp"
#include "cubic_bezier.hpp"
#include "yarn_properties.hpp"
#include "gauge.hpp"
#include "yarn_path.hpp"
#include <map>
#include <string>
#include <vector>

namespace yarnpath {

// Position and direction at a glyph anchor point
struct GlyphAnchorPosition {
    Vec3 position;    // Local coords normalized [0,1]^3
    Vec3 tangent;     // Yarn direction (normalized)
};

// A template for generating stitch geometry
struct StitchGlyph {
    FormKind kind;
    std::map<std::string, GlyphAnchorPosition> anchors;  // "entry", "apex", "exit", etc.
    std::vector<CubicBezier> path_segments;

    // Get anchor or return nullptr
    const GlyphAnchorPosition* get_anchor(const std::string& name) const;

    // Transform glyph to world coordinates
    // origin: bottom-left of stitch cell
    // scale: (stitch_width, row_height, fabric_thickness)
    StitchGlyph transformed(const Vec3& origin, const Vec3& scale) const;
};

// Factory for creating glyphs based on stitch type
class GlyphFactory {
public:
    GlyphFactory() = default;
    explicit GlyphFactory(const YarnProperties& yarn, const Gauge& gauge);

    // Set properties for glyph generation
    void set_yarn_properties(const YarnProperties& yarn) { yarn_ = yarn; }
    void set_gauge(const Gauge& gauge) { gauge_ = gauge; }

    // Create glyph for a loop with given formation
    StitchGlyph create_glyph(FormKind kind, const Loop& loop) const;

    // Individual glyph generators
    StitchGlyph create_cast_on_glyph() const;
    StitchGlyph create_knit_glyph(bool twisted = false) const;
    StitchGlyph create_purl_glyph(bool twisted = false) const;
    StitchGlyph create_slip_glyph() const;
    StitchGlyph create_bind_off_glyph() const;
    StitchGlyph create_yarn_over_glyph() const;
    StitchGlyph create_kfb_glyph(int loop_index) const;  // 0 = front, 1 = back
    StitchGlyph create_m1l_glyph() const;
    StitchGlyph create_m1r_glyph() const;
    StitchGlyph create_k2tog_glyph() const;
    StitchGlyph create_ssk_glyph() const;
    StitchGlyph create_s2kp_glyph() const;

private:
    YarnProperties yarn_;
    Gauge gauge_;

    // Helper to create basic loop shape
    std::vector<CubicBezier> create_basic_loop_path(
        const Vec3& entry, const Vec3& apex, const Vec3& exit,
        float loop_height, bool front_to_back) const;

    // Helper to create anchor positions for a standard loop
    std::map<std::string, GlyphAnchorPosition> create_standard_anchors(
        bool front_to_back, float apex_height) const;
};

}  // namespace yarnpath

#endif // YARNPATH_GEOMETRY_STITCH_GLYPH_HPP
