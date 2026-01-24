#include "stitch_glyph.hpp"
#include <cmath>

namespace yarnpath {

const GlyphAnchorPosition* StitchGlyph::get_anchor(const std::string& name) const {
    auto it = anchors.find(name);
    if (it != anchors.end()) {
        return &it->second;
    }
    return nullptr;
}

StitchGlyph StitchGlyph::transformed(const Vec3& origin, const Vec3& scale) const {
    StitchGlyph result;
    result.kind = kind;

    // Transform anchors
    for (const auto& [name, anchor] : anchors) {
        GlyphAnchorPosition transformed_anchor;
        transformed_anchor.position = Vec3(
            origin.x + anchor.position.x * scale.x,
            origin.y + anchor.position.y * scale.y,
            origin.z + anchor.position.z * scale.z
        );
        // Tangent needs to be scaled then renormalized
        transformed_anchor.tangent = Vec3(
            anchor.tangent.x * scale.x,
            anchor.tangent.y * scale.y,
            anchor.tangent.z * scale.z
        ).normalized();
        result.anchors[name] = transformed_anchor;
    }

    // Transform path segments
    for (const auto& seg : path_segments) {
        CubicBezier transformed_seg;
        for (int i = 0; i < 4; ++i) {
            transformed_seg.control_points[i] = Vec3(
                origin.x + seg.control_points[i].x * scale.x,
                origin.y + seg.control_points[i].y * scale.y,
                origin.z + seg.control_points[i].z * scale.z
            );
        }
        result.path_segments.push_back(transformed_seg);
    }

    return result;
}

GlyphFactory::GlyphFactory(const YarnProperties& yarn, const Gauge& gauge)
    : yarn_(yarn), gauge_(gauge) {}

StitchGlyph GlyphFactory::create_glyph(FormKind kind, const Loop& loop) const {
    switch (kind) {
        case FormKind::CastOn:
            return create_cast_on_glyph();
        case FormKind::Knit:
            return create_knit_glyph(loop.twisted);
        case FormKind::Purl:
            return create_purl_glyph(loop.twisted);
        case FormKind::Slip:
            return create_slip_glyph();
        case FormKind::BindOff:
            return create_bind_off_glyph();
        case FormKind::YarnOver:
            return create_yarn_over_glyph();
        case FormKind::KFB:
            // KFB creates 2 loops; use loop index to determine which one
            // This is a simplification - actual implementation may need more context
            return create_kfb_glyph(0);
        case FormKind::M1L:
            return create_m1l_glyph();
        case FormKind::M1R:
            return create_m1r_glyph();
        case FormKind::K2tog:
            return create_k2tog_glyph();
        case FormKind::SSK:
            return create_ssk_glyph();
        case FormKind::S2KP:
            return create_s2kp_glyph();
        default:
            return create_knit_glyph();
    }
}

std::map<std::string, GlyphAnchorPosition> GlyphFactory::create_standard_anchors(
    bool front_to_back, float apex_height) const {
    std::map<std::string, GlyphAnchorPosition> anchors;

    // Entry: bottom-left area, tangent going up and right
    float entry_z = front_to_back ? 0.3f : 0.7f;
    float exit_z = front_to_back ? 0.7f : 0.3f;

    anchors["entry"] = GlyphAnchorPosition{
        .position = Vec3(0.2f, 0.0f, entry_z),
        .tangent = Vec3(0.5f, 1.0f, 0.0f).normalized()
    };

    // Apex: top-center
    anchors["apex"] = GlyphAnchorPosition{
        .position = Vec3(0.5f, apex_height, 0.5f),
        .tangent = Vec3(1.0f, 0.0f, 0.0f).normalized()
    };

    // Exit: bottom-right area, tangent going up and right
    anchors["exit"] = GlyphAnchorPosition{
        .position = Vec3(0.8f, 0.0f, exit_z),
        .tangent = Vec3(0.5f, 1.0f, 0.0f).normalized()
    };

    return anchors;
}

std::vector<CubicBezier> GlyphFactory::create_basic_loop_path(
    const Vec3& entry, const Vec3& apex, const Vec3& exit,
    float loop_height, bool front_to_back) const {

    std::vector<CubicBezier> segments;

    (void)front_to_back;  // Reserved for future Z adjustments

    // Entry to apex curve
    Vec3 entry_tangent = Vec3(0.0f, loop_height * 0.5f, 0.0f);
    Vec3 apex_tangent_in = Vec3(-0.2f, 0.0f, 0.0f);

    CubicBezier entry_to_apex = CubicBezier::from_hermite(
        entry, entry_tangent, apex, apex_tangent_in);
    segments.push_back(entry_to_apex);

    // Apex to exit curve
    Vec3 apex_tangent_out = Vec3(0.2f, 0.0f, 0.0f);
    Vec3 exit_tangent = Vec3(0.0f, -loop_height * 0.5f, 0.0f);

    CubicBezier apex_to_exit = CubicBezier::from_hermite(
        apex, apex_tangent_out, exit, exit_tangent);
    segments.push_back(apex_to_exit);

    return segments;
}

StitchGlyph GlyphFactory::create_cast_on_glyph() const {
    StitchGlyph glyph;
    glyph.kind = FormKind::CastOn;

    // Cast-on has a base anchor instead of entry
    float apex_height = yarn_.loop_aspect_ratio;

    glyph.anchors["base"] = GlyphAnchorPosition{
        .position = Vec3(0.5f, -0.2f, 0.5f),
        .tangent = Vec3(0.0f, 1.0f, 0.0f)
    };

    glyph.anchors["apex"] = GlyphAnchorPosition{
        .position = Vec3(0.5f, apex_height * 0.5f, 0.5f),
        .tangent = Vec3(1.0f, 0.0f, 0.0f)
    };

    glyph.anchors["exit"] = GlyphAnchorPosition{
        .position = Vec3(0.8f, 0.0f, 0.6f),
        .tangent = Vec3(0.5f, 0.5f, 0.0f).normalized()
    };

    // Path from base up around to exit
    Vec3 base = glyph.anchors["base"].position;
    Vec3 apex = glyph.anchors["apex"].position;
    Vec3 exit = glyph.anchors["exit"].position;

    // Base to apex
    CubicBezier base_to_apex = CubicBezier::from_hermite(
        base, Vec3(0.0f, 0.3f, 0.0f),
        apex, Vec3(-0.2f, 0.0f, 0.0f));
    glyph.path_segments.push_back(base_to_apex);

    // Apex to exit
    CubicBezier apex_to_exit = CubicBezier::from_hermite(
        apex, Vec3(0.2f, 0.0f, 0.0f),
        exit, Vec3(0.0f, -0.2f, 0.0f));
    glyph.path_segments.push_back(apex_to_exit);

    return glyph;
}

StitchGlyph GlyphFactory::create_knit_glyph(bool twisted) const {
    StitchGlyph glyph;
    glyph.kind = FormKind::Knit;

    float apex_height = yarn_.loop_aspect_ratio * 0.5f;
    bool front_to_back = !twisted;  // Twisted goes back-to-front

    glyph.anchors = create_standard_anchors(front_to_back, apex_height);

    // Create path segments
    Vec3 entry = glyph.anchors["entry"].position;
    Vec3 apex = glyph.anchors["apex"].position;
    Vec3 exit = glyph.anchors["exit"].position;

    glyph.path_segments = create_basic_loop_path(entry, apex, exit, apex_height, front_to_back);

    return glyph;
}

StitchGlyph GlyphFactory::create_purl_glyph(bool twisted) const {
    StitchGlyph glyph;
    glyph.kind = FormKind::Purl;

    float apex_height = yarn_.loop_aspect_ratio * 0.5f;
    bool front_to_back = twisted;  // Purl is back-to-front normally

    glyph.anchors = create_standard_anchors(front_to_back, apex_height);

    Vec3 entry = glyph.anchors["entry"].position;
    Vec3 apex = glyph.anchors["apex"].position;
    Vec3 exit = glyph.anchors["exit"].position;

    glyph.path_segments = create_basic_loop_path(entry, apex, exit, apex_height, front_to_back);

    return glyph;
}

StitchGlyph GlyphFactory::create_slip_glyph() const {
    StitchGlyph glyph;
    glyph.kind = FormKind::Slip;

    // Slip stitch just passes by without forming a new loop shape
    // The apex is lower since the loop isn't really "formed"
    float apex_height = yarn_.loop_aspect_ratio * 0.3f;

    glyph.anchors["entry"] = GlyphAnchorPosition{
        .position = Vec3(0.2f, 0.0f, 0.5f),
        .tangent = Vec3(1.0f, 0.2f, 0.0f).normalized()
    };

    glyph.anchors["apex"] = GlyphAnchorPosition{
        .position = Vec3(0.5f, apex_height, 0.5f),
        .tangent = Vec3(1.0f, 0.0f, 0.0f)
    };

    glyph.anchors["exit"] = GlyphAnchorPosition{
        .position = Vec3(0.8f, 0.0f, 0.5f),
        .tangent = Vec3(1.0f, 0.2f, 0.0f).normalized()
    };

    // Simple curve passing through
    CubicBezier pass_through = CubicBezier::from_hermite(
        glyph.anchors["entry"].position, Vec3(0.3f, 0.1f, 0.0f),
        glyph.anchors["exit"].position, Vec3(0.3f, 0.1f, 0.0f));
    glyph.path_segments.push_back(pass_through);

    return glyph;
}

StitchGlyph GlyphFactory::create_bind_off_glyph() const {
    StitchGlyph glyph;
    glyph.kind = FormKind::BindOff;

    float apex_height = yarn_.loop_aspect_ratio * 0.4f;

    glyph.anchors["entry"] = GlyphAnchorPosition{
        .position = Vec3(0.2f, 0.0f, 0.4f),
        .tangent = Vec3(0.5f, 1.0f, 0.0f).normalized()
    };

    glyph.anchors["apex"] = GlyphAnchorPosition{
        .position = Vec3(0.5f, apex_height, 0.5f),
        .tangent = Vec3(1.0f, 0.0f, 0.0f)
    };

    glyph.anchors["end"] = GlyphAnchorPosition{
        .position = Vec3(0.8f, apex_height * 0.5f, 0.5f),
        .tangent = Vec3(1.0f, 0.0f, 0.0f)
    };

    Vec3 entry = glyph.anchors["entry"].position;
    Vec3 apex = glyph.anchors["apex"].position;
    Vec3 end = glyph.anchors["end"].position;

    CubicBezier entry_to_apex = CubicBezier::from_hermite(
        entry, Vec3(0.0f, 0.3f, 0.0f),
        apex, Vec3(-0.2f, 0.0f, 0.0f));
    glyph.path_segments.push_back(entry_to_apex);

    CubicBezier apex_to_end = CubicBezier::from_hermite(
        apex, Vec3(0.2f, 0.0f, 0.0f),
        end, Vec3(0.1f, 0.0f, 0.0f));
    glyph.path_segments.push_back(apex_to_end);

    return glyph;
}

StitchGlyph GlyphFactory::create_yarn_over_glyph() const {
    StitchGlyph glyph;
    glyph.kind = FormKind::YarnOver;

    // Yarn over wraps around without going through a parent loop
    float apex_height = yarn_.loop_aspect_ratio * 0.6f;

    glyph.anchors["entry"] = GlyphAnchorPosition{
        .position = Vec3(0.1f, 0.3f, 0.3f),
        .tangent = Vec3(0.5f, 0.5f, 0.3f).normalized()
    };

    glyph.anchors["apex"] = GlyphAnchorPosition{
        .position = Vec3(0.5f, apex_height, 0.5f),
        .tangent = Vec3(1.0f, 0.0f, 0.0f)
    };

    glyph.anchors["exit"] = GlyphAnchorPosition{
        .position = Vec3(0.9f, 0.3f, 0.7f),
        .tangent = Vec3(0.5f, 0.5f, -0.3f).normalized()
    };

    Vec3 entry = glyph.anchors["entry"].position;
    Vec3 apex = glyph.anchors["apex"].position;
    Vec3 exit = glyph.anchors["exit"].position;

    // Wrap around path
    CubicBezier entry_to_apex = CubicBezier::from_hermite(
        entry, Vec3(0.2f, 0.3f, 0.1f),
        apex, Vec3(-0.2f, 0.0f, 0.0f));
    glyph.path_segments.push_back(entry_to_apex);

    CubicBezier apex_to_exit = CubicBezier::from_hermite(
        apex, Vec3(0.2f, 0.0f, 0.0f),
        exit, Vec3(0.2f, 0.3f, -0.1f));
    glyph.path_segments.push_back(apex_to_exit);

    return glyph;
}

StitchGlyph GlyphFactory::create_kfb_glyph(int loop_index) const {
    StitchGlyph glyph;
    glyph.kind = FormKind::KFB;

    float apex_height = yarn_.loop_aspect_ratio * 0.5f;

    if (loop_index == 0) {
        // Front loop (first one created)
        glyph.anchors["entry"] = GlyphAnchorPosition{
            .position = Vec3(0.15f, 0.0f, 0.3f),
            .tangent = Vec3(0.3f, 1.0f, 0.0f).normalized()
        };

        glyph.anchors["apex"] = GlyphAnchorPosition{
            .position = Vec3(0.35f, apex_height, 0.5f),
            .tangent = Vec3(1.0f, 0.0f, 0.0f)
        };

        glyph.anchors["exit"] = GlyphAnchorPosition{
            .position = Vec3(0.5f, 0.0f, 0.5f),
            .tangent = Vec3(0.3f, 1.0f, 0.0f).normalized()
        };
    } else {
        // Back loop (second one created)
        glyph.anchors["entry"] = GlyphAnchorPosition{
            .position = Vec3(0.5f, 0.0f, 0.5f),
            .tangent = Vec3(0.3f, 1.0f, 0.0f).normalized()
        };

        glyph.anchors["apex"] = GlyphAnchorPosition{
            .position = Vec3(0.65f, apex_height, 0.5f),
            .tangent = Vec3(1.0f, 0.0f, 0.0f)
        };

        glyph.anchors["exit"] = GlyphAnchorPosition{
            .position = Vec3(0.85f, 0.0f, 0.7f),
            .tangent = Vec3(0.3f, 1.0f, 0.0f).normalized()
        };
    }

    Vec3 entry = glyph.anchors["entry"].position;
    Vec3 apex = glyph.anchors["apex"].position;
    Vec3 exit = glyph.anchors["exit"].position;

    glyph.path_segments = create_basic_loop_path(entry, apex, exit, apex_height, true);

    return glyph;
}

StitchGlyph GlyphFactory::create_m1l_glyph() const {
    StitchGlyph glyph;
    glyph.kind = FormKind::M1L;

    // M1L lifts the bar between stitches from the left
    float apex_height = yarn_.loop_aspect_ratio * 0.4f;

    glyph.anchors["entry"] = GlyphAnchorPosition{
        .position = Vec3(0.2f, -0.1f, 0.4f),
        .tangent = Vec3(0.2f, 1.0f, 0.0f).normalized()
    };

    glyph.anchors["apex"] = GlyphAnchorPosition{
        .position = Vec3(0.5f, apex_height, 0.5f),
        .tangent = Vec3(1.0f, 0.0f, 0.0f)
    };

    glyph.anchors["exit"] = GlyphAnchorPosition{
        .position = Vec3(0.8f, 0.0f, 0.6f),
        .tangent = Vec3(0.5f, 0.5f, 0.0f).normalized()
    };

    Vec3 entry = glyph.anchors["entry"].position;
    Vec3 apex = glyph.anchors["apex"].position;
    Vec3 exit = glyph.anchors["exit"].position;

    glyph.path_segments = create_basic_loop_path(entry, apex, exit, apex_height, true);

    return glyph;
}

StitchGlyph GlyphFactory::create_m1r_glyph() const {
    StitchGlyph glyph;
    glyph.kind = FormKind::M1R;

    // M1R lifts the bar between stitches from the right
    float apex_height = yarn_.loop_aspect_ratio * 0.4f;

    glyph.anchors["entry"] = GlyphAnchorPosition{
        .position = Vec3(0.2f, 0.0f, 0.6f),
        .tangent = Vec3(0.5f, 0.5f, 0.0f).normalized()
    };

    glyph.anchors["apex"] = GlyphAnchorPosition{
        .position = Vec3(0.5f, apex_height, 0.5f),
        .tangent = Vec3(1.0f, 0.0f, 0.0f)
    };

    glyph.anchors["exit"] = GlyphAnchorPosition{
        .position = Vec3(0.8f, -0.1f, 0.4f),
        .tangent = Vec3(0.2f, 1.0f, 0.0f).normalized()
    };

    Vec3 entry = glyph.anchors["entry"].position;
    Vec3 apex = glyph.anchors["apex"].position;
    Vec3 exit = glyph.anchors["exit"].position;

    glyph.path_segments = create_basic_loop_path(entry, apex, exit, apex_height, true);

    return glyph;
}

StitchGlyph GlyphFactory::create_k2tog_glyph() const {
    StitchGlyph glyph;
    glyph.kind = FormKind::K2tog;

    // K2tog consumes 2 parent loops, creating 1 child
    float apex_height = yarn_.loop_aspect_ratio * 0.5f;

    // Entry comes from the middle of two stitches
    glyph.anchors["entry"] = GlyphAnchorPosition{
        .position = Vec3(0.0f, 0.0f, 0.3f),
        .tangent = Vec3(0.5f, 1.0f, 0.0f).normalized()
    };

    glyph.anchors["apex"] = GlyphAnchorPosition{
        .position = Vec3(0.5f, apex_height, 0.5f),
        .tangent = Vec3(1.0f, 0.0f, 0.0f)
    };

    glyph.anchors["exit"] = GlyphAnchorPosition{
        .position = Vec3(1.0f, 0.0f, 0.7f),
        .tangent = Vec3(0.5f, 1.0f, 0.0f).normalized()
    };

    Vec3 entry = glyph.anchors["entry"].position;
    Vec3 apex = glyph.anchors["apex"].position;
    Vec3 exit = glyph.anchors["exit"].position;

    glyph.path_segments = create_basic_loop_path(entry, apex, exit, apex_height, true);

    return glyph;
}

StitchGlyph GlyphFactory::create_ssk_glyph() const {
    StitchGlyph glyph;
    glyph.kind = FormKind::SSK;

    // SSK consumes 2 parent loops, creating 1 child (leans left)
    float apex_height = yarn_.loop_aspect_ratio * 0.5f;

    glyph.anchors["entry"] = GlyphAnchorPosition{
        .position = Vec3(0.0f, 0.0f, 0.7f),
        .tangent = Vec3(0.5f, 1.0f, -0.2f).normalized()
    };

    glyph.anchors["apex"] = GlyphAnchorPosition{
        .position = Vec3(0.5f, apex_height, 0.5f),
        .tangent = Vec3(1.0f, 0.0f, 0.0f)
    };

    glyph.anchors["exit"] = GlyphAnchorPosition{
        .position = Vec3(1.0f, 0.0f, 0.3f),
        .tangent = Vec3(0.5f, 1.0f, 0.2f).normalized()
    };

    Vec3 entry = glyph.anchors["entry"].position;
    Vec3 apex = glyph.anchors["apex"].position;
    Vec3 exit = glyph.anchors["exit"].position;

    glyph.path_segments = create_basic_loop_path(entry, apex, exit, apex_height, true);

    return glyph;
}

StitchGlyph GlyphFactory::create_s2kp_glyph() const {
    StitchGlyph glyph;
    glyph.kind = FormKind::S2KP;

    // S2KP consumes 3 parent loops, creating 1 child (centered decrease)
    float apex_height = yarn_.loop_aspect_ratio * 0.5f;

    glyph.anchors["entry"] = GlyphAnchorPosition{
        .position = Vec3(-0.25f, 0.0f, 0.5f),
        .tangent = Vec3(0.5f, 1.0f, 0.0f).normalized()
    };

    glyph.anchors["apex"] = GlyphAnchorPosition{
        .position = Vec3(0.5f, apex_height, 0.5f),
        .tangent = Vec3(1.0f, 0.0f, 0.0f)
    };

    glyph.anchors["exit"] = GlyphAnchorPosition{
        .position = Vec3(1.25f, 0.0f, 0.5f),
        .tangent = Vec3(0.5f, 1.0f, 0.0f).normalized()
    };

    Vec3 entry = glyph.anchors["entry"].position;
    Vec3 apex = glyph.anchors["apex"].position;
    Vec3 exit = glyph.anchors["exit"].position;

    glyph.path_segments = create_basic_loop_path(entry, apex, exit, apex_height, true);

    return glyph;
}

}  // namespace yarnpath
