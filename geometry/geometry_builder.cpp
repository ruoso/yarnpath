#include "geometry_builder.hpp"
#include "logging.hpp"
#include <algorithm>
#include <cmath>

namespace yarnpath {

GeometryBuilder::GeometryBuilder(
    const YarnPath& yarn_path,
    const YarnProperties& yarn,
    const Gauge& gauge,
    const FabricSurface& surface)
    : yarn_path_(yarn_path)
    , yarn_(yarn)
    , gauge_(gauge)
    , surface_(surface)
    , glyph_factory_(yarn, gauge) {}

GeometryPath GeometryBuilder::build() {
    auto log = yarnpath::logging::get_logger();
    GeometryPath result;

    if (yarn_path_.loops().empty()) {
        log->debug("GeometryBuilder: no loops to process");
        return result;
    }

    log->debug("GeometryBuilder: building geometry for {} loops", yarn_path_.loops().size());

    // Phase 1: Initialize cast-on positions
    log->debug("GeometryBuilder: Phase 1 - initializing cast-on positions");
    initialize_cast_on_positions();
    log->debug("GeometryBuilder: initialized {} cast-on positions", loop_positions_.size());

    // Phase 2: Propagate positions through topology
    log->debug("GeometryBuilder: Phase 2 - propagating positions");
    propagate_positions();
    log->debug("GeometryBuilder: propagated {} total loop positions", loop_positions_.size());

    // Phase 3: Generate glyph geometry for each loop
    log->debug("GeometryBuilder: Phase 3 - generating glyph geometries");
    generate_glyph_geometries();
    log->debug("GeometryBuilder: generated {} loop glyphs", loop_glyphs_.size());

    // Phase 4: Create anchor geometries from glyphs
    log->debug("GeometryBuilder: Phase 4 - creating anchor geometries");
    create_anchor_geometries();
    log->debug("GeometryBuilder: created {} anchor geometries", anchor_geometries_.size());

    // Phase 5: Connect segments between anchors
    log->debug("GeometryBuilder: Phase 5 - connecting segments");
    auto segments = connect_segments();
    log->debug("GeometryBuilder: connected {} segments", segments.size());

    // Phase 6: Apply constraints
    log->debug("GeometryBuilder: Phase 6 - applying constraints");
    apply_constraints(segments);

    // Build output
    for (const auto& [loop_id, pos] : loop_positions_) {
        result.loop_positions_.push_back(pos);
        result.loop_position_index_[loop_id] = result.loop_positions_.size() - 1;
    }

    for (const auto& [anchor_id, geom] : anchor_geometries_) {
        result.anchors_.push_back(geom);
        result.anchor_index_[anchor_id] = result.anchors_.size() - 1;
    }

    for (auto& seg : segments) {
        result.segment_index_[seg.segment_id] = result.segments_.size();
        result.segments_.push_back(std::move(seg));
    }

    log->debug("GeometryBuilder: build complete - {} loops, {} anchors, {} segments",
               result.loop_positions_.size(), result.anchors_.size(), result.segments_.size());
    return result;
}

void GeometryBuilder::initialize_cast_on_positions() {
    uint32_t column = 0;

    for (const auto& loop : yarn_path_.loops()) {
        if (loop.kind == FormKind::CastOn) {
            LoopPosition pos;
            pos.loop_id = loop.id;
            pos.row = 0;
            pos.column = column;
            pos.u = gauge_.stitch_to_u(static_cast<float>(column));
            pos.v = 0.0f;

            loop_positions_[loop.id] = pos;
            ++column;
        }
    }
}

void GeometryBuilder::propagate_positions() {
    auto log = yarnpath::logging::get_logger();
    // BFS from positioned loops
    std::queue<LoopId> to_process;
    std::set<LoopId> processed;
    std::set<LoopId> queued;  // Track what's been queued to avoid duplicates

    // Start with all positioned loops (cast-on)
    for (const auto& [loop_id, _] : loop_positions_) {
        processed.insert(loop_id);
        // Add children of cast-on to queue
        const Loop* loop = yarn_path_.get_loop(loop_id);
        if (loop) {
            for (LoopId child_id : loop->child_loops) {
                if (queued.count(child_id) == 0) {
                    to_process.push(child_id);
                    queued.insert(child_id);
                }
            }
        }
    }

    // Also queue all loops with no parents (YarnOver, M1L, M1R) - they can be positioned immediately
    for (const auto& loop : yarn_path_.loops()) {
        if (loop.parent_loops.empty() && processed.count(loop.id) == 0 && queued.count(loop.id) == 0) {
            to_process.push(loop.id);
            queued.insert(loop.id);
        }
    }

    size_t iteration = 0;
    size_t max_iterations = yarn_path_.loops().size() * 2;  // Safety limit

    while (!to_process.empty() && iteration < max_iterations) {
        ++iteration;
        LoopId loop_id = to_process.front();
        to_process.pop();

        if (processed.count(loop_id) > 0) {
            continue;
        }

        const Loop* loop = yarn_path_.get_loop(loop_id);
        if (!loop) {
            continue;
        }

        // Check if all parents are positioned
        if (!all_parents_positioned(loop_id)) {
            // Re-queue for later
            to_process.push(loop_id);
            continue;
        }

        // Compute position from parents
        LoopPosition pos = compute_child_position(*loop);
        loop_positions_[loop_id] = pos;
        processed.insert(loop_id);

        // Queue children
        for (LoopId child_id : loop->child_loops) {
            if (processed.count(child_id) == 0 && queued.count(child_id) == 0) {
                to_process.push(child_id);
                queued.insert(child_id);
            }
        }
    }

    if (iteration >= max_iterations) {
        log->warn("GeometryBuilder: propagate_positions hit iteration limit, {} loops processed out of {}",
                  processed.size(), yarn_path_.loops().size());
    }
}

bool GeometryBuilder::all_parents_positioned(LoopId loop_id) const {
    const Loop* loop = yarn_path_.get_loop(loop_id);
    if (!loop) {
        return false;
    }

    for (LoopId parent_id : loop->parent_loops) {
        if (loop_positions_.count(parent_id) == 0) {
            return false;
        }
    }
    return true;
}

LoopPosition GeometryBuilder::compute_child_position(const Loop& loop) const {
    LoopPosition pos;
    pos.loop_id = loop.id;

    if (loop.parent_loops.empty()) {
        // No parents (yarn over, etc.) - use stitch_id to estimate position
        // This is a fallback case
        pos.row = 1;
        pos.column = 0;
        pos.u = 0.0f;
        pos.v = gauge_.row_to_v(1.0f);
        return pos;
    }

    // Compute centroid of parent positions
    float sum_u = 0.0f;
    float max_v = 0.0f;
    uint32_t max_row = 0;

    for (LoopId parent_id : loop.parent_loops) {
        auto it = loop_positions_.find(parent_id);
        if (it != loop_positions_.end()) {
            sum_u += it->second.u;
            max_v = std::max(max_v, it->second.v);
            max_row = std::max(max_row, it->second.row);
        }
    }

    float avg_u = sum_u / static_cast<float>(loop.parent_loops.size());

    pos.row = max_row + 1;
    pos.column = static_cast<uint32_t>(std::round(avg_u * gauge_.stitches_per_unit));
    pos.u = avg_u;
    pos.v = max_v + gauge_.row_height();

    // Apply FormKind-specific adjustments
    switch (loop.kind) {
        case FormKind::K2tog:
        case FormKind::SSK:
            // Decrease: position at centroid (already computed)
            break;

        case FormKind::S2KP:
            // Centered decrease: position at middle parent
            if (loop.parent_loops.size() == 3) {
                auto it = loop_positions_.find(loop.parent_loops[1]);
                if (it != loop_positions_.end()) {
                    pos.u = it->second.u;
                    pos.column = it->second.column;
                }
            }
            break;

        case FormKind::YarnOver:
        case FormKind::M1L:
        case FormKind::M1R:
            // Increases: these add width, neighbors will adjust
            break;

        default:
            // Standard stitches: inherit parent position
            break;
    }

    return pos;
}

void GeometryBuilder::generate_glyph_geometries() {
    for (const auto& loop : yarn_path_.loops()) {
        StitchGlyph glyph = glyph_factory_.create_glyph(loop.kind, loop);

        // Get loop position
        auto pos_it = loop_positions_.find(loop.id);
        if (pos_it == loop_positions_.end()) {
            continue;
        }

        const LoopPosition& pos = pos_it->second;

        // Transform glyph to world coordinates
        Vec3 origin = to_world_position(pos.u, pos.v, 0.0f);

        // Scale based on gauge and yarn
        Vec3 scale(
            gauge_.stitch_width(),
            gauge_.row_height() * yarn_.loop_aspect_ratio,
            gauge_.fabric_thickness * yarn_.radius
        );

        loop_glyphs_[loop.id] = glyph.transformed(origin, scale);
    }
}

void GeometryBuilder::create_anchor_geometries() {
    for (const auto& anchor_node : yarn_path_.anchors()) {
        AnchorGeometry geom;
        geom.anchor_id = anchor_node.id;

        // Determine which loop this anchor belongs to
        LoopId loop_id = 0;
        std::string anchor_name;

        std::visit([&](auto&& anchor) {
            using T = std::decay_t<decltype(anchor)>;
            if constexpr (std::is_same_v<T, anchor::LoopForm>) {
                loop_id = anchor.loop_id;
                anchor_name = "form";
            } else if constexpr (std::is_same_v<T, anchor::LoopApex>) {
                loop_id = anchor.loop_id;
                anchor_name = "apex";
            } else if constexpr (std::is_same_v<T, anchor::LoopEntry>) {
                loop_id = anchor.loop_id;
                anchor_name = "entry";
            } else if constexpr (std::is_same_v<T, anchor::LoopExit>) {
                loop_id = anchor.loop_id;
                anchor_name = "exit";
            } else if constexpr (std::is_same_v<T, anchor::CastOnBase>) {
                loop_id = anchor.loop_id;
                anchor_name = "base";
            } else if constexpr (std::is_same_v<T, anchor::BindOffEnd>) {
                loop_id = anchor.loop_id;
                anchor_name = "end";
            } else if constexpr (std::is_same_v<T, anchor::YarnOverApex>) {
                loop_id = anchor.loop_id;
                anchor_name = "apex";
            } else if constexpr (std::is_same_v<T, anchor::CrossOver>) {
                // Cable crossover - use first over loop
                if (!anchor.over_loops.empty()) {
                    loop_id = anchor.over_loops[0];
                }
                anchor_name = "crossover";
            }
        }, anchor_node.anchor);

        // Get glyph for this loop
        auto glyph_it = loop_glyphs_.find(loop_id);
        if (glyph_it != loop_glyphs_.end()) {
            const auto* glyph_anchor = glyph_it->second.get_anchor(anchor_name);
            if (glyph_anchor) {
                geom.position = glyph_anchor->position;
                geom.tangent = glyph_anchor->tangent;
            } else {
                // Fallback: use loop position
                auto pos_it = loop_positions_.find(loop_id);
                if (pos_it != loop_positions_.end()) {
                    geom.position = to_world_position(
                        pos_it->second.u + gauge_.stitch_width() * 0.5f,
                        pos_it->second.v + gauge_.row_height() * 0.5f,
                        0.0f
                    );
                    geom.tangent = vec3::unit_x();
                }
            }
        } else {
            // No glyph - estimate from position
            auto pos_it = loop_positions_.find(loop_id);
            if (pos_it != loop_positions_.end()) {
                geom.position = to_world_position(
                    pos_it->second.u + gauge_.stitch_width() * 0.5f,
                    pos_it->second.v + gauge_.row_height() * 0.5f,
                    0.0f
                );
                geom.tangent = vec3::unit_x();
            } else {
                geom.position = vec3::zero();
                geom.tangent = vec3::unit_x();
            }
        }

        // Use layout hints if provided
        if (anchor_node.hint_x || anchor_node.hint_y || anchor_node.hint_z) {
            auto pos_it = loop_positions_.find(loop_id);
            if (pos_it != loop_positions_.end()) {
                float hint_x = anchor_node.hint_x.value_or(0.5f);
                float hint_y = anchor_node.hint_y.value_or(0.5f);
                float hint_z = anchor_node.hint_z.value_or(0.5f);

                geom.position = to_world_position(
                    pos_it->second.u + hint_x * gauge_.stitch_width(),
                    pos_it->second.v + hint_y * gauge_.row_height(),
                    (hint_z - 0.5f) * gauge_.fabric_thickness * yarn_.radius
                );
            }
        }

        // Surface normal
        auto pos_it = loop_positions_.find(loop_id);
        if (pos_it != loop_positions_.end()) {
            geom.normal = surface_.normal(pos_it->second.u, pos_it->second.v);
        } else {
            geom.normal = vec3::unit_z();
        }

        anchor_geometries_[anchor_node.id] = geom;
    }
}

std::vector<SegmentGeometry> GeometryBuilder::connect_segments() {
    std::vector<SegmentGeometry> result;

    for (const auto& yarn_seg : yarn_path_.segments()) {
        auto from_it = anchor_geometries_.find(yarn_seg.from_anchor);
        auto to_it = anchor_geometries_.find(yarn_seg.to_anchor);

        if (from_it == anchor_geometries_.end() || to_it == anchor_geometries_.end()) {
            continue;
        }

        SegmentGeometry seg_geom;
        seg_geom.segment_id = yarn_seg.id;
        seg_geom.curve = create_connecting_curve(from_it->second, to_it->second, yarn_seg);
        seg_geom.arc_length = seg_geom.curve.total_arc_length();
        seg_geom.max_curvature = seg_geom.curve.max_curvature();

        result.push_back(std::move(seg_geom));
    }

    return result;
}

BezierSpline GeometryBuilder::create_connecting_curve(
    const AnchorGeometry& from,
    const AnchorGeometry& to,
    const YarnSegment& segment) const {

    BezierSpline spline;

    // Adjust Z based on segment type (ThroughLoop pass mode)
    float z_offset_from = 0.0f;
    float z_offset_to = 0.0f;

    std::visit([&](auto&& seg_type) {
        using T = std::decay_t<decltype(seg_type)>;
        if constexpr (std::is_same_v<T, segment::ThroughLoop>) {
            z_offset_to = get_z_offset(seg_type.mode);
        }
    }, segment.segment_type);

    Vec3 adjusted_from = from.position;
    adjusted_from.z += z_offset_from;

    Vec3 adjusted_to = to.position;
    adjusted_to.z += z_offset_to;

    // Scale tangent based on distance
    float dist = adjusted_from.distance_to(adjusted_to);
    float tangent_scale = std::max(dist * 0.4f, yarn_.min_bend_radius);

    Vec3 tangent_from = from.tangent * tangent_scale;
    Vec3 tangent_to = to.tangent * tangent_scale;

    CubicBezier curve = CubicBezier::from_hermite(
        adjusted_from, tangent_from,
        adjusted_to, tangent_to
    );

    spline.add_segment(curve);
    return spline;
}

void GeometryBuilder::apply_constraints(std::vector<SegmentGeometry>& segments) {
    float max_curvature = yarn_.max_curvature();

    for (auto& seg : segments) {
        if (seg.max_curvature > max_curvature) {
            seg.curve.clamp_curvature(max_curvature);
            seg.arc_length = seg.curve.total_arc_length();
            seg.max_curvature = seg.curve.max_curvature();
        }
    }
}

float GeometryBuilder::get_z_offset(PassMode mode) const {
    float thickness = gauge_.fabric_thickness * yarn_.radius;

    switch (mode) {
        case PassMode::KnitWise:
            // Front to back
            return -thickness * 0.3f;
        case PassMode::PurlWise:
            // Back to front
            return thickness * 0.3f;
        case PassMode::ThroughBackLoop:
            // Twisted knit
            return -thickness * 0.4f;
        case PassMode::ThroughFrontLoop:
            // Twisted purl
            return thickness * 0.4f;
        default:
            return 0.0f;
    }
}

Vec3 GeometryBuilder::to_world_position(float u, float v, float z) const {
    return surface_.local_to_world(u, v, 0.0f, 0.0f, z);
}

}  // namespace yarnpath
