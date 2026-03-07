#include "geometry_build_state.hpp"
#include <algorithm>

namespace yarnpath {

GeometryBuildState::GeometryBuildState(const YarnProperties& yarn_, const Gauge& gauge_)
    : max_curvature(std::min(yarn_.max_curvature(), 1.0f / yarn_.compressed_radius))
    , yarn_compressed_radius(yarn_.compressed_radius)
    , yarn_compressed_diameter(yarn_.compressed_radius * 2.0f)
    , loop_dims(LoopDimensions::calculate(yarn_, gauge_))
    , effective_loop_height(loop_dims.loop_height * yarn_.loop_size_factor())
    , effective_stitch_width(loop_dims.loop_width)
    , effective_opening_diameter(loop_dims.opening_diameter)
    , yarn(yarn_)
    , gauge(gauge_)
{}

}  // namespace yarnpath
