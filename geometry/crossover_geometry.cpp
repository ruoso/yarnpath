#include "crossover_geometry.hpp"
#include <limits>

namespace yarnpath {

CrossoverData claim_nearest_slot(
    const std::vector<CrossoverSlot>& slots,
    std::set<size_t>& claimed,
    const Vec3& child_position,
    float yarn_compressed_radius,
    bool as_entry) {

    size_t best_idx = 0;
    float best_dist = std::numeric_limits<float>::max();

    for (size_t i = 0; i < slots.size(); ++i) {
        if (claimed.count(i)) continue;
        float dist = (slots[i].position - child_position).length();
        if (dist < best_dist) {
            best_dist = dist;
            best_idx = i;
        }
    }

    claimed.insert(best_idx);
    const auto& slot = slots[best_idx];

    CrossoverData crossover;
    if (as_entry) {
        // Parent→child: approach from parent side, exit to child side
        crossover.entry = slot.position + slot.crossing_normal * yarn_compressed_radius;
        crossover.exit = slot.position - slot.crossing_normal * yarn_compressed_radius;
    } else {
        // Child→parent: approach from child side, exit to parent side
        crossover.entry = slot.position - slot.crossing_normal * yarn_compressed_radius;
        crossover.exit = slot.position + slot.crossing_normal * yarn_compressed_radius;
    }
    crossover.exit_direction = slot.tangent;
    return crossover;
}

}  // namespace yarnpath
