#include "crossover_geometry.hpp"
#include <limits>

namespace yarnpath {

CrossoverData claim_nearest_slot(
    const std::vector<CrossoverSlot>& slots,
    std::set<size_t>& claimed,
    const Vec3& child_position,
    float yarn_compressed_radius,
    bool as_entry,
    const Vec3& crossing_wale,
    const Vec3& travel_direction) {

    // Select the slot on the correct side of the child:
    //   Entry: approach side (opposite to travel) → most negative travel_proj
    //   Exit: departure side (along travel) → most positive travel_proj
    size_t best_idx = 0;
    float best_score = std::numeric_limits<float>::max();

    for (size_t i = 0; i < slots.size(); ++i) {
        if (claimed.count(i)) continue;
        float travel_proj = (slots[i].position - child_position).dot(travel_direction);
        // For entry: minimize travel_proj (pick most behind slot)
        // For exit: maximize travel_proj (pick most ahead slot)
        float score = as_entry ? travel_proj : -travel_proj;
        if (score < best_score) {
            best_score = score;
            best_idx = i;
        }
    }

    claimed.insert(best_idx);
    const auto& slot = slots[best_idx];

    // Offset entry/exit along the wale direction (through the parent loop
    // opening). Entry is below the opening (-wale), exit is above (+wale).
    CrossoverData crossover;
    if (as_entry) {
        // Parent→child: entry below opening, exit above
        crossover.entry = slot.position - crossing_wale * yarn_compressed_radius;
        crossover.exit = slot.position + crossing_wale * yarn_compressed_radius;
    } else {
        // Child→parent: entry above opening, exit below
        crossover.entry = slot.position + crossing_wale * yarn_compressed_radius;
        crossover.exit = slot.position - crossing_wale * yarn_compressed_radius;
    }
    crossover.exit_direction = slot.tangent;
    return crossover;
}

}  // namespace yarnpath
