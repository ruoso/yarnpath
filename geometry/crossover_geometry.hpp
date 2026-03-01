#ifndef YARNPATH_GEOMETRY_CROSSOVER_GEOMETRY_HPP
#define YARNPATH_GEOMETRY_CROSSOVER_GEOMETRY_HPP

#include <math/vec3.hpp>
#include <vector>
#include <set>

namespace yarnpath {

// Crossover data for a single crossing between parent and child yarn
struct CrossoverData {
    Vec3 entry;           // Where the crossing starts (approach side)
    Vec3 exit;            // Where the crossing ends (departure side)
    Vec3 exit_direction;  // Tangent direction at exit
};

// A crossover slot on the parent loop path, unbiased toward entry or exit.
// The parent distributes these evenly; children claim the nearest available slot
// and decide whether to use it as entry (parent→child) or exit (child→parent).
struct CrossoverSlot {
    Vec3 position;          // Center of crossing on parent loop path
    Vec3 crossing_normal;   // Fabric normal direction for parent/child offset
    Vec3 tangent;           // Parent loop tangent at this point
};

// Claim the nearest unclaimed crossover slot for a child crossing.
// as_entry=true: parent→child direction (entry = parent side, exit = child side)
// as_entry=false: child→parent direction (entry = child side, exit = parent side)
CrossoverData claim_nearest_slot(
    const std::vector<CrossoverSlot>& slots,
    std::set<size_t>& claimed,
    const Vec3& child_position,
    float yarn_compressed_radius,
    bool as_entry);

}  // namespace yarnpath

#endif // YARNPATH_GEOMETRY_CROSSOVER_GEOMETRY_HPP
