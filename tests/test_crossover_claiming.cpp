#include <gtest/gtest.h>
#include <geometry/crossover_geometry.hpp>

#include <cmath>
#include <set>

using namespace yarnpath;

// ---------------------------------------------------------------------------
// Helper: create a row of synthetic crossover slots
// ---------------------------------------------------------------------------
static std::vector<CrossoverSlot> make_slots(
    int count, const Vec3& center, const Vec3& tangent, const Vec3& normal, float spacing) {
    std::vector<CrossoverSlot> slots;
    for (int i = 0; i < count; ++i) {
        float offset = (static_cast<float>(i) - (count - 1) / 2.0f) * spacing;
        CrossoverSlot slot;
        slot.position = center + tangent * offset;
        slot.tangent = tangent;
        slot.crossing_normal = normal;
        slots.push_back(slot);
    }
    return slots;
}

// ---------------------------------------------------------------------------
// Test 1: Entry and exit are on opposite sides along wale direction
// ---------------------------------------------------------------------------
TEST(CrossoverClaimingTest, EntryExitOppositeWaleDirections) {
    Vec3 wale(0, 0, 1);   // crossing_wale direction
    Vec3 tangent(1, 0, 0);
    auto slots = make_slots(1, Vec3(0, 5, 0), tangent, wale, 1.0f);
    float radius = 0.75f;
    Vec3 travel(1, 0, 0);

    // as_entry=true: entry below (-wale), exit above (+wale)
    {
        std::set<size_t> claimed;
        auto xover = claim_nearest_slot(slots, claimed, Vec3(0, 5, 0), radius, true, wale, travel);
        float entry_z = xover.entry.dot(wale);
        float exit_z = xover.exit.dot(wale);
        EXPECT_LT(entry_z, exit_z) << "Entry should be below (-wale), exit above (+wale)";
    }

    // as_entry=false: entry above (+wale), exit below (-wale)
    {
        std::set<size_t> claimed;
        auto xover = claim_nearest_slot(slots, claimed, Vec3(0, 5, 0), radius, false, wale, travel);
        float entry_z = xover.entry.dot(wale);
        float exit_z = xover.exit.dot(wale);
        EXPECT_GT(entry_z, exit_z) << "Entry should be above (+wale), exit below (-wale)";
    }
}

// ---------------------------------------------------------------------------
// Test 2: Offset magnitude is yarn_compressed_radius
// ---------------------------------------------------------------------------
TEST(CrossoverClaimingTest, OffsetMagnitudeIsYarnRadius) {
    Vec3 normal(0, 0, 1);
    Vec3 tangent(1, 0, 0);
    Vec3 center(0, 5, 0);
    auto slots = make_slots(1, center, tangent, normal, 1.0f);
    float radius = 0.75f;
    Vec3 travel(1, 0, 0);

    std::set<size_t> claimed;
    auto xover = claim_nearest_slot(slots, claimed, center, radius, true, normal, travel);

    // Entry and exit should be exactly radius away from slot center
    float entry_dist = (xover.entry - center).length();
    float exit_dist = (xover.exit - center).length();
    EXPECT_NEAR(entry_dist, radius, 1e-5f);
    EXPECT_NEAR(exit_dist, radius, 1e-5f);
}

// ---------------------------------------------------------------------------
// Test 3: Exit direction inherits parent tangent
// ---------------------------------------------------------------------------
TEST(CrossoverClaimingTest, ExitDirectionInheritsTangent) {
    Vec3 tangent(0.707f, 0.707f, 0);  // 45-degree tangent
    Vec3 normal(0, 0, 1);
    auto slots = make_slots(1, Vec3(0, 5, 0), tangent, normal, 1.0f);
    Vec3 travel(1, 0, 0);

    std::set<size_t> claimed;
    auto xover = claim_nearest_slot(slots, claimed, Vec3(0, 5, 0), 0.75f, true, normal, travel);

    EXPECT_NEAR(xover.exit_direction.x, tangent.x, 1e-5f);
    EXPECT_NEAR(xover.exit_direction.y, tangent.y, 1e-5f);
    EXPECT_NEAR(xover.exit_direction.z, tangent.z, 1e-5f);
}

// ---------------------------------------------------------------------------
// Test 4: Entry claims slot on approach side, exit on departure side
// ---------------------------------------------------------------------------
TEST(CrossoverClaimingTest, EntryOnApproachSideExitOnDepartureSide) {
    Vec3 tangent(1, 0, 0);
    Vec3 normal(0, 0, 1);
    // 2 slots spread along X at X=-1 and X=+1
    auto slots = make_slots(2, Vec3(0, 5, 0), tangent, normal, 2.0f);
    Vec3 child_pos(0, 5, 0);
    Vec3 travel(1, 0, 0);  // traveling in +X

    std::set<size_t> claimed;
    // Entry should be on approach side (behind, -X) → slot at X=-1
    auto entry = claim_nearest_slot(slots, claimed, child_pos, 0.75f, true, normal, travel);
    // Exit should be on departure side (ahead, +X) → slot at X=+1
    auto exit = claim_nearest_slot(slots, claimed, child_pos, 0.75f, false, normal, travel);

    EXPECT_LT(entry.entry.x, exit.exit.x)
        << "Entry crossover should be behind exit crossover along travel direction";
}

// ---------------------------------------------------------------------------
// Test 5: Claimed slots are not reusable
// ---------------------------------------------------------------------------
TEST(CrossoverClaimingTest, ClaimedSlotsNotReusable) {
    Vec3 tangent(1, 0, 0);
    Vec3 normal(0, 0, 1);
    auto slots = make_slots(3, Vec3(0, 5, 0), tangent, normal, 2.0f);
    Vec3 travel(1, 0, 0);

    // Child near center slot (index 1)
    Vec3 child_pos = slots[1].position;

    std::set<size_t> claimed;
    // Entry claims the most-behind slot (index 0, at X=-2)
    claim_nearest_slot(slots, claimed, child_pos, 0.75f, true, normal, travel);
    EXPECT_EQ(claimed.size(), 1u);

    // Second entry claim should get a different slot
    claim_nearest_slot(slots, claimed, child_pos, 0.75f, true, normal, travel);
    EXPECT_EQ(claimed.size(), 2u);
}
