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
// Test 1: Entry and exit are on opposite sides of the normal
// ---------------------------------------------------------------------------
TEST(CrossoverClaimingTest, EntryExitOppositeNormals) {
    Vec3 normal(0, 0, 1);
    Vec3 tangent(1, 0, 0);
    auto slots = make_slots(1, Vec3(0, 5, 0), tangent, normal, 1.0f);
    float radius = 0.75f;

    // as_entry=true: entry on +normal, exit on -normal
    {
        std::set<size_t> claimed;
        auto xover = claim_nearest_slot(slots, claimed, Vec3(0, 5, 0), radius, true);
        float entry_z = xover.entry.dot(normal);
        float exit_z = xover.exit.dot(normal);
        EXPECT_GT(entry_z, exit_z) << "Entry should be on +normal side, exit on -normal side";
    }

    // as_entry=false: entry on -normal, exit on +normal
    {
        std::set<size_t> claimed;
        auto xover = claim_nearest_slot(slots, claimed, Vec3(0, 5, 0), radius, false);
        float entry_z = xover.entry.dot(normal);
        float exit_z = xover.exit.dot(normal);
        EXPECT_LT(entry_z, exit_z) << "Exit should be on +normal side, entry on -normal side";
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

    std::set<size_t> claimed;
    auto xover = claim_nearest_slot(slots, claimed, center, radius, true);

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

    std::set<size_t> claimed;
    auto xover = claim_nearest_slot(slots, claimed, Vec3(0, 5, 0), 0.75f, true);

    EXPECT_NEAR(xover.exit_direction.x, tangent.x, 1e-5f);
    EXPECT_NEAR(xover.exit_direction.y, tangent.y, 1e-5f);
    EXPECT_NEAR(xover.exit_direction.z, tangent.z, 1e-5f);
}

// ---------------------------------------------------------------------------
// Test 4: Nearest slot is claimed
// ---------------------------------------------------------------------------
TEST(CrossoverClaimingTest, NearestSlotIsClaimed) {
    Vec3 tangent(1, 0, 0);
    Vec3 normal(0, 0, 1);
    // 4 slots spread along X
    auto slots = make_slots(4, Vec3(0, 5, 0), tangent, normal, 2.0f);

    // Child near the 3rd slot (index 2)
    Vec3 child_pos = slots[2].position + Vec3(0.1f, 0, 0);

    std::set<size_t> claimed;
    claim_nearest_slot(slots, claimed, child_pos, 0.75f, true);

    EXPECT_EQ(claimed.size(), 1u);
    EXPECT_TRUE(claimed.count(2)) << "Nearest slot (index 2) should be claimed";
}

// ---------------------------------------------------------------------------
// Test 5: Claimed slots are not reusable
// ---------------------------------------------------------------------------
TEST(CrossoverClaimingTest, ClaimedSlotsNotReusable) {
    Vec3 tangent(1, 0, 0);
    Vec3 normal(0, 0, 1);
    auto slots = make_slots(3, Vec3(0, 5, 0), tangent, normal, 2.0f);

    // Child near center slot (index 1)
    Vec3 child_pos = slots[1].position;

    std::set<size_t> claimed;
    claim_nearest_slot(slots, claimed, child_pos, 0.75f, true);
    EXPECT_TRUE(claimed.count(1));

    // Second claim with same position should get a different slot
    claim_nearest_slot(slots, claimed, child_pos, 0.75f, true);
    EXPECT_EQ(claimed.size(), 2u);
    // Slot 1 was taken, so the second nearest (0 or 2) should be claimed
    EXPECT_TRUE(claimed.count(0) || claimed.count(2))
        << "Second claim should pick a different slot";
}
