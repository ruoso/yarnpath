#include <gtest/gtest.h>
#include "geometry.hpp"
#include "yarn_path.hpp"
#include "stitch_instruction.hpp"
#include "row_instruction.hpp"
#include "physical_loop.hpp"
#include <cmath>

using namespace yarnpath;

// Helper to create a simple pattern
PatternInstructions create_simple_pattern(const std::vector<std::string>& rows) {
    PatternInstructions pattern;
    for (size_t i = 0; i < rows.size(); ++i) {
        RowInstruction row;
        row.side = (i % 2 == 0) ? RowSide::RS : RowSide::WS;

        for (char c : rows[i]) {
            if (c == 'K') {
                row.stitches.push_back(instruction::Knit{});
            } else if (c == 'P') {
                row.stitches.push_back(instruction::Purl{});
            } else if (c == 'C') {
                row.stitches.push_back(instruction::CastOn{1});
            }
        }
        pattern.rows.push_back(row);
    }
    return pattern;
}

// ============================================
// Cast-On Loop Shape Tests
// These tests verify that a single cast-on loop has proper 3D shape
// ============================================

TEST(CastOnLoopShapeTest, SingleCastOnHas3DDepth) {
    // A single cast-on loop should curve in 3D:
    // - Entry at front (negative Z)
    // - Apex at back (positive Z)
    // - Exit at front (negative Z)
    PatternInstructions pattern = create_simple_pattern({"C"});
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    // Get the polyline with fine sampling
    auto polyline = geometry.to_polyline_fixed(20);

    ASSERT_GT(polyline.size(), 5u) << "Should have multiple points in the loop";

    // Find min and max Z values
    float min_z = polyline[0].z;
    float max_z = polyline[0].z;
    for (const auto& pt : polyline) {
        min_z = std::min(min_z, pt.z);
        max_z = std::max(max_z, pt.z);
    }

    float z_range = max_z - min_z;

    std::cout << "\n=== Single Cast-On Loop Z Range ===" << std::endl;
    std::cout << "Min Z: " << min_z << std::endl;
    std::cout << "Max Z: " << max_z << std::endl;
    std::cout << "Z Range: " << z_range << std::endl;

    // The loop should have significant Z depth (not flat)
    // Expected depth is approximately opening_diameter * 0.5
    auto loop_dim = LoopDimensions::calculate(yarn, gauge);
    float expected_depth = loop_dim.opening_diameter * 0.5f;

    std::cout << "Expected depth (opening_diameter * 0.5): " << expected_depth << std::endl;

    EXPECT_GT(z_range, expected_depth * 0.5f)
        << "Cast-on loop should have Z depth of at least " << (expected_depth * 0.5f)
        << " but got " << z_range;
}

TEST(CastOnLoopShapeTest, CastOnApexIsAtTop) {
    // The apex (highest Y point) should be at Z ≈ 0 (top of the needle)
    // With a full wrap-around, the topmost point of the yarn is at the top
    // of the cylinder, not at the back.
    PatternInstructions pattern = create_simple_pattern({"C"});
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    auto polyline = geometry.to_polyline_fixed(20);

    // Find the point with maximum Y (the apex)
    Vec3 apex_point = polyline[0];
    for (const auto& pt : polyline) {
        if (pt.y > apex_point.y) {
            apex_point = pt;
        }
    }

    std::cout << "\n=== Cast-On Apex Position ===" << std::endl;
    std::cout << "Apex at: (" << apex_point.x << ", " << apex_point.y << ", " << apex_point.z << ")" << std::endl;

    // Apex should be at the top of the needle (Z ≈ 0)
    // Allow some tolerance since the apex might be slightly off-center
    float wrap_radius = gauge.needle_diameter / 2.0f + yarn.radius;
    EXPECT_NEAR(apex_point.z, 0.0f, wrap_radius * 0.3f)
        << "Apex should be near Z = 0 (top of needle), but Z = " << apex_point.z;
}

TEST(CastOnLoopShapeTest, CastOnEntryExitAtFront) {
    // Entry and exit points (first and last) should be at negative Z (front)
    PatternInstructions pattern = create_simple_pattern({"C"});
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    auto polyline = geometry.to_polyline_fixed(20);

    ASSERT_GE(polyline.size(), 2u);

    Vec3 entry = polyline.front();
    Vec3 exit = polyline.back();

    std::cout << "\n=== Cast-On Entry/Exit ===" << std::endl;
    std::cout << "Entry: (" << entry.x << ", " << entry.y << ", " << entry.z << ")" << std::endl;
    std::cout << "Exit: (" << exit.x << ", " << exit.y << ", " << exit.z << ")" << std::endl;

    // Entry and exit should be at front (negative Z or close to 0)
    EXPECT_LT(entry.z, 0.1f)
        << "Entry should be at front (Z < 0), but Z = " << entry.z;
    EXPECT_LT(exit.z, 0.1f)
        << "Exit should be at front (Z < 0), but Z = " << exit.z;
}

TEST(CastOnLoopShapeTest, MultipleCastOnLoopsEachHave3DShape) {
    // Multiple cast-on stitches should each form their own 3D loop
    PatternInstructions pattern = create_simple_pattern({"CCC"});
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    auto polyline = geometry.to_polyline_fixed(20);

    // Count how many times Z goes from front to back to front (loop cycles)
    int front_to_back_transitions = 0;
    bool was_at_front = polyline[0].z < 0.0f;

    for (size_t i = 1; i < polyline.size(); ++i) {
        bool is_at_front = polyline[i].z < 0.0f;
        if (was_at_front && !is_at_front) {
            front_to_back_transitions++;
        }
        was_at_front = is_at_front;
    }

    std::cout << "\n=== Multiple Cast-On Loops ===" << std::endl;
    std::cout << "Front-to-back transitions: " << front_to_back_transitions << std::endl;
    std::cout << "Expected (one per loop): 3" << std::endl;

    // Each cast-on loop should go front->back, so we expect 3 transitions for 3 loops
    EXPECT_GE(front_to_back_transitions, 3)
        << "Expected at least 3 front-to-back transitions for 3 cast-on loops";
}

TEST(CastOnLoopShapeTest, LoopFormsContinuousCurve) {
    // The loop should be a smooth continuous curve, not disjointed
    PatternInstructions pattern = create_simple_pattern({"C"});
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    auto polyline = geometry.to_polyline_fixed(50);

    // Check that consecutive points are not too far apart
    float max_gap = 0.0f;
    for (size_t i = 1; i < polyline.size(); ++i) {
        Vec3 diff = polyline[i] - polyline[i-1];
        float gap = diff.length();
        max_gap = std::max(max_gap, gap);
    }

    auto loop_dim = LoopDimensions::calculate(yarn, gauge);

    std::cout << "\n=== Loop Continuity ===" << std::endl;
    std::cout << "Max gap between points: " << max_gap << std::endl;
    std::cout << "Loop height: " << loop_dim.loop_height << std::endl;

    // Max gap should be much smaller than the loop dimensions
    // With 50 samples, gaps should be small fractions of loop size
    EXPECT_LT(max_gap, loop_dim.loop_height * 0.2f)
        << "Loop should be continuous - max gap " << max_gap
        << " is too large compared to loop height " << loop_dim.loop_height;
}

TEST(CastOnLoopShapeTest, NeedleCylinderFitsInsideLoop) {
    // The cast-on loop should wrap around the needle.
    // A cylinder representing the needle should fit inside the loop
    // without colliding with the yarn path.
    //
    // The needle cylinder is:
    // - Centered at the loop center
    // - Axis along the X direction (horizontal, perpendicular to fabric)
    // - Radius = needle_diameter / 2
    //
    // The yarn path (with thickness yarn.radius) should not intersect this cylinder.

    PatternInstructions pattern = create_simple_pattern({"C"});
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    // Get loop position to find the center
    ASSERT_FALSE(geometry.loop_positions().empty());
    const auto& loop_pos = geometry.loop_positions()[0];

    // Calculate expected needle position
    // The needle is at the center of the loop opening
    auto loop_dim = LoopDimensions::calculate(yarn, gauge);

    // Needle cylinder parameters:
    // - Center at (loop_pos.u + loop_width/2, loop_pos.v + loop_height/2, 0)
    // - Axis along X
    // - Radius = needle_diameter / 2
    float needle_radius = gauge.needle_diameter / 2.0f;
    Vec3 needle_center(
        loop_pos.u + loop_dim.loop_width / 2.0f,
        loop_pos.v + loop_dim.loop_height / 2.0f,
        0.0f
    );

    std::cout << "\n=== Needle Cylinder Fit Test ===" << std::endl;
    std::cout << "Needle diameter: " << gauge.needle_diameter << std::endl;
    std::cout << "Needle radius: " << needle_radius << std::endl;
    std::cout << "Yarn radius: " << yarn.radius << std::endl;
    std::cout << "Needle center: (" << needle_center.x << ", " << needle_center.y << ", " << needle_center.z << ")" << std::endl;

    // Get fine-grained polyline
    auto polyline = geometry.to_polyline_fixed(100);

    // Check each point on the yarn path
    // Distance from point to cylinder axis (which is along X through needle_center)
    // For a point P and axis through C along X:
    // distance = sqrt((P.y - C.y)^2 + (P.z - C.z)^2)
    //
    // The yarn has thickness yarn.radius, so the yarn surface is at:
    // distance_to_axis - yarn.radius from the needle surface
    //
    // For no collision: distance_to_axis > needle_radius + yarn.radius

    float min_clearance = std::numeric_limits<float>::max();
    Vec3 closest_point;

    for (const auto& pt : polyline) {
        // Distance from yarn centerline to needle axis (in YZ plane)
        float dy = pt.y - needle_center.y;
        float dz = pt.z - needle_center.z;
        float dist_to_axis = std::sqrt(dy * dy + dz * dz);

        // Clearance = distance from yarn surface to needle surface
        float clearance = dist_to_axis - needle_radius - yarn.radius;

        if (clearance < min_clearance) {
            min_clearance = clearance;
            closest_point = pt;
        }
    }

    std::cout << "Minimum clearance: " << min_clearance << std::endl;
    std::cout << "Closest point: (" << closest_point.x << ", " << closest_point.y << ", " << closest_point.z << ")" << std::endl;

    // The yarn should wrap AROUND the needle, not through it
    // Clearance should be positive (no collision) or at most slightly negative
    // (yarn touching needle is expected during cast-on)
    EXPECT_GT(min_clearance, -yarn.radius * 0.5f)
        << "Yarn should wrap around needle, not pass through it. "
        << "Clearance " << min_clearance << " indicates collision.";

    // Additionally, the loop should actually go around the needle
    // Check that we have points on both sides of the needle (both +Z and -Z)
    bool has_front = false;  // negative Z
    bool has_back = false;   // positive Z

    for (const auto& pt : polyline) {
        if (pt.z < needle_center.z - needle_radius * 0.5f) has_front = true;
        if (pt.z > needle_center.z + needle_radius * 0.5f) has_back = true;
    }

    EXPECT_TRUE(has_front) << "Loop should have points at front of needle (negative Z)";
    EXPECT_TRUE(has_back) << "Loop should have points at back of needle (positive Z)";

    std::cout << "Has points at front: " << (has_front ? "yes" : "no") << std::endl;
    std::cout << "Has points at back: " << (has_back ? "yes" : "no") << std::endl;
}

TEST(CastOnLoopShapeTest, LoopWrapsFullyAroundNeedle) {
    // The cast-on loop should wrap fully around the needle cylinder.
    // We check this by computing the angle of each yarn point relative to the
    // needle axis (in the YZ plane) and verifying we have coverage in all
    // four quadrants: top (+Y), bottom (-Y), front (-Z), back (+Z).
    //
    // A proper wrap-around should cover at least 270 degrees of the cylinder,
    // meaning the yarn goes around most of the needle circumference.

    PatternInstructions pattern = create_simple_pattern({"C"});
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    ASSERT_FALSE(geometry.loop_positions().empty());
    const auto& loop_pos = geometry.loop_positions()[0];
    auto loop_dim = LoopDimensions::calculate(yarn, gauge);

    // Needle center in the YZ plane
    float needle_center_y = loop_pos.v + loop_dim.loop_height / 2.0f;
    float needle_center_z = 0.0f;

    auto polyline = geometry.to_polyline_fixed(100);

    std::cout << "\n=== Full Wrap-Around Test ===" << std::endl;
    std::cout << "Needle center (Y,Z): (" << needle_center_y << ", " << needle_center_z << ")" << std::endl;

    // Track which quadrants we have points in
    // Quadrants defined by angle from +Y axis (like a clock):
    //   Top:    -45° to +45°    (around +Y)
    //   Back:   +45° to +135°   (around +Z)
    //   Bottom: +135° to -135°  (around -Y, i.e., 135° to 225°)
    //   Front:  -135° to -45°   (around -Z)
    bool has_top = false;     // +Y region
    bool has_bottom = false;  // -Y region
    bool has_front = false;   // -Z region
    bool has_back = false;    // +Z region

    float min_angle = 180.0f;
    float max_angle = -180.0f;

    constexpr float PI = 3.14159265f;
    constexpr float RAD_TO_DEG = 180.0f / PI;

    for (const auto& pt : polyline) {
        float dy = pt.y - needle_center_y;
        float dz = pt.z - needle_center_z;

        // Angle in degrees, measured from +Y axis, positive clockwise toward +Z
        // atan2(dz, dy) gives angle from +Y axis
        float angle = std::atan2(dz, dy) * RAD_TO_DEG;

        min_angle = std::min(min_angle, angle);
        max_angle = std::max(max_angle, angle);

        // Check quadrants
        // Top: angle near 0° (between -45° and +45°)
        if (angle > -45.0f && angle < 45.0f) has_top = true;
        // Back: angle near 90° (between 45° and 135°)
        if (angle > 45.0f && angle < 135.0f) has_back = true;
        // Bottom: angle near ±180° (less than -135° or greater than 135°)
        if (angle < -135.0f || angle > 135.0f) has_bottom = true;
        // Front: angle near -90° (between -135° and -45°)
        if (angle > -135.0f && angle < -45.0f) has_front = true;
    }

    float angular_span = max_angle - min_angle;

    std::cout << "Angular range: " << min_angle << "° to " << max_angle << "°" << std::endl;
    std::cout << "Angular span: " << angular_span << "°" << std::endl;
    std::cout << "Quadrant coverage:" << std::endl;
    std::cout << "  Top (+Y):    " << (has_top ? "YES" : "NO") << std::endl;
    std::cout << "  Back (+Z):   " << (has_back ? "YES" : "NO") << std::endl;
    std::cout << "  Bottom (-Y): " << (has_bottom ? "YES" : "NO") << std::endl;
    std::cout << "  Front (-Z):  " << (has_front ? "YES" : "NO") << std::endl;

    // A cast-on loop should wrap around the needle, covering all four quadrants
    EXPECT_TRUE(has_top) << "Loop should have points in the top quadrant (+Y)";
    EXPECT_TRUE(has_back) << "Loop should have points in the back quadrant (+Z)";
    EXPECT_TRUE(has_bottom) << "Loop should have points in the bottom quadrant (-Y)";
    EXPECT_TRUE(has_front) << "Loop should have points in the front quadrant (-Z)";

    // The angular span should be at least 270 degrees for a proper wrap-around
    // (entry and exit are at the front, so we don't get a full 360°)
    EXPECT_GE(angular_span, 270.0f)
        << "Loop should wrap around at least 270° of the needle, but only spans "
        << angular_span << "°";
}

TEST(CastOnLoopShapeTest, DebugStockinetteLoopPositions) {
    // Debug test to understand loop positioning in a multi-row pattern
    PatternInstructions pattern = create_simple_pattern({"CCCC", "KKKK", "KKKK", "KKKK"});
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    std::cout << "\n=== Debug Stockinette Loop Positions ===" << std::endl;
    std::cout << "Yarn path loops: " << yarn_path.loops().size() << std::endl;

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path, yarn, gauge, surface
    );

    std::cout << "Loop positions: " << geometry.loop_positions().size() << std::endl;

    // Group loops by their v position (with tolerance for same "row")
    std::set<float> unique_v_values;
    for (const auto& pos : geometry.loop_positions()) {
        std::cout << "Loop " << pos.loop_id << ": u=" << pos.u << " v=" << pos.v << std::endl;
        unique_v_values.insert(pos.v);
    }

    std::cout << "\nUnique v values: " << unique_v_values.size() << std::endl;
    for (float v : unique_v_values) {
        std::cout << "  v=" << v << std::endl;
    }

    // We expect 16 loops (4 per row, 4 rows)
    EXPECT_EQ(geometry.loop_positions().size(), 16u);

    // We expect positions to increase in v (loops stack vertically)
    // Cast-on should be at v=0, subsequent rows should have increasing v
    float prev_min_v = -1.0f;
    for (float v : unique_v_values) {
        EXPECT_GT(v, prev_min_v) << "Loop v positions should increase";
        prev_min_v = v;
    }
}
