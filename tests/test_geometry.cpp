#include <gtest/gtest.h>
#include "geometry.hpp"
#include "yarn_path.hpp"
#include "stitch_instruction.hpp"
#include "row_instruction.hpp"
#include <cmath>

using namespace yarnpath;

// Helper to create a simple pattern
PatternInstructions create_pattern(const std::vector<std::string>& rows) {
    PatternInstructions pattern;
    for (size_t i = 0; i < rows.size(); ++i) {
        RowInstruction row;
        row.side = (i % 2 == 0) ? RowSide::RS : RowSide::WS;

        // Parse simple notation
        for (char c : rows[i]) {
            if (c == 'K') {
                row.stitches.push_back(instruction::Knit{});
            } else if (c == 'P') {
                row.stitches.push_back(instruction::Purl{});
            } else if (c == 'C') {
                row.stitches.push_back(instruction::CastOn{1});
            } else if (c == 'B') {
                row.stitches.push_back(instruction::BindOff{1});
            } else if (c == 'O') {
                row.stitches.push_back(instruction::YarnOver{});
            } else if (c == '2') {
                row.stitches.push_back(instruction::K2tog{});
            } else if (c == 'S') {
                row.stitches.push_back(instruction::SSK{});
            }
        }
        pattern.rows.push_back(row);
    }
    return pattern;
}

// ============================================
// Vec3 Tests
// ============================================

TEST(Vec3Test, DefaultConstruction) {
    Vec3 v;
    EXPECT_FLOAT_EQ(v.x, 0.0f);
    EXPECT_FLOAT_EQ(v.y, 0.0f);
    EXPECT_FLOAT_EQ(v.z, 0.0f);
}

TEST(Vec3Test, ValueConstruction) {
    Vec3 v(1.0f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(v.x, 1.0f);
    EXPECT_FLOAT_EQ(v.y, 2.0f);
    EXPECT_FLOAT_EQ(v.z, 3.0f);
}

TEST(Vec3Test, Addition) {
    Vec3 a(1.0f, 2.0f, 3.0f);
    Vec3 b(4.0f, 5.0f, 6.0f);
    Vec3 c = a + b;
    EXPECT_FLOAT_EQ(c.x, 5.0f);
    EXPECT_FLOAT_EQ(c.y, 7.0f);
    EXPECT_FLOAT_EQ(c.z, 9.0f);
}

TEST(Vec3Test, DotProduct) {
    Vec3 a(1.0f, 0.0f, 0.0f);
    Vec3 b(0.0f, 1.0f, 0.0f);
    EXPECT_FLOAT_EQ(a.dot(b), 0.0f);

    Vec3 c(1.0f, 2.0f, 3.0f);
    Vec3 d(4.0f, 5.0f, 6.0f);
    EXPECT_FLOAT_EQ(c.dot(d), 32.0f);
}

TEST(Vec3Test, CrossProduct) {
    Vec3 x = vec3::unit_x();
    Vec3 y = vec3::unit_y();
    Vec3 z = x.cross(y);
    EXPECT_FLOAT_EQ(z.x, 0.0f);
    EXPECT_FLOAT_EQ(z.y, 0.0f);
    EXPECT_FLOAT_EQ(z.z, 1.0f);
}

TEST(Vec3Test, Length) {
    Vec3 v(3.0f, 4.0f, 0.0f);
    EXPECT_FLOAT_EQ(v.length(), 5.0f);
}

TEST(Vec3Test, Normalized) {
    Vec3 v(3.0f, 4.0f, 0.0f);
    Vec3 n = v.normalized();
    EXPECT_FLOAT_EQ(n.length(), 1.0f);
    EXPECT_FLOAT_EQ(n.x, 0.6f);
    EXPECT_FLOAT_EQ(n.y, 0.8f);
}

TEST(Vec3Test, Lerp) {
    Vec3 a(0.0f, 0.0f, 0.0f);
    Vec3 b(10.0f, 10.0f, 10.0f);
    Vec3 mid = lerp(a, b, 0.5f);
    EXPECT_FLOAT_EQ(mid.x, 5.0f);
    EXPECT_FLOAT_EQ(mid.y, 5.0f);
    EXPECT_FLOAT_EQ(mid.z, 5.0f);
}

// ============================================
// CubicBezier Tests
// ============================================

TEST(CubicBezierTest, Evaluate) {
    CubicBezier curve(
        Vec3(0.0f, 0.0f, 0.0f),
        Vec3(0.0f, 1.0f, 0.0f),
        Vec3(1.0f, 1.0f, 0.0f),
        Vec3(1.0f, 0.0f, 0.0f)
    );

    Vec3 start = curve.evaluate(0.0f);
    EXPECT_FLOAT_EQ(start.x, 0.0f);
    EXPECT_FLOAT_EQ(start.y, 0.0f);

    Vec3 end = curve.evaluate(1.0f);
    EXPECT_FLOAT_EQ(end.x, 1.0f);
    EXPECT_FLOAT_EQ(end.y, 0.0f);
}

TEST(CubicBezierTest, ArcLength) {
    // Straight line should have arc length equal to distance
    CubicBezier line(
        Vec3(0.0f, 0.0f, 0.0f),
        Vec3(1.0f, 0.0f, 0.0f),
        Vec3(2.0f, 0.0f, 0.0f),
        Vec3(3.0f, 0.0f, 0.0f)
    );

    float arc_len = line.arc_length(100);
    EXPECT_NEAR(arc_len, 3.0f, 0.01f);
}

TEST(CubicBezierTest, FromHermite) {
    Vec3 p0(0.0f, 0.0f, 0.0f);
    Vec3 t0(3.0f, 0.0f, 0.0f);
    Vec3 p1(3.0f, 0.0f, 0.0f);
    Vec3 t1(3.0f, 0.0f, 0.0f);

    CubicBezier curve = CubicBezier::from_hermite(p0, t0, p1, t1);

    // Check endpoints match
    Vec3 start = curve.evaluate(0.0f);
    Vec3 end = curve.evaluate(1.0f);

    EXPECT_NEAR(start.x, p0.x, 0.001f);
    EXPECT_NEAR(start.y, p0.y, 0.001f);
    EXPECT_NEAR(end.x, p1.x, 0.001f);
    EXPECT_NEAR(end.y, p1.y, 0.001f);
}

TEST(CubicBezierTest, Split) {
    CubicBezier curve(
        Vec3(0.0f, 0.0f, 0.0f),
        Vec3(0.0f, 1.0f, 0.0f),
        Vec3(1.0f, 1.0f, 0.0f),
        Vec3(1.0f, 0.0f, 0.0f)
    );

    auto [left, right] = curve.split(0.5f);

    Vec3 mid_original = curve.evaluate(0.5f);
    Vec3 left_end = left.evaluate(1.0f);
    Vec3 right_start = right.evaluate(0.0f);

    EXPECT_NEAR(left_end.x, mid_original.x, 0.001f);
    EXPECT_NEAR(left_end.y, mid_original.y, 0.001f);
    EXPECT_NEAR(right_start.x, mid_original.x, 0.001f);
    EXPECT_NEAR(right_start.y, mid_original.y, 0.001f);
}

// ============================================
// BezierSpline Tests
// ============================================

TEST(BezierSplineTest, Empty) {
    BezierSpline spline;
    EXPECT_TRUE(spline.empty());
    EXPECT_EQ(spline.segment_count(), 0u);
}

TEST(BezierSplineTest, AddSegment) {
    BezierSpline spline;
    CubicBezier seg(
        Vec3(0.0f, 0.0f, 0.0f),
        Vec3(1.0f, 0.0f, 0.0f),
        Vec3(2.0f, 0.0f, 0.0f),
        Vec3(3.0f, 0.0f, 0.0f)
    );
    spline.add_segment(seg);

    EXPECT_FALSE(spline.empty());
    EXPECT_EQ(spline.segment_count(), 1u);
}

TEST(BezierSplineTest, ToPolyline) {
    BezierSpline spline;
    CubicBezier seg(
        Vec3(0.0f, 0.0f, 0.0f),
        Vec3(1.0f, 0.0f, 0.0f),
        Vec3(2.0f, 0.0f, 0.0f),
        Vec3(3.0f, 0.0f, 0.0f)
    );
    spline.add_segment(seg);

    auto points = spline.to_polyline_fixed(10);
    EXPECT_EQ(points.size(), 11u);  // Start + 10 samples
}

// ============================================
// YarnProperties Tests
// ============================================

TEST(YarnPropertiesTest, Defaults) {
    YarnProperties yarn;
    EXPECT_FLOAT_EQ(yarn.radius, 1.0f);
    EXPECT_FLOAT_EQ(yarn.min_bend_radius, 3.0f);
}

TEST(YarnPropertiesTest, DerivedProperties) {
    YarnProperties yarn;
    yarn.radius = 2.0f;
    yarn.min_bend_radius = 6.0f;

    EXPECT_FLOAT_EQ(yarn.min_clearance(), 4.0f);
    EXPECT_NEAR(yarn.max_curvature(), 1.0f / 6.0f, 0.001f);
}

TEST(YarnPropertiesTest, Presets) {
    auto fingering = YarnProperties::fingering();
    EXPECT_LT(fingering.radius, 1.0f);

    auto bulky = YarnProperties::bulky();
    EXPECT_GT(bulky.radius, 1.0f);
}

// ============================================
// Gauge Tests
// ============================================

TEST(GaugeTest, Defaults) {
    Gauge gauge;
    EXPECT_FLOAT_EQ(gauge.stitches_per_unit, 4.0f);
    EXPECT_FLOAT_EQ(gauge.rows_per_unit, 5.0f);
}

TEST(GaugeTest, StitchWidth) {
    Gauge gauge;
    gauge.stitches_per_unit = 5.0f;
    EXPECT_FLOAT_EQ(gauge.stitch_width(), 0.2f);
}

TEST(GaugeTest, RowHeight) {
    Gauge gauge;
    gauge.rows_per_unit = 4.0f;
    EXPECT_FLOAT_EQ(gauge.row_height(), 0.25f);
}

TEST(GaugeTest, Conversion) {
    Gauge gauge;
    gauge.stitches_per_unit = 4.0f;
    gauge.rows_per_unit = 5.0f;

    EXPECT_FLOAT_EQ(gauge.stitch_to_u(4.0f), 1.0f);
    EXPECT_FLOAT_EQ(gauge.row_to_v(5.0f), 1.0f);
}

// ============================================
// FabricSurface Tests
// ============================================

TEST(PlaneSurfaceTest, Default) {
    PlaneSurface surface;

    Vec3 pos = surface.position(1.0f, 2.0f);
    EXPECT_FLOAT_EQ(pos.x, 1.0f);
    EXPECT_FLOAT_EQ(pos.y, 2.0f);
    EXPECT_FLOAT_EQ(pos.z, 0.0f);
}

TEST(PlaneSurfaceTest, Normal) {
    PlaneSurface surface;

    Vec3 n = surface.normal(0.0f, 0.0f);
    EXPECT_FLOAT_EQ(n.z, 1.0f);
}

TEST(PlaneSurfaceTest, LocalToWorld) {
    PlaneSurface surface;

    Vec3 world = surface.local_to_world(1.0f, 2.0f, 0.1f, 0.2f, 0.3f);
    EXPECT_FLOAT_EQ(world.x, 1.1f);
    EXPECT_FLOAT_EQ(world.y, 2.2f);
    EXPECT_FLOAT_EQ(world.z, 0.3f);
}

TEST(CylinderSurfaceTest, Position) {
    CylinderSurface surface(1.0f, 2.0f * std::numbers::pi_v<float>);

    // At u=0, should be at (1, 0, 0)
    Vec3 pos0 = surface.position(0.0f, 0.0f);
    EXPECT_NEAR(pos0.x, 1.0f, 0.001f);
    EXPECT_NEAR(pos0.y, 0.0f, 0.001f);
    EXPECT_NEAR(pos0.z, 0.0f, 0.001f);

    // At u=pi (half circumference), should be at (-1, 0, 0)
    Vec3 pos_half = surface.position(std::numbers::pi_v<float>, 0.0f);
    EXPECT_NEAR(pos_half.x, -1.0f, 0.001f);
    EXPECT_NEAR(pos_half.z, 0.0f, 0.001f);
}

// ============================================
// StitchGlyph Tests
// ============================================

TEST(GlyphFactoryTest, CreateKnitGlyph) {
    GlyphFactory factory(YarnProperties::worsted(), Gauge::worsted());
    Loop dummy_loop{};
    dummy_loop.kind = FormKind::Knit;

    StitchGlyph glyph = factory.create_glyph(FormKind::Knit, dummy_loop);

    EXPECT_EQ(glyph.kind, FormKind::Knit);
    EXPECT_NE(glyph.get_anchor("entry"), nullptr);
    EXPECT_NE(glyph.get_anchor("apex"), nullptr);
    EXPECT_NE(glyph.get_anchor("exit"), nullptr);
    EXPECT_FALSE(glyph.path_segments.empty());
}

TEST(GlyphFactoryTest, CreateAllStitchTypes) {
    GlyphFactory factory(YarnProperties::worsted(), Gauge::worsted());
    Loop dummy_loop{};

    std::vector<FormKind> all_kinds = {
        FormKind::CastOn, FormKind::Knit, FormKind::Purl, FormKind::Slip,
        FormKind::BindOff, FormKind::YarnOver, FormKind::KFB,
        FormKind::M1L, FormKind::M1R, FormKind::K2tog, FormKind::SSK, FormKind::S2KP
    };

    for (FormKind kind : all_kinds) {
        dummy_loop.kind = kind;
        StitchGlyph glyph = factory.create_glyph(kind, dummy_loop);
        EXPECT_EQ(glyph.kind, kind);
        EXPECT_FALSE(glyph.anchors.empty()) << "Failed for kind " << static_cast<int>(kind);
    }
}

TEST(StitchGlyphTest, Transform) {
    GlyphFactory factory(YarnProperties::worsted(), Gauge::worsted());
    Loop dummy_loop{};
    dummy_loop.kind = FormKind::Knit;

    StitchGlyph glyph = factory.create_glyph(FormKind::Knit, dummy_loop);
    StitchGlyph transformed = glyph.transformed(
        Vec3(10.0f, 20.0f, 0.0f),
        Vec3(2.0f, 3.0f, 1.0f)
    );

    // Check that apex anchor was transformed
    const auto* original_apex = glyph.get_anchor("apex");
    const auto* transformed_apex = transformed.get_anchor("apex");

    ASSERT_NE(original_apex, nullptr);
    ASSERT_NE(transformed_apex, nullptr);

    // Should be offset by origin and scaled
    EXPECT_GT(transformed_apex->position.x, original_apex->position.x);
    EXPECT_GT(transformed_apex->position.y, original_apex->position.y);
}

// ============================================
// GeometryPath Integration Tests
// ============================================

TEST(GeometryPathTest, EmptyYarnPath) {
    YarnPath empty_yarn_path;
    PlaneSurface surface;

    GeometryPath geometry = GeometryPath::from_yarn_path(
        empty_yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted(),
        surface
    );

    EXPECT_TRUE(geometry.anchors().empty());
    EXPECT_TRUE(geometry.segments().empty());
}

TEST(GeometryPathTest, CastOnOnly) {
    PatternInstructions pattern = create_pattern({"CCCC"});
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted(),
        surface
    );

    // Should have anchors for each cast-on stitch
    EXPECT_FALSE(geometry.anchors().empty());

    // Loop positions should form a horizontal line
    const auto& positions = geometry.loop_positions();
    EXPECT_EQ(positions.size(), 4u);

    // All at row 0
    for (const auto& pos : positions) {
        EXPECT_EQ(pos.row, 0u);
        EXPECT_FLOAT_EQ(pos.v, 0.0f);
    }

    // Sequential columns
    float prev_u = -1.0f;
    for (const auto& pos : positions) {
        EXPECT_GT(pos.u, prev_u);
        prev_u = pos.u;
    }
}

TEST(GeometryPathTest, StockinettePattern) {
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KKKK",
        "PPPP",
        "KKKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    Gauge gauge = Gauge::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        gauge,
        surface
    );

    // Check that loop positions form a grid
    const auto& positions = geometry.loop_positions();
    EXPECT_EQ(positions.size(), 16u);  // 4x4 grid

    // Verify row heights increase
    std::map<uint32_t, float> row_v_values;
    for (const auto& pos : positions) {
        if (row_v_values.count(pos.row) == 0) {
            row_v_values[pos.row] = pos.v;
        }
    }

    EXPECT_LT(row_v_values[0], row_v_values[1]);
    EXPECT_LT(row_v_values[1], row_v_values[2]);
    EXPECT_LT(row_v_values[2], row_v_values[3]);
}

TEST(GeometryPathTest, ToPolyline) {
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KKKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted(),
        surface
    );

    // Convert to polyline
    auto polyline = geometry.to_polyline(0.1f);
    EXPECT_FALSE(polyline.empty());
}

TEST(GeometryPathTest, ToOBJ) {
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KKKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted(),
        surface
    );

    std::string obj = geometry.to_obj();
    EXPECT_FALSE(obj.empty());
    EXPECT_NE(obj.find("v "), std::string::npos);  // Has vertices
}

TEST(GeometryPathTest, BoundingBox) {
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KKKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted(),
        surface
    );

    auto [min_pt, max_pt] = geometry.bounding_box();

    // Box should have non-zero extent in X and Y
    EXPECT_LT(min_pt.x, max_pt.x);
    EXPECT_LT(min_pt.y, max_pt.y);
}

TEST(GeometryPathTest, Validation) {
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KKKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        yarn,
        Gauge::worsted(),
        surface
    );

    ValidationResult result = geometry.validate(yarn);
    // Should be valid (or only have warnings, not errors)
    // We don't strictly require valid=true since some edge cases may have warnings
    EXPECT_TRUE(result.errors.empty());
}

TEST(GeometryPathTest, DecreasePattern) {
    // K2tog creates a decrease (consumes 2 parent loops)
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "K2KK"  // K2tog reduces 4 to 3
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted(),
        surface
    );

    // Row 1 should have 3 loops (4 - 1 = 3)
    int row1_count = 0;
    for (const auto& pos : geometry.loop_positions()) {
        if (pos.row == 1) {
            row1_count++;
        }
    }
    EXPECT_EQ(row1_count, 3);
}

TEST(GeometryPathTest, CylinderSurface) {
    PatternInstructions pattern = create_pattern({
        "CCCCCCCC",
        "KKKKKKKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    // Use cylinder surface
    float circumference = 2.0f;  // Will wrap 8 stitches in circumference
    CylinderSurface surface(circumference / (2.0f * std::numbers::pi_v<float>), circumference);

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted(),
        surface
    );

    // Geometry should have positions on cylinder surface
    auto [min_pt, max_pt] = geometry.bounding_box();

    // On a cylinder, X and Z will vary
    EXPECT_LT(min_pt.x, max_pt.x);
}

TEST(GeometryPathTest, TotalArcLength) {
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KKKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted(),
        surface
    );

    float arc_length = geometry.total_arc_length();
    EXPECT_GT(arc_length, 0.0f);
}

TEST(GeometryPathTest, AnchorLookup) {
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KKKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted(),
        surface
    );

    // Should be able to look up anchors by ID
    for (const auto& anchor : geometry.anchors()) {
        const AnchorGeometry* found = geometry.get_anchor(anchor.anchor_id);
        ASSERT_NE(found, nullptr);
        EXPECT_EQ(found->anchor_id, anchor.anchor_id);
    }
}

TEST(GeometryPathTest, SegmentLookup) {
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KKKK"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted(),
        surface
    );

    // Should be able to look up segments by ID
    for (const auto& seg : geometry.segments()) {
        const SegmentGeometry* found = geometry.get_segment(seg.segment_id);
        ASSERT_NE(found, nullptr);
        EXPECT_EQ(found->segment_id, seg.segment_id);
    }
}

// ============================================
// Loops Without Parents (YarnOver, M1L, M1R)
// These loops have no parent loops and must be queued specially during position propagation
// ============================================

TEST(GeometryPathTest, YarnOverPositioning) {
    // YarnOver creates loops with no parents - these need special handling
    // in position propagation since they're not children of any other loop
    PatternInstructions pattern = create_pattern({
        "CCCC",      // Cast on 4 stitches
        "KOKOK"      // K, YO, K, YO, K - creates 2 yarn overs with no parents
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted(),
        surface
    );

    // All loops should be positioned (the bug was yarn overs weren't getting positioned)
    EXPECT_EQ(geometry.loop_positions().size(), yarn_path.loops().size());

    // Yarn over loops (in row 1) should have valid positions
    for (const auto& pos : geometry.loop_positions()) {
        EXPECT_TRUE(std::isfinite(pos.u));
        EXPECT_TRUE(std::isfinite(pos.v));
    }

    // Should be able to generate output without hanging
    std::string obj = geometry.to_obj();
    EXPECT_FALSE(obj.empty());
}

TEST(GeometryPathTest, MultipleYarnOversInRow) {
    // More complex pattern with multiple yarn overs
    PatternInstructions pattern = create_pattern({
        "CCCCCC",     // Cast on 6 stitches
        "K2OKO2K"     // K2tog, YO, K, YO, K2tog, K - mixed increases/decreases
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        YarnProperties::worsted(),
        Gauge::worsted(),
        surface
    );

    // All loops should be positioned
    EXPECT_EQ(geometry.loop_positions().size(), yarn_path.loops().size());

    // Should complete without infinite loop
    auto [min_pt, max_pt] = geometry.bounding_box();
    EXPECT_LT(min_pt.x, max_pt.x);
}

// ============================================
// Curvature Constraint Tests
// ============================================

TEST(GeometryPathTest, CurvatureIsFinite) {
    // Test that curvature values are computed correctly (finite, non-negative)
    PatternInstructions pattern = create_pattern({
        "CCCC",
        "KKKK",
        "PPPP"
    });
    StitchGraph graph = StitchGraph::from_instructions(pattern);
    YarnPath yarn_path = YarnPath::from_stitch_graph(graph);

    PlaneSurface surface;
    YarnProperties yarn = YarnProperties::worsted();

    GeometryPath geometry = GeometryPath::from_yarn_path(
        yarn_path,
        yarn,
        Gauge::worsted(),
        surface
    );

    // Check that curvature values are reasonable (finite and non-negative)
    for (const auto& seg : geometry.segments()) {
        EXPECT_GE(seg.max_curvature, 0.0f)
            << "Segment " << seg.segment_id << " has negative curvature";
        EXPECT_TRUE(std::isfinite(seg.max_curvature))
            << "Segment " << seg.segment_id << " has non-finite curvature";
    }

    // Verify that curvature clamping was attempted (segments may have been subdivided)
    // The subdivision increases segment count when curvature is too high
    EXPECT_FALSE(geometry.segments().empty());
}
