# Fixing Test Issues — Analysis and Plan

## Overview

30 tests fail across two layers: **surface/yarn-path** (3 tests) and **geometry builder** (27 tests). This document captures the root cause analysis, reasoning about test correctness, and the incremental plan to fix them.

## Orientation Model Correctness

### How `orientation_instruction_to_fabric()` works

The function in `yarn_path/yarn_path.cpp:126-130` converts instruction-level orientation to fabric-side orientation:

| Instruction | Side | Raw Orientation | After Flip | Fabric Appearance (RS) |
|------------|------|-----------------|------------|----------------------|
| RS Knit    | RS   | Front           | Front      | Knit-face (V)        |
| WS Knit    | WS   | Front           | **Back**   | Purl-face (bump)     |
| RS Purl    | RS   | Back            | Back       | Purl-face (bump)     |
| WS Purl    | WS   | Back            | **Front**  | Knit-face (V)        |

This flip is **physically correct**: it reflects what each stitch looks like from the RS of the fabric.

### Consequences for yarn length and Z position

- `LoopOrientation::Back` → 0.88× yarn length factor, negative z_bulge
- `LoopOrientation::Front` → 1.0× yarn length, positive z_bulge

This means:
- **Stockinette** (RS Knit + WS Purl) → all stitches are Front → uniform yarn length, uniform Z → flat fabric ✓
- **Garter stitch** (all Knit flat) → alternating Front/Back → alternating Z → wavy fabric ✓
- **Ribbing** (K/P in same row) → alternating Front/Back within row → depth alternation ✓

## Test Pattern Bugs (5 tests)

### SurfaceYarnLength tests (3 tests)

**Tests**: `KnitVsPurlMass`, `StiffnessProportionalToYarnDensity`, `OrientationAffectsZPosition`

**Pattern used**:
- Row 1 (RS): Knit, Knit → Front orientation
- Row 2 (WS): Purl, Purl → Front orientation (after flip)

**Problem**: Both rows produce Front-oriented stitches (this is stockinette — all stitches face the same way). The tests expect them to differ (0.88 mass ratio, opposite Z signs, different stiffness), but physically they're identical.

**Fix**: Change `row2.side = RowSide::WS` to `row2.side = RowSide::RS`. RS Purl → Back orientation → 0.88 factor, negative Z. Now the tests compare genuine Front vs Back stitches.

### Depth mirroring test

**Test**: `StockinetteKnitAndPurlRowsShouldBeNearMirrorInDepth`

**Pattern**: `{"CCCC", "KKKK", "PPPP", "KKKK"}` with `create_pattern` (alternating RS/WS)
- Row 1 (WS): KKKK → WS Knit → Back
- Row 2 (RS): PPPP → RS Purl → Back
- Row 3 (WS): KKKK → WS Knit → Back

**Problem**: ALL non-cast rows are Back orientation. The test expects row 1 and row 2 to have opposite Z depth, but they have the same orientation. Additionally, the test name says "Stockinette" but:
- Real stockinette (RS K + WS P) produces all-Front stitches — there IS no depth alternation in stockinette
- The depth alternation phenomenon occurs in **garter stitch** (all K when flat knitting), where RS K = Front and WS K = Back

**Fix**: Change pattern to garter stitch `{"CCCC", "KKKK", "KKKK", "KKKK"}`:
- Row 1 (WS K) = Back (negative Z)
- Row 2 (RS K) = Front (positive Z)
- Opposite Z → sum ≈ 0 → near-mirror ✓

Rename test to reflect garter stitch.

### Ribbing alternation test

**Test**: `RibbingShouldHaveStrongFrontBackAlternationBetweenWales`

**Pattern**: `{"CCCCCC", "KPKPKP"}` with `create_pattern`
- Row 1 is WS → WS K = Back (negative Z), WS P = Front (positive Z)

**Problem**: The K/P contrast within the row IS correct (they have opposite orientations). But the test expects `knit_z - purl_z > 3.0 * radius`, which assumes knit Z > purl Z. On a WS row, it's the opposite: WS K has Back/negative Z, WS P has Front/positive Z, so `knit_z - purl_z < 0`.

**Fix**: Change to `std::abs(knit_z - purl_z) > 3.0f * radius` to test for alternation magnitude regardless of sign.

## Geometry Code Issues (25 tests)

### Issue 1: Curvature Violations (3 tests)

**Tests**: `StockinettePatternStrictCurvatureCompliance`, `RibbingMixedPatternStrictCurvatureCompliance`, `FinerYarnStricterCurvatureCompliance`

**Root cause**: `math/curvature_utils.cpp` builds Hermite curves with a 2× safety factor on tangent magnitudes and `max_splits=3` (up to 8 sub-segments). For tight S-curves, this isn't enough — interior curvature peaks can exceed the limit.

**Fix approach**:
- Increase `max_splits` from 3 to 5
- After building each curve, verify actual curvature; if violated, apply `subdivide_for_curvature_dense()` as fallback
- Replace fixed 2× safety factor with iterative refinement

### Issue 2: C0/C1 Continuity Gaps (3 tests)

**Tests**: `C0AdjacentSegmentEndpointsShouldMatchExactly`, `C1AdjacentSegmentTangentAngleShouldBeVerySmall`, `AdjacentSegmentTangentsShouldBeNearC1Continuous`

**Root cause**: Each segment's curve is built independently. The running_spline tracks the global path, but segment boundaries don't always connect smoothly because:
- Crossover passthrough curves may end at positions that don't align with the next segment's start
- Connector curves are built from surface positions rather than from the running spline's actual endpoint

**Fix approach**: Ensure each segment's first curve starts at the running spline's end point with matching tangent direction.

### Issue 3: Weak Shape Expression (9 tests)

**Tests**: All `GeometryReferenceBehavior` and `GeometryRefLocalStitchShapes` tests

**Root cause**: `stitch_shape.cpp` computes correct shape parameters (z_bulge, apex_lean_x, height_multiplier, width_multiplier) but the geometry builder (`loop_precompute.cpp`) doesn't fully use them:

- **z_bulge**: Applied as tangent lean in `build_surface_guided_loop()` lines 246-251, but the actual waypoints (entry, apex, exit) have no Z displacement. The lean only subtly affects curve shape, not the overall Z position.
- **apex_lean_x**: Applied to `geom.apex.x` in `precompute_loop_geometry()` line 104, but then overwritten when crossover slots reposition the apex.
- **height_multiplier**: Compounds with `apex_height_factor` in line 105, which should work if apex calculation is correct.
- **width_multiplier**: Stored in `loop_width` but not used in geometry construction.

**Fix approach**:
- Add explicit Z displacement to loop waypoints: `apex.z += z_bulge`, and offset entry/exit Z
- Preserve apex_lean_x through crossover slot repositioning
- Use `loop_width × width_multiplier` to set entry/exit spread for yarn-overs
- Ensure `height_multiplier` correctly scales the effective loop height

### Issue 4: Sharp Polyline Corners (1 test)

**Test**: `PolylineLocalTurnShouldAvoidSharpCorners`

**Root cause**: Curvature violations create sharp bends in the polyline. Fixing curvature compliance (Issue 1) should largely fix this.

### Issue 5: Degenerate Boundaries (1 test)

**Test**: `CastOnOnlyShouldProduceFiniteNonDegenerateGeometry`

**Root cause**: Single-segment pattern (just "C") — `initialize_running_spline()` uses `first_target = positions[0] + Vec3(1,0,0)` when only one position exists, and the init tail may create a zero-length bezier.

**Fix approach**: Handle single-position edge case in `initialize_running_spline()`.

### Issue 6: Self-Contact / Clearance (6 tests)

**Tests**: 2× `GeometrySelfContactReference`, 4× `GeometryGlobalClearanceReferenceExt`

**Root cause**: Geometry builder doesn't enforce minimum distance between non-local yarn sections. Loops and connectors can overlap.

**Fix approach**: Add post-construction clearance enforcement pass — sample the full polyline, detect violations, push apart by adjusting control points.

### Issue 7: Non-Linear Scaling (2 tests)

**Tests**: `DoublingNeedleDiameterShouldDoubleHeightPrecisely`, `DoublingNeedleDiameterShouldDoubleWidthPrecisely`

**Root cause**: Some geometry construction uses fixed-size elements that don't scale with gauge. For example, `initialize_running_spline()` computes tail length from `min_bend_radius * 3.0f` (yarn property, not gauge-scaled).

**Fix approach**: Ensure ALL dimension calculations in geometry construction use gauge-derived values.

### Issue 8: Asymmetric Construction (4 tests)

**Tests**: `StockinetteSwatchShouldBeMirrorSymmetricInX`, 3× `GeometryRefSymmetryReflectionExt`

**Root cause**: Segments are built left-to-right with the running spline accumulating bias. `center_geometry_x()` centers the bounding box but doesn't fix internal asymmetry.

**Fix approach**: Apply mirror-averaging as post-process for symmetric patterns, or build from center outward.

### Issue 9: Parameter Sensitivity (1 test)

**Test**: `CompressedRadiusSweepShouldMonotonicallyIncreaseGlobalClearanceProxy`

**Root cause**: Larger yarn radius doesn't monotonically increase clearance because geometry construction may produce tighter layouts for thicker yarn (more curvature-safe splitting → more control points → more places to get close).

**Fix approach**: Ensure clearance enforcement (Issue 6) scales with yarn radius.

### Issue 10: Sampling Consistency (1 test)

**Test**: `FixedAndArcLengthSamplingShouldPreserveStrictLocalTurnProfile`

**Root cause**: `to_polyline_fixed()` and `to_polyline()` use different sampling strategies. When segment boundaries have tight curves, the resampled turn profiles diverge.

**Fix approach**: Improve both sampling methods to produce consistent results at segment boundaries.

## Incremental Implementation Order

### Pass 1: Fix 5 test patterns (test-only changes)
Files: `tests/test_surface_yarn_length.cpp`, `tests/test_geometry_knit_reference_behavior.cpp`

### Pass 2: Curvature compliance + degenerate boundaries (~6 tests)
Files: `math/curvature_utils.cpp`, `geometry/loop_precompute.cpp`, `geometry/geometry_builder.cpp`

### Pass 3: C0/C1 continuity at segment boundaries (~3 tests)
Files: `geometry/geometry_builder.cpp`, `geometry/loop_precompute.cpp`

### Pass 4: Shape fidelity — Z depth, lean, elongation, width (~9 tests)
Files: `geometry/loop_precompute.cpp`, `stitch_shape/stitch_shape.cpp`

### Pass 5: Global quality — clearance, scaling, symmetry, sampling (~9 tests)
Files: `geometry/geometry_builder.cpp`, `geometry/geometry_validator.cpp`, `geometry/geometry_path.cpp`

## Verification

After each pass:
1. Build: `cmake --build build`
2. Run full test suite: `ctest --test-dir build --output-on-failure`
3. Verify targeted tests pass AND no previously passing tests regress
4. Commit if green
