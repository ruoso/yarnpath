# Idealized Realistic Yarn Spline Generation — Plan

## Goal

Produce idealized, physically-correct yarn splines from surface-relaxed positions
and stitch topology. The geometry builder constructs "textbook correct" loop shapes
without tension adjustment — a future spline relaxation stage will handle that by
comparing arc lengths to `target_yarn_length`.

## Architecture: Option C — Hybrid

The surface solver owns coarse positions + local frames + shape parameters.
The geometry builder owns detailed spline construction within those frames.

- Surface provides: position, stitch_axis, fabric_normal, LoopShapeParams per segment
- Geometry builds: U-shaped loop curves, crossover threading, inter-segment connectors
- Future relaxation stage: adjusts arc lengths to match yarn length constraints

---

## Step 1: Serialize `stitch_axis` and `shape` on `SurfaceNode`

### Problem

`SurfaceNode` already has `stitch_axis` (Vec3) and `shape` (LoopShapeParams)
computed during surface relaxation, but `surface_graph_json.hpp` does not
serialize either field. The geometry builder is forced to re-derive shape
parameters independently via `compute_loop_shape_params()`, and re-derive
course direction from raw positions.

### Requirements

- Add `stitch_axis` to `to_json`/`from_json` for `SurfaceNode` in
  `serialization/surface_graph_json.hpp`
- Add `shape` (LoopShapeParams) to `to_json`/`from_json` for `SurfaceNode` —
  this requires adding LoopShapeParams serialization if it doesn't exist
- Round-trip test: serialize a SurfaceGraph, deserialize it, verify `stitch_axis`
  and `shape` fields survive
- No changes to the solver itself — just the serialization layer

### Files to modify

- `serialization/surface_graph_json.hpp`
- `stitch_shape/stitch_shape.hpp` (if LoopShapeParams serialization is needed)
- Tests for round-trip verification

---

## Step 2: Compute and store `fabric_normal` at end of solve

### Problem

The geometry builder currently estimates fabric normal by crossing the tangent
between consecutive segment positions with global Y-up (`estimate_fabric_normal()`
in `loop_precompute.cpp`). This breaks for non-planar fabrics and ignores the
rich adjacency information the surface solver already has.

### Requirements

- Add `fabric_normal` field (Vec3) to `SurfaceNode` in `surface/surface_node.hpp`
- At the end of `surface_solver.cpp` solve, after convergence, run a final O(N)
  pass computing `fabric_normal` per node:
  - Use continuity neighbors (prev/next from adjacency index) to get a local
    tangent plane
  - `fabric_normal = stitch_axis.cross(neighbor_direction).normalized()`
  - Boundary fallback: at cast-on/bind-off edges where only one neighbor exists,
    use `LoopOrientation` (Front → +Z, Back → -Z) to pick a default normal
  - Ensure consistent orientation: normal should point toward the "front" face
    of the fabric (positive Z in the default layout)
- Serialize `fabric_normal` in `surface_graph_json.hpp`
- The normal must be unit-length (or zero for degenerate cases)

### Files to modify

- `surface/surface_node.hpp` — add field
- `surface/surface_solver.cpp` — add post-solve computation pass
- `serialization/surface_graph_json.hpp` — serialize the new field
- Tests for normal computation correctness

---

## Step 3: Replace `PositionResolver` with `SegmentFrame` provider

### Problem

`PositionResolver` currently returns only a `Vec3` position per segment. The
geometry builder then re-derives fabric normal, course direction, and shape
parameters from scratch using heuristics. With Steps 1-2, all of this data is
available directly on `SurfaceNode`.

### Requirements

- Define a `SegmentFrame` struct:
  ```
  struct SegmentFrame {
      Vec3 position;        // relaxed world-space position
      Vec3 stitch_axis;     // course direction (along the row)
      Vec3 fabric_normal;   // perpendicular to fabric surface
      Vec3 wale_axis;       // = fabric_normal.cross(stitch_axis), row-stacking direction
      LoopShapeParams shape; // z_bulge, loop_height, stitch_width, multipliers, etc.
  };
  ```
- Rewrite `PositionResolver` to return `SegmentFrame` per segment, reading all
  fields from the enriched `SurfaceNode`
- `wale_axis` is derived as `fabric_normal.cross(stitch_axis).normalized()`
- Keep the parentless-segment Y-adjustment logic (lower parentless segments
  below their children by one yarn diameter)
- Remove `estimate_fabric_normal()` from `loop_precompute.cpp` — it's replaced
  by `SegmentFrame::fabric_normal`
- Update all call sites in `geometry_builder.cpp` and `loop_precompute.cpp` to
  use `SegmentFrame` instead of raw `Vec3` positions

### Files to modify

- `geometry/position_resolver.hpp` — new `SegmentFrame` struct, updated API
- `geometry/position_resolver.cpp` — read from enriched SurfaceNode
- `geometry/geometry_builder.cpp` — use SegmentFrame throughout
- `geometry/loop_precompute.hpp` / `.cpp` — remove `estimate_fabric_normal()`,
  accept SegmentFrame
- `geometry/geometry_build_state.hpp` — may need to store current SegmentFrame

---

## Step 4: Redesign loop curves as U-shaped wraps in local frame

### Problem

Current loops are simple "rise to apex, fall back" Hermite curves with Z-bulge
applied as a tangent lean. This doesn't capture the actual yarn path: a U-shaped
wrap that enters from below, passes through the parent loop opening, wraps over
the top, and exits back through. The `PhysicalLoop` cylinder-wrap model exists
but isn't connected to the builder.

### Requirements

- In the segment's local frame (stitch_axis = X/course, wale_axis = Y/up,
  fabric_normal = Z/depth):
  - **Entry leg**: yarn arrives from the previous segment, curves downward
    (-wale) and forward (+fabric_normal for knit, -fabric_normal for purl),
    passing through the parent loop opening
  - **Through opening**: the yarn passes through a gap of
    `effective_opening_diameter` along the fabric_normal axis, at the base of
    the parent loop
  - **Rise to apex**: yarn curves upward (+wale) to apex height
    (`loop_height * height_multiplier`), offset by `z_bulge` along
    fabric_normal, leaned by `apex_lean_x` along stitch_axis for decreases
  - **Exit leg**: mirror of entry (or asymmetric per `symmetric_exit` flag),
    returning through the opening
- All waypoints computed in local frame, then transformed to world space using
  `SegmentFrame` rotation matrix
- Use `PhysicalLoop`'s wrap radius concept (half needle diameter) to set
  curvature at the apex — this gives the rounded top of the loop
- Build as Hermite curves through waypoints using existing
  `build_curvature_safe_hermite_segments()` — curvature constraint still applies
- The U-shape must look correct for:
  - Knit stitches (front bulge, positive z_bulge)
  - Purl stitches (back bulge, negative z_bulge) — near-mirror of knit
  - Decreases (K2tog/SSK: strong lean via apex_lean_x)
  - Yarn-overs (wider via width_multiplier = 1.4)
  - Slipped stitches (taller via height_multiplier = 1.8)

### Files to modify

- `geometry/loop_precompute.hpp` — new loop construction API accepting SegmentFrame
- `geometry/loop_precompute.cpp` — rewrite `build_loop_curve()` and
  `build_loop_with_crossings()`
- `geometry/geometry_builder.cpp` — wire up new loop construction
- `geometry/physical_loop.hpp` / `.cpp` — keep as reference, extract wrap radius
  concept, potentially retire from active build

---

## Step 5: Distribute crossover slots along parent loop legs

### Problem

Current crossover slots cluster at the parent's apex, spaced only by
`yarn_compressed_diameter`. All children thread through nearly the same point.
Real yarn interlocks at distinct positions along the parent loop's entry and
exit legs.

### Requirements

- Change `CrossoverSlot` representation from `{position, offset, tangent}` to
  `{parameter_t, leg (entry/exit), normal_offset}` — a parametric position
  along the parent's already-constructed loop curve
- Children entering the parent claim slots on the parent's entry leg; children
  exiting claim slots on the exit leg
- Slots spaced by `yarn_compressed_diameter` along the leg's arc length
- Normal offset (front/back crossing) determined by fabric_normal direction
- Slot claiming remains nearest-available by proximity
- Depends on Step 4: parent loop curve must be constructed before children can
  claim parametric positions on it
- Current yarn-order processing guarantees parents built before children — this
  ordering is preserved

### Files to modify

- `geometry/crossover_geometry.hpp` — redesigned CrossoverSlot struct
- `geometry/crossover_geometry.cpp` — new distribution algorithm
- `geometry/loop_precompute.cpp` — updated slot claiming
- `geometry/geometry_builder.cpp` — updated passthrough construction

---

## Step 6: Simplify `GeometryBuildState`

### Problem

`GeometryBuildState` currently stores `effective_loop_height`,
`effective_stitch_width`, `effective_opening_diameter` as global constants
derived from `LoopDimensions`. With `SegmentFrame` providing per-segment
`LoopShapeParams`, these globals are wrong for segments with non-default
multipliers (yarn-overs, slips, transfers).

### Requirements

- Remove `effective_loop_height`, `effective_stitch_width`,
  `effective_opening_diameter` from `GeometryBuildState`
- Keep only: `running_spline`, `max_curvature`, `yarn_compressed_radius`,
  `yarn_compressed_diameter`
- Per-segment dimensions come from `SegmentFrame::shape` multiplied by base
  `LoopDimensions` values
- Base `LoopDimensions` can remain as a parameter to the builder (it's derived
  from yarn + gauge, not per-segment)
- Verify all references to removed fields are updated to use per-segment values

### Files to modify

- `geometry/geometry_build_state.hpp` / `.cpp` — remove per-segment fields
- `geometry/geometry_builder.cpp` — pass LoopDimensions + SegmentFrame to
  loop/crossover construction
- `geometry/loop_precompute.cpp` — compute effective dimensions per-segment
  from LoopDimensions * shape multipliers

---

## Execution Order

Steps 1-2 are independent surface-side changes (can be done in parallel).
Step 3 depends on 1-2 (needs the enriched data to read).
Step 4 depends on 3 (needs SegmentFrame).
Step 5 depends on 4 (needs parent loop curves).
Step 6 can be done alongside 4-5 (cleanup).

```
[1: Serialize stitch_axis + shape] ──┐
                                      ├──→ [3: SegmentFrame] ──→ [4: U-shaped loops] ──→ [5: Crossover distribution]
[2: Compute fabric_normal]     ──────┘                                    │
                                                                          └──→ [6: Simplify build state]
```

## Tests

All tests use Google Test. Existing test helpers: `make_yarn_path_from_compact()`
for building patterns from compact notation (C=CastOn, K=Knit, P=Purl,
B=BindOff, O=YarnOver, 2=K2tog, S=SSK, L=Slip), `make_default_yarn()` (worsted),
`make_default_gauge()` (5mm needles). Existing end-to-end helpers
`build_surface_for_pattern()` and `build_geometry_for_pattern()` are duplicated
across ~15 test files — reuse or consolidate as needed.

New test file: `tests/test_serialization.cpp` (none exists today).
New test file: `tests/test_fabric_normal.cpp`.
New test file: `tests/test_segment_frame.cpp`.
Existing files updated: `tests/test_geometry_reference_behavior.cpp` and others.

---

### Step 1 Tests: Serialization of `stitch_axis` and `shape`

**File: `tests/test_serialization.cpp`**

These are pure data round-trip tests — no solver or geometry involved.

1. **SurfaceNode_StitchAxis_RoundTrip** — Create a `SurfaceNode` with a
   non-trivial `stitch_axis` (e.g. `{0.6, 0.8, 0.0}`). Serialize to JSON,
   deserialize back. Assert the `stitch_axis` components match within `1e-6`.

2. **SurfaceNode_Shape_RoundTrip** — Create a `SurfaceNode` with `shape` set to
   a `LoopShapeParams` with non-default values (e.g. `z_bulge = -2.5`,
   `apex_lean_x = 0.3`, `height_multiplier = 1.8`, `width_multiplier = 1.4`,
   `symmetric_exit = false`). Serialize/deserialize. Assert every field of the
   deserialized `shape` matches.

3. **SurfaceNode_Shape_DefaultValues_RoundTrip** — Same as above but with
   default-constructed `LoopShapeParams`. Verifies that zero/default values
   aren't accidentally dropped during serialization.

4. **SurfaceGraph_FullRoundTrip_PreservesFrameData** — Build a small
   `SurfaceGraph` (e.g. from a 3-stitch stockinette: `"CCC,KKK,BBB"`) via
   `build_surface_for_pattern()`. Serialize the entire graph to JSON, deserialize
   into a new `SurfaceGraph`. For each node, assert `stitch_axis` and all
   `shape` fields match the original.

5. **LoopShapeParams_Serialization_AllFields** — Unit test for the
   `LoopShapeParams` `to_json`/`from_json` in isolation (not wrapped in a
   `SurfaceNode`). Verifies every field: `z_bulge`, `loop_height`,
   `stitch_width`, `z_bulge_factor`, `apex_lean_x`, `apex_height_factor`,
   `symmetric_exit`, `entry_tangent_scale`, `width_multiplier`,
   `height_multiplier`.

---

### Step 2 Tests: `fabric_normal` computation

**File: `tests/test_fabric_normal.cpp`**

These test the post-solve normal computation pass independently. Build a
`SurfaceGraph`, run the solver, then verify the resulting `fabric_normal` values.

1. **FlatStockinette_NormalsPointPositiveZ** — Build surface for a flat
   stockinette swatch (`"CCCC,KKKK,KKKK,BBBB"`). After solving, all
   `fabric_normal` vectors should have a dominant +Z component (front-facing).
   Assert `fabric_normal.z > 0.5` for all nodes. This validates the basic
   "front of fabric" convention.

2. **FlatStockinette_NormalsAreUnitLength** — Same swatch. Assert
   `fabric_normal.length()` is within `1e-3` of `1.0` for all nodes.

3. **FlatStockinette_NormalsAreConsistent** — All normals in a flat swatch
   should roughly agree. Assert that the dot product of any two nodes' normals
   is `> 0.8` (within ~37° of each other).

4. **NormalPerpendicularToStitchAxis** — For each node, assert
   `|fabric_normal.dot(stitch_axis)| < 0.1` — the normal should be
   approximately perpendicular to the course direction.

5. **BoundaryNodes_HaveValidNormals** — Check that cast-on row (first row) and
   bind-off row (last row) nodes have non-zero, unit-length normals. These are
   boundary nodes with only one continuity neighbor, so the fallback logic is
   exercised.

6. **PurlRows_NormalStillFrontFacing** — Build a garter-like swatch
   (`"CCC,PPP,KKK,BBB"`). Even on purl rows, the `fabric_normal` should still
   point toward the fabric front (+Z dominant). The normal is a property of the
   fabric surface, not the stitch orientation — `z_bulge` in `LoopShapeParams`
   handles the knit/purl distinction separately.

7. **FabricNormal_Serialization_RoundTrip** — Solve a surface, serialize,
   deserialize. Assert `fabric_normal` survives the round trip (extends Step 1
   test to cover the new field).

---

### Step 3 Tests: `SegmentFrame` provider

**File: `tests/test_segment_frame.cpp`**

These test the `PositionResolver` → `SegmentFrame` conversion. Assumes Steps 1-2
are complete so `SurfaceNode` carries `stitch_axis`, `fabric_normal`, and `shape`.

1. **SegmentFrame_PositionMatchesSurface** — Build surface for a small swatch.
   Resolve `SegmentFrame` for each segment. Assert `frame.position` matches
   the surface node's `position` (within `1e-6`) for all segments that have
   parents. (Parentless segments are adjusted — tested separately.)

2. **SegmentFrame_StitchAxisMatchesSurface** — Assert `frame.stitch_axis`
   equals the corresponding surface node's `stitch_axis` (within `1e-6`).

3. **SegmentFrame_FabricNormalMatchesSurface** — Assert `frame.fabric_normal`
   equals the corresponding surface node's `fabric_normal` (within `1e-6`).

4. **SegmentFrame_WaleAxisIsOrthogonal** — For each frame, assert:
   - `|wale_axis.dot(stitch_axis)| < 1e-3` (perpendicular to course)
   - `|wale_axis.dot(fabric_normal)| < 1e-3` (perpendicular to normal)
   - `wale_axis.length()` within `1e-3` of `1.0` (unit length)

5. **SegmentFrame_FrameIsRightHanded** — Assert
   `stitch_axis.cross(wale_axis).dot(fabric_normal) > 0.9` — the three axes
   form a right-handed coordinate system.

6. **SegmentFrame_ParentlessAdjustment** — Build a swatch and verify that
   cast-on segments (which have no parents) have their position lowered below
   their children by approximately one yarn diameter. Compare
   `frame.position.y` to the min child `position.y - yarn_diameter`.

7. **SegmentFrame_ShapeMatchesSurface** — Assert `frame.shape.z_bulge`,
   `frame.shape.loop_height`, `frame.shape.stitch_width`, and all multiplier
   fields match the surface node's `shape`.

8. **SegmentFrame_KnitVsPurl_DifferentZBulge** — Build a swatch with both knit
   and purl rows (`"CCC,KKK,PPP,BBB"`). Assert that knit-row frames have
   `shape.z_bulge > 0` and purl-row frames have `shape.z_bulge < 0`.

---

### Step 4 Tests: U-shaped loop curves

**File: Updates to `tests/test_geometry_reference_behavior.cpp` and
`tests/test_geometry_stitch_types.cpp`; new file `tests/test_loop_shape.cpp`**

These test the actual spline shape produced for individual loops. Many of these
correspond to currently-failing reference behavior tests that should begin
passing.

#### Unit tests for loop shape (`tests/test_loop_shape.cpp`)

1. **KnitLoop_ApexInFrontOfBase** — Build geometry for a single knit stitch
   (`"CC,KK,BB"`). Sample the loop-forming segment's spline. Assert the apex
   point (max wale-axis displacement) has a positive `fabric_normal`-direction
   offset relative to the base — the loop bulges toward the front.

2. **PurlLoop_ApexBehindBase** — Same for purl. Assert the apex has a negative
   `fabric_normal`-direction offset. Knit and purl should be approximate
   mirrors in depth.

3. **KnitPurl_DepthMirror** — Build `"CCC,KKK,BBB"` and `"CCC,PPP,BBB"`.
   Measure the average Z (fabric_normal direction) offset of the loop apexes.
   Assert they have opposite sign and similar magnitude (within 20%).
   Corresponds to the existing `StitchDepth_StockinetteKnitPurl` reference test.

4. **Loop_PassesThroughParentOpening** — Build a 2-row swatch. For the
   loop-forming segments in row 2, sample the spline finely. Assert there
   exists at least one sample point where the `fabric_normal`-direction position
   crosses from one side to the other (the yarn physically passes through the
   opening). This confirms the U-shape rather than a simple over-the-top arc.

5. **Loop_ApexHeight_ScalesWithMultiplier** — Build geometry for a normal knit
   and a slipped stitch. Assert the slipped stitch's apex height (in wale
   direction) is approximately `height_multiplier` (1.8×) taller than the
   knit's. Corresponds to `SlipStitch_Elongation` reference test.

6. **Loop_Width_ScalesWithMultiplier** — Build geometry including a yarn-over.
   Assert the YO's loop width (in stitch_axis direction) is approximately
   `width_multiplier` (1.4×) wider than a regular knit. Corresponds to
   `YarnOver_ExtraYarn` reference test.

7. **Decrease_OpposingLean** — Build geometry with K2tog and SSK on the same
   row. Measure the apex `stitch_axis`-direction offset for each. Assert K2tog
   leans one direction and SSK leans the opposite (signs differ), and both have
   magnitude proportional to `apex_lean_x`. Corresponds to `DecreaseLean`
   reference test.

8. **Loop_CurvatureAtApex_BoundedByWrapRadius** — Sample curvature along the
   loop spline at the apex region. Assert max curvature ≤ `1 / (needle_diameter
   / 2)` — the yarn wraps around the needle, so the curvature can't exceed the
   reciprocal of the wrap radius.

9. **Loop_IsContinuousAndSmooth** — For each loop segment's spline, assert C0
   continuity (gaps < `1e-5`) and C1 continuity (tangent angle < 10°) at all
   internal Bézier boundaries.

#### Updated reference behavior tests

10. **StitchDepth_StockinetteKnitPurl** (existing, expected to start passing) —
    Knit and purl depth should be near-mirror.

11. **DecreaseLean_K2togVsSSK** (existing, expected to start passing) — K2tog
    and SSK should show strong opposing lean.

12. **SlipStitch_Elongation** (existing, expected to start passing) — Slipped
    stitches should be significantly taller.

13. **YarnOver_ExtraYarn** (existing, expected to start passing) — Yarn-overs
    should consume more arc length and be wider.

---

### Step 5 Tests: Crossover distribution

**File: `tests/test_crossover_distribution.cpp`**

1. **SingleChild_CrossoverAtLoopMidpoint** — Build a simple stockinette (one
   parent, one child per loop). Assert the child's crossover position lies
   approximately at the midpoint of the parent's entry/exit leg (not at the
   apex).

2. **MultipleChildren_CrossoversSpaced** — Build a pattern where one parent has
   multiple children (e.g. via a KFB increase that creates two stitches from
   one). Assert the crossover positions are spaced by at least
   `yarn_compressed_diameter` along the parent leg arc.

3. **Crossover_FrontBackOffset** — For a crossover slot, assert the crossing
   point is offset from the parent loop curve by approximately
   `yarn_compressed_radius` along the `fabric_normal` direction — the child
   yarn passes in front of or behind the parent.

4. **Crossover_EntryAndExitOnDifferentLegs** — For a loop-forming child, assert
   that the entry crossover is on the parent's entry leg and the exit crossover
   is on the parent's exit leg (not both on the same leg).

5. **Crossover_ParametricPosition_WithinLegBounds** — Assert that all claimed
   `parameter_t` values are within `[0, 1]` on their respective leg curves.

6. **Decrease_MultipleParents_DistinctSlots** — Build a K2tog (child passes
   through 2 parents). Assert the child claims one crossover from each parent,
   and the crossover positions are distinct (not overlapping).

---

### Step 6 Tests: Simplified `GeometryBuildState`

**File: Updates to `tests/test_geometry.cpp` and `tests/test_geometry_topology.cpp`**

These are mostly regression tests — verify that the cleanup doesn't break
existing behavior.

1. **PerSegment_DimensionsDifferByType** — Build geometry for a pattern with
   knit, yarn-over, and slip stitches on the same row. Assert that the effective
   dimensions used for each segment differ according to their
   `LoopShapeParams` multipliers (YO wider, slip taller). This validates that
   per-segment dimensions replaced the old global constants.

2. **BuildState_NoGlobalDimensions** — Compile-time/structural check: verify
   `GeometryBuildState` no longer has `effective_loop_height`,
   `effective_stitch_width`, `effective_opening_diameter` fields. (This could
   be a static_assert or simply verified by the fact that old code referencing
   those fields fails to compile.)

3. **ExistingGeometryTests_StillPass** — All tests in `test_geometry.cpp`,
   `test_geometry_topology.cpp`, `test_geometry_smoothness.cpp` should continue
   to pass (possibly with updated thresholds if the shape improves). This is a
   regression gate, not a new test.

4. **MixedPattern_GeometryIsFinite** — Build geometry for a complex mixed
   pattern (`"CCCCC,KK2KK,KKOKK,PPPPP,BBBBB"`). Assert all segments produce
   finite positions (no NaN, no Inf), non-zero arc length, and valid bounding
   box. This exercises per-segment dimensions with varied stitch types in one
   swatch.

---

### Integration Tests (Span Multiple Steps)

**File: `tests/test_geometry_integration.cpp`**

These end-to-end tests verify the full pipeline from pattern → surface →
enriched surface → geometry with the new architecture.

1. **EndToEnd_Stockinette_ProducesValidGeometry** — Full pipeline for a 4×4
   stockinette. Assert: no NaN, all segments have splines, C0/C1 continuity
   within tolerance, curvature bounded, bounding box is reasonable.

2. **EndToEnd_Ribbing_KnitPurlAlternate** — 1×1 ribbing pattern. Assert that
   adjacent columns alternate in Z-depth sign (knit columns forward, purl
   columns backward).

3. **EndToEnd_Decreases_LeanAndConverge** — Pattern with K2tog and SSK. Assert
   fabric narrows (fewer stitches in later rows), decrease stitches lean in
   opposite directions.

4. **EndToEnd_YarnOver_CreatesOpening** — Pattern with yarn-overs. Assert the
   YO segment's loop is wider than neighbors, arc length is longer.

5. **EndToEnd_Cable_CrossoverThreading** — A simple cable pattern. Assert that
   the crossing segments actually interleave in Z-depth (one passes in front
   of the other).

6. **EndToEnd_GeometryMatchesSurfacePositions** — For each segment, assert that
   the spline passes near (within a few yarn diameters of) the surface-relaxed
   position. The geometry adds loop detail but shouldn't drift far from the
   surface solution.

---

## Future Work (Out of Scope)

- **Spline relaxation pass**: Compare arc_length to target_yarn_length, derive
  tension, adjust control points. Separate pipeline stage after geometry.
- **Tube mesh generation**: Generate circular cross-sections along spline using
  rotation-minimizing frames. The idealized spline design should be compatible
  with this (smooth, C1-continuous, bounded curvature).
- **Decoupled per-segment construction**: Replace the running-spline accumulation
  with independent per-segment curve building + C1 stitching pass. Enables
  parallelism and better testability. Deferred — current sequential approach
  works for shape correctness.
