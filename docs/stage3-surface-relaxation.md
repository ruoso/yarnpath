# Stage 3: Surface Relaxation (YarnPath to SurfaceGraph)

## Purpose

Stage 3 converts the topological yarn path (a sequence of segments with parent-child relationships) into a 2.5D surface mesh and relaxes it to find physically plausible node positions. The output is a `SurfaceGraph` with solved positions, stitch axes, and fabric normals that Stage 4 (geometry builder) uses to place Bezier spline curves.

## Pipeline Overview

```
YarnPath ──► SurfaceBuilder ──► SurfaceGraph ──► SurfaceSolver ──► solved SurfaceGraph
                                                    │
                                            ForceConfig, SolveConfig
```

## SurfaceBuilder Pipeline

`SurfaceBuilder::from_yarn_path()` runs these steps in order:

1. **create_nodes()** — One `SurfaceNode` per `YarnSegment`. Copies `forms_loop`, computes `StitchShapeParams` via `compute_stitch_shape()`, and sets mass from `target_yarn_length * linear_density`.

2. **widen_loops_for_crossovers()** — Counts how many yarn cross-sections pass through each parent loop (child slots + parent legs) and widens `loop_width` to `total_cross_sections * compressed_diameter` when the computed shape is too narrow.

3. **create_continuity_edges()** — Connects consecutive segments (i, i+1) with `YarnContinuity` spring edges. Rest length = `stitch_width * tension_factor`, clamped to a minimum of `half_width_a + half_width_b` (so widened nodes don't overlap). Stiffness scales with yarn density.

4. **create_passthrough_edges()** — Connects each child segment to its parent loop(s) with `PassThrough` spring edges. Rest length uses a 3D Pythagorean formula: `sqrt(y_component^2 + z_component^2)` where y_component accounts for loop height + clearance and z_component accounts for the parent's z_bulge + compressed_radius. Slip stitches (Transferred) get 0.3x stiffness.

5. **create_constraints()** — Adds `MaxStretch` constraints on continuity edges (limit = rest_length * (1 + elasticity)) and `MinDistance` constraints on passthrough edges (limit = min_clearance).

6. **initialize_positions()** — Places nodes on a grid: row 0 goes right, subsequent rows alternate direction, Y increases by `loop_height` per row. Z offset comes from `z_bulge` for initial fabric curl. Stitch axes are initialized from neighbor positions.

## StitchShape Computation

`compute_stitch_shape()` determines the 3D shape of each stitch from three independent axes:

| Axis | Values | Effect |
|------|--------|--------|
| **LoopOrientation** | Front / Back / Neutral | Sets z_bulge_factor (+1 / -1 / +0.5) and apex_height_factor |
| **WrapDirection** | CW / CCW / None | Sets apex_lean_x (right lean / left lean / 0) |
| **WorkType** | Worked / Transferred / Created | Modifies z_bulge_factor, height/width multipliers |

Derived values:
- `z_bulge = compressed_diameter * 2 * z_bulge_factor`
- `loop_width = base_loop_width * width_multiplier`
- `loop_height = base_loop_height * height_multiplier`

## SurfaceSolver

The solver uses Verlet integration with constraint projection:

### Per-step cycle
1. **compute_forces()** — Spring forces, passthrough tension, loop curvature, collision repulsion, bending resistance, gravity, damping
2. **integrate_verlet()** — `v += (F/m) * dt`, `x += v * dt`
3. **apply_floor_constraint()** — Optional floor plane
4. **project_constraints()** — Iterative constraint projection (MaxStretch, MinDistance) using graph-colored independent sets for parallelism

### Force model

| Force | Purpose | Key parameter |
|-------|---------|---------------|
| Spring | Hooke's law on all edges | `edge.stiffness`, `edge.rest_length` |
| Passthrough tension | Extra straightening on through-loop edges | `passthrough_tension_factor` |
| Loop curvature | Encourages natural loop height | `loop_curvature_strength` |
| Collision | AABB-based repulsion via spatial hash grid | `collision_strength` |
| Bending | Prevents sharp folds along continuity chain | `bending_stiffness`, `min_bend_angle` |
| Gravity | Downward pull proportional to mass | `gravity_strength` |
| Damping | Velocity-proportional drag | `damping` |

### Convergence
The solver checks energy change per iteration against `convergence_threshold`. After convergence (or max iterations), it computes fabric normals.

### Fabric Normal Computation
`compute_fabric_normals()` runs two passes:
1. For each node: compute `stitch_axis` from continuity neighbors, derive `wale = Gram-Schmidt(unit_y, course)`, set `fabric_normal = course x wale`.
2. Propagate consistent orientation: flip any normal that points opposite its predecessor.

## Edge Rest Length Formulas

### Continuity edges
```
base_rest = stitch_width * (1 + (1 - tension) * 0.25)
min_rest  = half_width_a + half_width_b
rest      = max(base_rest * factor, min_rest)
```

### Passthrough edges (3D)
```
y = loop_height + 1.5 * min_clearance
z = |parent.z_bulge| + compressed_radius
rest = sqrt(y^2 + z^2) * factor
```

## Known Limitations

- Initial positions use a simple grid layout; complex topologies (cables, lace) may need more iterations to untangle.
- Collision detection uses AABB overlap, which is conservative for anisotropic stitch shapes.
- The solver uses a static time step; adaptive dt could improve convergence for stiff systems.
- Fabric normals assume a single connected component; disconnected regions (e.g., separate panels) would need independent orientation seeds.
- Bending resistance is applied only along the continuity chain; cross-row bending (wale direction) is not modeled.
