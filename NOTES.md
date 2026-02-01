# Knitted Geometry from Instructions: Architecture Overview (MVP)

## Goal

Given knitting instructions in **Shirley Paden’s format**, produce a **plausible 3D geometry** of the knitted piece.

“Plausible” means:

* topologically correct (loops interlock correctly),
* visually knit-like,
* roughly consistent with gauge and shaping,
* suitable for preview, inspection, and later refinement,

**without** solving a full physical simulation of yarn tension, drape, or relaxation.

---

## Core Insight

Knitted fabric has three distinct layers of description:

1. **Topology** — how loops interlock
2. **Geometry** — where yarn runs in space
3. **Physics** — how the yarn relaxes under forces

These layers should be **separated**.

For the MVP:

* we fully solve **(1) Topology**,
* we construct **(2) Geometry** using physics-aware constraints,
* we *do not* solve **(3) Physics** (energy minimization, equilibrium).

Physics constraints guide geometry; physics solvers are optional later.

---

## High-level Pipeline

```
Shirley Paden Instructions
        ↓
StitchNode Graph (fabric topology)
        ↓
YarnPath (linear yarn topology)
        ↓
Constraint-aware Geometry Assembly
        ↓
Plausible 3D Knit Geometry
```

Each stage has a clear responsibility.

---

## 1. Parsing Instructions → StitchNode Graph

### What this represents

The **StitchNode graph** is a logical description of the fabric:

* nodes = stitches
* edges = “this stitch was worked through that stitch”
* rows, stitch counts, panels, increases/decreases are explicit

This graph answers:

* how many stitches exist at each row,
* how stitches depend on previous rows,
* how shaping changes topology.

### Properties

* row-structured
* panel-aware
* acyclic across rows
* **no geometry, no yarn continuity**

This maps very naturally from Shirley Paden’s notation.

---

## 2. StitchNode Graph → YarnPath

### Why this step exists

A stitch graph is **not drawable**.

Yarn is:

* continuous,
* ordered,
* one-dimensional.

The **YarnPath** converts fabric topology into a **linear sequence of yarn events**.

### YarnPath definition

A YarnPath is an ordered list of `LoopEvent`s, each describing:

* which new loop is created,
* which previous loop(s) it passes through,
* orientation (knit / purl),
* metadata (increase, decrease, cable crossing, etc.).

This step is **purely topological**.
No geometry. No physics.

### Why this separation matters

* StitchNode graph = *what the fabric is*
* YarnPath = *how the yarn moves*

You keep both.

---

## 3. Geometry Assembly (Constraints-Only)

This is where the MVP lives.

### Key principle

We **respect physics constraints** but do **not solve physics**.

That means:

* no force integration,
* no equilibrium solving,
* no global relaxation.

Instead, we use **procedural geometry templates** that are *physics-aware*.

---

### 3.1 Fabric-space embedding

Each stitch gets fabric coordinates `(u, v)`:

* `v` from row index and row gauge
* `u` from column index and stitch gauge

Panels may be embedded onto:

* a plane (flat knitting),
* a cylinder (in-the-round),
* simple surfaces (e.g. tapered sleeves).

This gives a global scaffold.

---

### 3.2 Stitch glyphs (local geometry templates)

Each stitch type corresponds to a **glyph**:

* a small parametric yarn curve representing one loop,
* oriented for knit vs purl,
* sized according to gauge and yarn radius.

Each glyph:

* has entry and exit tangents,
* has bounded curvature,
* respects minimum loop size,
* assumes yarn thickness.

This is where *physics constraints are encoded*, but not solved.

---

### 3.3 Yarn assembly

We traverse the YarnPath in order:

* place each glyph at its fabric-space location,
* connect glyphs along the yarn path,
* enforce:

  * continuity (C¹ smoothing),
  * curvature limits,
  * approximate arc-length conservation.

This assembles a **single continuous yarn curve**.

---

### 3.4 Geometry-only cleanup (optional but recommended)

Lightweight, non-physical fixes:

* simple repulsion if segments get closer than `2 × yarn_compressed_radius`,
* curvature clamping,
* local smoothing.

No forces, no iterations toward equilibrium — just validity.

---

## What This MVP Produces Well

✅ Correct loop interlocking
✅ Correct stitch counts and shaping
✅ Plausible 3D knit texture
✅ Believable garment volume
✅ Fast, deterministic results
✅ Excellent starting point for later refinement

This is ideal for:

* pattern previews,
* editors,
* visualization,
* topology validation,
* user feedback.

---

## What This MVP Explicitly Does *Not* Solve

❌ Edge curl
❌ Drape and sag under gravity
❌ Global tension redistribution
❌ Blocking effects
❌ Exact cable “pop”
❌ Mechanical validation of tightness

These are **emergent phenomena** that require solving physics.

---

## Why This Is Not a Hack

Because:

* Assembly ≠ relaxation
* Constraints ≠ solving

You are doing exactly what many physical systems do in practice:

* construct a feasible configuration,
* optionally relax it later.

This mirrors:

* knot diagrams vs tightened knots,
* cloth layout vs cloth simulation,
* protein backbone construction vs energy minimization.

The better your constraint-aware assembly, the *less* physics you need later.

---

## Extension Path (Future Work)

The architecture intentionally leaves doors open:

* add **local relaxation** for selected stitches (e.g. cables),
* add **panel-level reduced physics** for curl and sag,
* add **full yarn-level physics** as an optional “validate” mode.

None of these require changing:

* the StitchNode graph,
* the YarnPath,
* or the instruction parser.

---

## Final One-Sentence Summary

> We compile Shirley Paden’s instructions into fabric topology, linearize that topology into a yarn path, and assemble a continuous, constraint-aware yarn geometry — using physics as *rules*, not as a *solver* — to produce a fast, plausible 3D knit suitable for preview and future refinement.

If you want, next we can:

* turn this into a diagram,
* sketch concrete data structures,
* or write pseudocode for the StitchNode → YarnPath conversion.

