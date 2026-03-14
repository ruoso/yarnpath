# Stage 2: StitchGraph → YarnPath

## Purpose

Stage 2 converts the fabric topology graph (StitchGraph) into a linear yarn
path — the sequence of yarn segments that a single strand of yarn follows
through the fabric. This is the representation used by downstream geometry
stages to generate 3D curves.

## Input / Output

| | Type | Description |
|---|---|---|
| **In** | `StitchGraph` | Nodes with parent links, grouped by row |
| **In** | `YarnProperties` | Yarn radii, stiffness, elasticity |
| **In** | `Gauge` | Needle diameter → loop height, stitch width |
| **Out** | `YarnPath` | Ordered vector of `YarnSegment` |

## Algorithm: YarnPathBuilder

The builder walks the StitchGraph row by row, maintaining a *live loops*
state machine that tracks which loops are currently on the needle.

```
for each row:
    for each stitch node in row:
        1. Collect parent loops from node.worked_through
           → consume matching entries from live_loops_
        2. Determine orientation metadata from stitch type + row side
        3. Create a YarnSegment with through=parents, forms_loop=true
        4. Add segment to live_loops_ (unless bind-off)
```

### Live loops state

`live_loops_` is a vector of `SegmentId` representing loops currently on the
needle, in needle order. `consume_live_loop(pos)` removes and returns the loop
at a given position. New loops are appended via `add_live_loop(id)`.

For WS rows, the StitchGraph has already reversed stitch order, so the builder
processes stitches in their natural working direction.

## YarnSegment fields

| Field | Type | Description |
|---|---|---|
| `through` | `vector<SegmentId>` | Parent loop segments this yarn passes through |
| `forms_loop` | `bool` | Whether this segment forms a new loop |
| `orientation` | `LoopOrientation` | Front/Back/Neutral — fabric-space face |
| `wrap_direction` | `WrapDirection` | Clockwise/CounterClockwise/None — lean |
| `work_type` | `WorkType` | Worked/Transferred/Created |
| `target_yarn_length` | `float` | Pre-calculated yarn consumption (mm) |

## Orientation model

Stitch instructions have an *instruction-space* orientation:
- Knit → Front, Purl → Back

On WS rows, the fabric is flipped, so the builder applies
`orientation_instruction_to_fabric(side, orientation)`:
- RS: pass through unchanged
- WS: Front↔Back swap, Neutral unchanged

This means a WS Purl (instruction Back) becomes fabric Front — matching the
standard knitting convention that stockinette (RS knit, WS purl) produces
all-Front fabric.

## Yarn length pre-calculation

Base formula: `π × loop_height + stitch_width`

Modifiers applied multiplicatively:

| Condition | Factor | Rationale |
|---|---|---|
| Purl (Back orientation) | ×0.88 | Shorter yarn path through back |
| K2tog (Clockwise) | ×0.82 | Right-leaning, tight |
| SSK (CounterClockwise, 2 parents) | ×0.86 | Left-leaning, slightly looser |
| S2KP (CounterClockwise, 3 parents) | ×0.80 | Triple decrease, very tight |
| Slip (Transferred) | 0.5 × stitch_width | Minimal connector |
| Created, no parents (YO, CastOn) | π × loop_height | Loop only, no passthrough |
| Created + twist (M1L/M1R) | π × loop_height × 1.1 | Twisted pickup |

## Known limitations

- Single yarn only — no color work or multi-strand patterns.
- Cable segments are treated as individual knit stitches with reordered parents;
  no special yarn length adjustment for the cable crossing itself.
- Bind-off segments use the base knit formula; real bind-off chains consume
  more yarn per stitch.
- The yarn length modifiers are empirical approximations, not derived from
  physical simulation.
