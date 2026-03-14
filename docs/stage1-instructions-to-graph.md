# Stage 1: Instructions to Graph

## Purpose

Stage 1 transforms text knitting instructions into a fabric topology graph (StitchGraph). Each node in the graph represents a single stitch, with edges encoding which stitches were worked through which parent stitches.

## Sub-stages

### Parser (text → AST)

The parser (`parser/parser.cpp`) tokenizes pattern text and produces a `PatternAST` — a sequence of `CastOnNode`, `BindOffNode`, and `RowNode` entries. Each `RowNode` contains a tree of `RowElement`s (stitches and repeats).

### Emitter (AST → PatternInstructions)

The emitter (`parser/emitter.cpp`) flattens the AST into `PatternInstructions` — a flat sequence of `RowInstruction`s, each with an explicit `RowSide` and a vector of `StitchInstruction`s.

Key behaviors:
- **RS/WS auto-alternation**: Cast-on is always RS. Subsequent rows use the next alternating side unless the AST node specifies an explicit side. Bind-off uses the next side in the alternation.
- **Repeat expansion**: `*body*, rep from * to end/last N sts` is expanded based on available stitch count.
- **Stitch count tracking**: The emitter tracks live stitch count to determine repeat iteration counts.

### Graph builder (PatternInstructions → StitchGraph)

The graph builder (`stitch_graph/stitch_node.cpp`, `from_instructions()`) processes each row's instructions sequentially, maintaining a `live_stitches` vector that represents the stitches available on the needle.

## Key data structures

| Structure | Role |
|-----------|------|
| `StitchInstruction` | Tagged union of instruction types with counts (Knit, CastOn{N}, Repeat{body, times}, etc.) |
| `RowInstruction` | One row: side + flat vector of StitchInstructions |
| `PatternInstructions` | All rows for a single panel |
| `StitchNode` | One stitch in the graph: id, operation, row, column, parent links |
| `StitchGraph` | All nodes + row metadata; built from PatternInstructions |

## WS row handling

WS rows are worked right-to-left in the fabric. The graph builder models this by:

1. **Reversing** the `live_stitches` vector before processing the row's instructions.
2. Processing instructions normally (consuming from front, appending to back).
3. **Reversing** `live_stitches` back after processing, so subsequent RS rows see the correct order.
4. **Remapping columns** to fabric position: `column = stitch_count - 1 - column` for all nodes in the WS row. This ensures `column` consistently means "fabric column" regardless of row side.

## RS/WS auto-alternation in the emitter

The emitter tracks a `next_side` state:
- Cast-on: always RS, sets `next_side = WS`.
- Row nodes: use explicit side if provided by the AST, otherwise use `next_side`. After emitting, `next_side` flips to the opposite of the row's actual side.
- Bind-off: uses `next_side`. After emitting, flips `next_side`.

## Stitch count flow

Each instruction type has a fixed consumed/produced contract:

| Type | Consumed | Produced |
|------|----------|----------|
| Knit, Purl, Slip | 1 | 1 |
| CastOn{N} | 0 | N |
| BindOff{N} | N | 0 |
| YarnOver, M1L, M1R | 0 | 1 |
| KFB | 1 | 2 |
| K2tog, SSK | 2 | 1 |
| S2KP | 3 | 1 |
| CableLeft{h,c}, CableRight{h,c} | h+c | h+c |

The emitter uses these to track live stitch counts for repeat expansion. The graph builder uses them implicitly by consuming from and appending to the `live_stitches` vector.

## Supported stitch types and their topological effects

- **1→1** (Knit, Purl, Slip): One parent, one child. Slip passes the stitch through unworked.
- **0→1** (YarnOver, M1L, M1R): No parents — creates a new stitch from nothing (or from the bar between stitches).
- **1→2** (KFB): One parent, two children. Both children reference the same parent.
- **2→1** (K2tog, SSK): Two parents, one child. The child's `worked_through` contains both parent IDs.
- **3→1** (S2KP): Three parents, one child.
- **N→N cables** (CableLeft, CableRight): N parents, N children, but reordered. The cable operation stores the held and crossed StitchId groups to record which stitches swapped positions.
- **0→N** (CastOn): No parents, N children.
- **N→0** (BindOff): N parents, no children (removed from live set).

## Known limitations

- **Single panel**: PatternInstructions represents one panel. Multi-panel patterns require multiple PatternInstructions and separate StitchGraphs.
- **No short rows**: All rows are assumed to consume/produce across the full width. Partial rows are not modeled.
