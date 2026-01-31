# YarnPath

Generate plausible 3D geometry from knitting instructions.

## Overview

YarnPath converts knitting patterns written in Shirley Paden notation into 3D geometry suitable for visualization or 3D printing. The pipeline processes patterns through multiple stages:

```
Text Pattern → Instructions → Stitch Graph → Yarn Path → Surface Graph → Geometry → OBJ
```

### Pipeline Stages

1. **Parse**: Tokenize and parse pattern text into structured instructions
2. **Stitch**: Build topology graph of stitches and their relationships
3. **Yarn**: Trace continuous yarn path through the stitch graph
4. **Surface**: Physics-based relaxation to find equilibrium positions
5. **Geometry**: Generate 3D Bezier curves representing yarn
6. **OBJ**: Export final geometry to Wavefront OBJ format

Each stage saves its output as JSON, enabling inspection, caching, and step-by-step execution.

## Building

### Prerequisites

- CMake 3.20+
- C++20 compiler
- OpenMP (optional, for parallel processing)
- GLFW + OpenGL (optional, for visualization)

### Build Steps

```bash
cmake -B build
cmake --build build
```

The executable will be at `build/app/yarnpath`.

## Usage

### Quick Start with Makefile Generation

The easiest way to process a pattern is to generate a Makefile:

```bash
# Generate Makefile for a pattern
./build/app/yarnpath generate-makefile examples/simple-pattern-2.txt -C build/pattern

# Or with a custom configuration
./build/app/yarnpath generate-makefile examples/simple-pattern-2.txt -C build/pattern -c config.json

# Build the entire pipeline
cd build/pattern
make

# Or build to a specific stage
make surface     # Stop after surface relaxation
make geometry    # Stop after geometry generation
```

### Manual Step-by-Step Execution

Run each pipeline stage individually:

```bash
# Parse pattern text to JSON
./yarnpath parse pattern.txt -o pattern.instructions.json

# Build stitch graph
./yarnpath stitch pattern.instructions.json -o pattern.stitch.json

# Build yarn path
./yarnpath yarn pattern.stitch.json -o pattern.yarn.json

# Run surface relaxation
./yarnpath surface pattern.yarn.json -o pattern.surface.json

# Generate geometry
./yarnpath geometry pattern.surface.json -o pattern.geometry.json

# Export to OBJ
./yarnpath obj pattern.geometry.json -o pattern.obj
```

### Configuration Options

The surface command accepts custom configuration:

```bash
# With custom config file
./yarnpath surface pattern.yarn.json -o pattern.surface.json -c config.json

# With command-line overrides
./yarnpath surface pattern.yarn.json -o pattern.surface.json --iterations 50000 --threshold 1e-5
```

Example `config.json`:

```json
{
  "yarn": {
    "radius": 1.0,
    "min_bend_radius": 3.0,
    "stiffness": 0.5
  },
  "gauge": {
    "stitches_per_unit": 4.0,
    "rows_per_unit": 5.0,
    "needle_diameter": 5.0
  }
}
```

### Interactive Visualization

Visualize surface relaxation or geometry in real-time:

```bash
# Watch surface relaxation
./yarnpath surface pattern.yarn.json --visualize

# View final geometry with surface
./yarnpath geometry pattern.surface.json --visualize

# Save result after visualization
./yarnpath surface pattern.yarn.json --visualize -o pattern.surface.json
```

**Controls:**
- `Space` - Pause/resume
- `R` - Reset
- `N` - Toggle node display
- `G` - Toggle geometry display
- `Q` - Quit

## JSON Format

Each intermediate file contains:

```json
{
  "version": "0.1.0",
  "step": "surface_graph",
  "timestamp": "2026-01-31T17:31:43Z",
  "source_file": "pattern.txt",
  "config": {
    "yarn": {...},
    "gauge": {...}
  },
  "stats": {
    "node_count": 294,
    "edge_count": 566
  },
  "data": {...}
}
```

This format enables:
- **Inspection**: View intermediate data at each stage
- **Caching**: Skip expensive recomputation
- **Debugging**: Isolate issues to specific pipeline stages
- **Experimentation**: Modify parameters without re-parsing

## Testing

Run the test suite:

```bash
cd build
ctest
```

## Command Reference

### Commands

- `generate-makefile` - Create Makefile for pipeline
- `parse` - Convert pattern text to instructions JSON
- `stitch` - Build stitch graph from instructions JSON
- `yarn` - Build yarn path topology from stitch graph JSON
- `surface` - Build and relax surface from yarn path JSON
- `geometry` - Build 3D geometry from surface JSON
- `obj` - Export geometry JSON to OBJ format

### Global Options

- `-o, --output FILE` - Output file (required for most commands)
- `-v, --verbose` - Enable verbose logging
- `-h, --help` - Show help message

### Generate-Makefile Options

- `-C <directory>` - Target directory for Makefile and outputs (required)
- `-c, --config FILE` - Configuration file to use for surface step
- `--executable <path>` - Path to yarnpath executable (default: yarnpath in PATH)

### Surface Options

- `-c, --config FILE` - Load configuration from FILE
- `--iterations N` - Max solver iterations (default: 100000)
- `--threshold N` - Convergence threshold (default: 1e-6)
- `--visualize` - Open interactive visualization

### Geometry Options

- `--visualize` - Open interactive visualization

## Examples

See the `examples/` directory for sample patterns:

- `simple-pattern.txt` - Basic stockinette sample
- `simple-pattern-2.txt` - Pattern with increases/decreases
- `stockinette-3x3.txt` - Small 3×3 stockinette swatch

## Architecture

### Core Libraries

- `parser/` - Tokenizer, parser, AST, instruction emitter
- `stitch_graph/` - Stitch topology and operations
- `yarn_path/` - Yarn tracing through stitch graph
- `surface/` - Physics-based relaxation solver
- `geometry/` - 3D curve generation with Bezier splines
- `visualizer/` - OpenGL interactive visualization

### New in v0.2

- `serialization/` - JSON serialization for all pipeline stages
- `cli/` - Modular CLI commands with subcommand architecture

## License

[Your license here]
