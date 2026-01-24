# Yarn Path

Generate plausible 3D geometry from knitting instructions.

## Architecture

```
Knitting Instructions → StitchNode Graph → YarnPath → 3D Geometry
```

- **stitch_graph**: Fabric topology (stitches and their connections)
- **yarn_path**: Linear yarn sequence (how the yarn moves)
- **geometry**: Constraint-aware geometry assembly

## Building

```
cmake -B build
cmake --build build
```

## Running

```
./build/app/yarnpath
```

## Testing

```
cd build && ctest
```
