#!/usr/bin/env python3
"""
Analyze OBJ files for sharp turns in the yarn path geometry.

Usage:
    python3 scripts/analyze_obj.py <file.obj> [--threshold DEGREES]
"""

import sys
import math
import argparse


def read_vertices(filename):
    """Read vertex positions from an OBJ file."""
    vertices = []
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('v '):
                parts = line.split()
                vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
    return vertices


def angle_between(v1, v2):
    """Compute angle between two vectors in degrees."""
    dot = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]
    len1 = math.sqrt(v1[0]**2 + v1[1]**2 + v1[2]**2)
    len2 = math.sqrt(v2[0]**2 + v2[1]**2 + v2[2]**2)
    if len1 < 1e-9 or len2 < 1e-9:
        return 0
    cos_angle = max(-1, min(1, dot / (len1 * len2)))
    return math.degrees(math.acos(cos_angle))


def analyze_turns(vertices, threshold=45.0):
    """Find all sharp turns exceeding the threshold angle."""
    sharp_turns = []
    for i in range(1, len(vertices) - 1):
        prev = vertices[i-1]
        curr = vertices[i]
        next_v = vertices[i+1]

        # Direction vectors
        dir1 = (curr[0] - prev[0], curr[1] - prev[1], curr[2] - prev[2])
        dir2 = (next_v[0] - curr[0], next_v[1] - curr[1], next_v[2] - curr[2])

        angle = angle_between(dir1, dir2)
        if angle > threshold:
            sharp_turns.append((i, angle, curr))

    return sharp_turns


def main():
    parser = argparse.ArgumentParser(description='Analyze OBJ files for sharp turns')
    parser.add_argument('filename', help='OBJ file to analyze')
    parser.add_argument('--threshold', type=float, default=45.0,
                        help='Angle threshold in degrees (default: 45)')
    parser.add_argument('--top', type=int, default=20,
                        help='Number of worst turns to display (default: 20)')
    args = parser.parse_args()

    vertices = read_vertices(args.filename)
    print(f"Total vertices: {len(vertices)}")

    turns = analyze_turns(vertices, args.threshold)
    print(f"Sharp turns (> {args.threshold}°): {len(turns)}")

    if not turns:
        print("No sharp turns found!")
        return 0

    # Show worst turns
    turns.sort(key=lambda x: x[1], reverse=True)
    print(f"\nWorst {min(args.top, len(turns))} turns:")
    for i, (idx, angle, pos) in enumerate(turns[:args.top]):
        print(f"  {i+1}. Vertex {idx}: {angle:.1f}° at ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")

    return 1 if turns else 0


if __name__ == "__main__":
    sys.exit(main())
