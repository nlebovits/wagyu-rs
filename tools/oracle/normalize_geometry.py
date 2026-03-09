#!/usr/bin/env python3
"""
Normalize polygon geometry for comparison.

Handles:
- Ring rotation (different starting vertices)
- Ring ordering within polygons
- Polygon ordering within multi-polygons
"""

import json
import sys
from typing import List, Tuple


def normalize_ring(ring: List[List[int]]) -> List[List[int]]:
    """Normalize a ring by rotating to start at the lexicographically smallest point."""
    if not ring or len(ring) < 2:
        return ring

    # Remove closing point if present (we'll add it back)
    if ring[0] == ring[-1]:
        ring = ring[:-1]

    if not ring:
        return ring

    # Find the lexicographically smallest point
    min_idx = 0
    for i in range(1, len(ring)):
        if ring[i] < ring[min_idx]:
            min_idx = i

    # Rotate ring to start at min point
    rotated = ring[min_idx:] + ring[:min_idx]

    # Add closing point back
    rotated.append(rotated[0].copy())

    return rotated


def ring_to_tuple(ring: List[List[int]]) -> Tuple:
    """Convert ring to hashable tuple for sorting."""
    return tuple(tuple(pt) for pt in ring)


def normalize_polygon(polygon: List[List[List[int]]]) -> List[List[List[int]]]:
    """Normalize a polygon (exterior + holes)."""
    if not polygon:
        return polygon

    # Normalize each ring
    normalized_rings = [normalize_ring(ring) for ring in polygon]

    # Keep exterior first, sort holes
    exterior = normalized_rings[0] if normalized_rings else []
    holes = sorted(normalized_rings[1:], key=ring_to_tuple) if len(normalized_rings) > 1 else []

    return [exterior] + holes


def polygon_to_tuple(polygon: List[List[List[int]]]) -> Tuple:
    """Convert polygon to hashable tuple for sorting."""
    return tuple(ring_to_tuple(ring) for ring in polygon)


def normalize_multipolygon(mp: List[List[List[List[int]]]]) -> List[List[List[List[int]]]]:
    """Normalize a multi-polygon."""
    if not mp:
        return mp

    # Normalize each polygon
    normalized = [normalize_polygon(poly) for poly in mp]

    # Sort polygons for consistent ordering
    normalized.sort(key=polygon_to_tuple)

    return normalized


def normalize_geometry(geom: List) -> List:
    """Normalize geometry output (list of multi-polygons or single multi-polygon)."""
    if not geom:
        return geom

    # Detect structure: is this a list of multi-polygons or a single one?
    # A multi-polygon has structure: [[[point, point, ...], ...], ...]
    # The oracle outputs a single multi-polygon

    return normalize_multipolygon(geom)


def main():
    if len(sys.argv) < 2:
        print("Usage: normalize_geometry.py <json_file> or pipe JSON to stdin", file=sys.stderr)
        sys.exit(1)

    if sys.argv[1] == "-":
        data = json.load(sys.stdin)
    else:
        with open(sys.argv[1]) as f:
            data = json.load(f)

    normalized = normalize_geometry(data)
    print(json.dumps(normalized, separators=(',', ':')))


if __name__ == "__main__":
    main()
