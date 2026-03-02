#!/usr/bin/env python3
"""
Convert GeoJSON polygon to oracle input format.

Usage:
    ./convert_geojson.py <input.geojson> [--clip <clip.geojson>] > output.json

Input: GeoJSON Polygon or MultiPolygon
Output: Oracle format JSON with subject/clip arrays
"""

import json
import sys
import argparse


def geojson_to_oracle(geojson):
    """Convert a GeoJSON geometry to oracle polygon array format."""
    if isinstance(geojson, str):
        geojson = json.loads(geojson)

    geom_type = geojson.get("type")
    coords = geojson.get("coordinates", [])

    if geom_type == "Polygon":
        # Polygon: coordinates is array of rings
        # Convert to array of polygons (single polygon)
        return [coords]
    elif geom_type == "MultiPolygon":
        # MultiPolygon: coordinates is array of polygons
        return coords
    else:
        raise ValueError(f"Unsupported geometry type: {geom_type}")


def main():
    parser = argparse.ArgumentParser(description="Convert GeoJSON to oracle input format")
    parser.add_argument("subject", help="Subject GeoJSON file")
    parser.add_argument("--clip", help="Clip GeoJSON file (optional)")
    parser.add_argument("--output", "-o", help="Output file (default: stdout)")
    args = parser.parse_args()

    # Read subject
    with open(args.subject) as f:
        subject_geojson = json.load(f)

    result = {
        "subject": geojson_to_oracle(subject_geojson)
    }

    # Read clip if provided
    if args.clip:
        with open(args.clip) as f:
            clip_geojson = json.load(f)
        result["clip"] = geojson_to_oracle(clip_geojson)

    # Output
    output = json.dumps(result, indent=2)

    if args.output:
        with open(args.output, "w") as f:
            f.write(output)
    else:
        print(output)


if __name__ == "__main__":
    main()
