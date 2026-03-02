/**
 * wagyu-oracle: CLI tool for running C++ wagyu as a test oracle
 *
 * This tool reads a JSON file containing subject and clip polygons,
 * runs the specified boolean operation, and outputs the result as JSON.
 *
 * Usage: wagyu-oracle <input.json> <operation> [fill_type]
 *
 * Input format:
 * {
 *   "subject": [[[[x,y], [x,y], ...]]], // MultiPolygon: array of polygons, each polygon is array of rings
 *   "clip": [[[[x,y], [x,y], ...]]]     // Optional
 * }
 *
 * Operations: union, intersection, difference, xor
 * Fill types: evenodd (default), nonzero, positive, negative
 *
 * Build: See build_oracle.sh
 */

#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include <mapbox/geometry/polygon.hpp>
#include <mapbox/geometry/wagyu/wagyu.hpp>

#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

using namespace rapidjson;
using namespace mapbox::geometry::wagyu;
using T = std::int64_t;

void print_usage(const char* program) {
    std::cerr << "Usage: " << program << " <input.json> <operation> [fill_type]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Operations: union, intersection, difference, xor" << std::endl;
    std::cerr << "Fill types: evenodd (default), nonzero, positive, negative" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Input JSON format:" << std::endl;
    std::cerr << "  {" << std::endl;
    std::cerr << "    \"subject\": [[[[x,y], ...]]]," << std::endl;
    std::cerr << "    \"clip\": [[[[x,y], ...]]]  // optional" << std::endl;
    std::cerr << "  }" << std::endl;
}

clip_type parse_operation(const char* op) {
    if (strcmp(op, "union") == 0) return clip_type_union;
    if (strcmp(op, "intersection") == 0) return clip_type_intersection;
    if (strcmp(op, "difference") == 0) return clip_type_difference;
    if (strcmp(op, "xor") == 0) return clip_type_x_or;

    std::cerr << "Unknown operation: " << op << std::endl;
    std::exit(1);
}

fill_type parse_fill_type(const char* ft) {
    if (strcmp(ft, "evenodd") == 0) return fill_type_even_odd;
    if (strcmp(ft, "nonzero") == 0) return fill_type_non_zero;
    if (strcmp(ft, "positive") == 0) return fill_type_positive;
    if (strcmp(ft, "negative") == 0) return fill_type_negative;

    std::cerr << "Unknown fill type: " << ft << std::endl;
    std::exit(1);
}

// Parse a ring from JSON array of [x,y] pairs
mapbox::geometry::linear_ring<T> parse_ring(const Value& ring_json) {
    mapbox::geometry::linear_ring<T> ring;
    for (SizeType i = 0; i < ring_json.Size(); ++i) {
        const Value& pt = ring_json[i];
        T x = static_cast<T>(pt[0].GetDouble());
        T y = static_cast<T>(pt[1].GetDouble());
        ring.push_back({x, y});
    }
    return ring;
}

// Parse a polygon from JSON array of rings
mapbox::geometry::polygon<T> parse_polygon(const Value& poly_json) {
    mapbox::geometry::polygon<T> poly;
    for (SizeType i = 0; i < poly_json.Size(); ++i) {
        poly.push_back(parse_ring(poly_json[i]));
    }
    return poly;
}

// Parse multi-polygon from JSON array of polygons
std::vector<mapbox::geometry::polygon<T>> parse_multi_polygon(const Value& mp_json) {
    std::vector<mapbox::geometry::polygon<T>> polys;
    for (SizeType i = 0; i < mp_json.Size(); ++i) {
        polys.push_back(parse_polygon(mp_json[i]));
    }
    return polys;
}

// Output multi-polygon as JSON
void output_result(const mapbox::geometry::multi_polygon<T>& result) {
    StringBuffer buffer;
    Writer<StringBuffer> writer(buffer);

    writer.StartArray();
    for (const auto& poly : result) {
        writer.StartArray();
        for (const auto& ring : poly) {
            writer.StartArray();
            for (const auto& pt : ring) {
                writer.StartArray();
                writer.Int64(pt.x);
                writer.Int64(pt.y);
                writer.EndArray();
            }
            writer.EndArray();
        }
        writer.EndArray();
    }
    writer.EndArray();

    std::cout << buffer.GetString() << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        print_usage(argv[0]);
        return 1;
    }

    const char* input_file = argv[1];
    clip_type operation = parse_operation(argv[2]);
    fill_type fill = fill_type_even_odd;

    if (argc > 3) {
        fill = parse_fill_type(argv[3]);
    }

    // Read input file
    FILE* file = fopen(input_file, "r");
    if (!file) {
        std::cerr << "Cannot open file: " << input_file << std::endl;
        return 1;
    }

    char buffer[65536];
    FileReadStream stream(file, buffer, sizeof(buffer));

    Document doc;
    doc.ParseStream(stream);
    fclose(file);

    if (doc.HasParseError()) {
        std::cerr << "JSON parse error at offset " << doc.GetErrorOffset() << std::endl;
        return 1;
    }

    // Parse subject polygons
    if (!doc.HasMember("subject") || !doc["subject"].IsArray()) {
        std::cerr << "Input must have 'subject' array" << std::endl;
        return 1;
    }

    auto subject_polys = parse_multi_polygon(doc["subject"]);

    // Parse clip polygons (optional)
    std::vector<mapbox::geometry::polygon<T>> clip_polys;
    if (doc.HasMember("clip") && doc["clip"].IsArray()) {
        clip_polys = parse_multi_polygon(doc["clip"]);
    }

    // Run wagyu
    wagyu<T> clipper;

    for (const auto& poly : subject_polys) {
        clipper.add_polygon(poly, polygon_type_subject);
    }

    for (const auto& poly : clip_polys) {
        clipper.add_polygon(poly, polygon_type_clip);
    }

    mapbox::geometry::multi_polygon<T> solution;
    clipper.execute(operation, solution, fill, fill);

    // Output result
    output_result(solution);

    return 0;
}
