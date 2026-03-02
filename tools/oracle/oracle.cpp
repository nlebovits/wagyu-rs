/**
 * wagyu-oracle: CLI tool for running C++ wagyu as a test oracle
 *
 * This tool reads a JSON file containing subject and clip polygons,
 * runs the specified boolean operation, and outputs the result as JSON.
 *
 * Usage: wagyu-oracle <input.json> <operation> [fill_type] [--debug]
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
 * Debug mode: --debug outputs structured logging to stderr in the same
 * format as the Rust implementation for diffing.
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

// Global debug flag
bool g_debug = false;

// Debug logging functions (match Rust format)
void log_vatti_start(size_t minima, size_t scanbeam) {
    if (g_debug) {
        std::cerr << "[VATTI_START] minima=" << minima << " scanbeam=" << scanbeam << std::endl;
    }
}

void log_vatti_end(size_t rings) {
    if (g_debug) {
        std::cerr << "[VATTI_END] rings=" << rings << std::endl;
    }
}

void log_input_polygon(const char* type, size_t poly_idx, size_t ring_count, size_t point_count) {
    if (g_debug) {
        std::cerr << "[INPUT] type=" << type << " poly=" << poly_idx
                  << " rings=" << ring_count << " points=" << point_count << std::endl;
    }
}

void log_output_polygon(size_t poly_idx, size_t ring_count) {
    if (g_debug) {
        std::cerr << "[OUTPUT] poly=" << poly_idx << " rings=" << ring_count << std::endl;
    }
}

void log_ring_points(size_t ring_idx, size_t point_count) {
    if (g_debug) {
        std::cerr << "[RING_CLOSE] id=" << ring_idx << " points=" << point_count << std::endl;
    }
}

void print_usage(const char* program) {
    std::cerr << "Usage: " << program << " <input.json> <operation> [fill_type] [--debug]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Operations: union, intersection, difference, xor" << std::endl;
    std::cerr << "Fill types: evenodd (default), nonzero, positive, negative" << std::endl;
    std::cerr << "Flags: --debug outputs structured logging to stderr" << std::endl;
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

    // Parse optional arguments
    for (int i = 3; i < argc; ++i) {
        if (strcmp(argv[i], "--debug") == 0) {
            g_debug = true;
        } else {
            fill = parse_fill_type(argv[i]);
        }
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

    // Log input polygons
    for (size_t i = 0; i < subject_polys.size(); ++i) {
        size_t point_count = 0;
        for (const auto& ring : subject_polys[i]) {
            point_count += ring.size();
        }
        log_input_polygon("Subject", i, subject_polys[i].size(), point_count);
    }
    for (size_t i = 0; i < clip_polys.size(); ++i) {
        size_t point_count = 0;
        for (const auto& ring : clip_polys[i]) {
            point_count += ring.size();
        }
        log_input_polygon("Clip", i, clip_polys[i].size(), point_count);
    }

    // Run wagyu
    wagyu<T> clipper;

    for (const auto& poly : subject_polys) {
        clipper.add_polygon(poly, polygon_type_subject);
    }

    for (const auto& poly : clip_polys) {
        clipper.add_polygon(poly, polygon_type_clip);
    }

    // Log algorithm start
    log_vatti_start(subject_polys.size() + clip_polys.size(), 0);

    mapbox::geometry::multi_polygon<T> solution;
    clipper.execute(operation, solution, fill, fill);

    // Log algorithm end and output rings
    size_t ring_idx = 0;
    for (size_t i = 0; i < solution.size(); ++i) {
        log_output_polygon(i, solution[i].size());
        for (const auto& ring : solution[i]) {
            log_ring_points(ring_idx++, ring.size());
        }
    }
    log_vatti_end(ring_idx);

    // Output result
    output_result(solution);

    return 0;
}
