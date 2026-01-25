#include "surface.hpp"
#include "logging.hpp"
#include <limits>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <map>

namespace yarnpath {

RelaxedSurface RelaxedSurface::from_graph(const SurfaceGraph& graph) {
    RelaxedSurface surface;

    // Ensure positions vector is sized to hold all segments
    // Find the maximum segment ID first
    SegmentId max_seg_id = 0;
    for (const auto& node : graph.nodes()) {
        if (node.segment_id > max_seg_id) {
            max_seg_id = node.segment_id;
        }
    }

    surface.positions_.resize(max_seg_id + 1, vec3::zero());

    // Copy positions from graph nodes
    for (const auto& node : graph.nodes()) {
        surface.positions_[node.segment_id] = node.position;
    }

    return surface;
}

Vec3 RelaxedSurface::position(SegmentId id) const {
    if (id >= positions_.size()) {
        return vec3::zero();
    }
    return positions_[id];
}

std::pair<Vec3, Vec3> RelaxedSurface::bounding_box() const {
    if (positions_.empty()) {
        return {vec3::zero(), vec3::zero()};
    }

    Vec3 min_pt(std::numeric_limits<float>::max(),
                std::numeric_limits<float>::max(),
                std::numeric_limits<float>::max());
    Vec3 max_pt(std::numeric_limits<float>::lowest(),
                std::numeric_limits<float>::lowest(),
                std::numeric_limits<float>::lowest());

    for (const auto& pos : positions_) {
        min_pt.x = std::min(min_pt.x, pos.x);
        min_pt.y = std::min(min_pt.y, pos.y);
        min_pt.z = std::min(min_pt.z, pos.z);
        max_pt.x = std::max(max_pt.x, pos.x);
        max_pt.y = std::max(max_pt.y, pos.y);
        max_pt.z = std::max(max_pt.z, pos.z);
    }

    return {min_pt, max_pt};
}

Vec3 RelaxedSurface::center_of_mass() const {
    if (positions_.empty()) {
        return vec3::zero();
    }

    Vec3 sum = vec3::zero();
    for (const auto& pos : positions_) {
        sum += pos;
    }

    return sum / static_cast<float>(positions_.size());
}

RelaxedSurface relax_surface(
    const YarnPath& path,
    const YarnProperties& yarn,
    const Gauge& gauge,
    const SurfaceBuildConfig& build_config,
    const SolveConfig& solve_config) {

    auto log = yarnpath::logging::get_logger();

    // Build the graph
    SurfaceGraph graph = SurfaceBuilder::from_yarn_path(path, yarn, gauge, build_config);

    // Solve to equilibrium
    SolveResult result = SurfaceSolver::solve(graph, yarn, solve_config);

    log->info("relax_surface: {} after {} iterations, energy {} -> {}",
              result.converged ? "converged" : "did not converge",
              result.iterations,
              result.initial_energy,
              result.final_energy);

    // Extract the relaxed surface
    return RelaxedSurface::from_graph(graph);
}

// Helper: Generate icosahedron vertices and faces
namespace {

struct IcoSphere {
    std::vector<Vec3> vertices;
    std::vector<std::array<int, 3>> faces;

    void generate_icosahedron() {
        // Golden ratio
        const float phi = (1.0f + std::sqrt(5.0f)) / 2.0f;
        const float a = 1.0f;
        const float b = 1.0f / phi;

        // 12 vertices of icosahedron
        vertices = {
            Vec3(0, b, -a).normalized(),
            Vec3(b, a, 0).normalized(),
            Vec3(-b, a, 0).normalized(),
            Vec3(0, b, a).normalized(),
            Vec3(0, -b, a).normalized(),
            Vec3(-a, 0, b).normalized(),
            Vec3(0, -b, -a).normalized(),
            Vec3(a, 0, -b).normalized(),
            Vec3(a, 0, b).normalized(),
            Vec3(-a, 0, -b).normalized(),
            Vec3(b, -a, 0).normalized(),
            Vec3(-b, -a, 0).normalized()
        };

        // 20 faces of icosahedron
        faces = {{
            {2, 1, 0}, {1, 2, 3}, {5, 4, 3}, {4, 8, 3},
            {7, 6, 0}, {6, 9, 0}, {11, 10, 4}, {10, 11, 6},
            {9, 5, 2}, {5, 9, 11}, {8, 7, 1}, {7, 8, 10},
            {2, 5, 3}, {8, 1, 3}, {9, 2, 0}, {1, 7, 0},
            {11, 9, 6}, {7, 10, 6}, {5, 11, 4}, {10, 8, 4}
        }};
    }

    int get_middle_point(int p1, int p2, std::map<std::pair<int,int>, int>& cache) {
        auto key = std::make_pair(std::min(p1, p2), std::max(p1, p2));
        auto it = cache.find(key);
        if (it != cache.end()) {
            return it->second;
        }

        Vec3 middle = (vertices[p1] + vertices[p2]) * 0.5f;
        int idx = static_cast<int>(vertices.size());
        vertices.push_back(middle.normalized());
        cache[key] = idx;
        return idx;
    }

    void subdivide() {
        std::map<std::pair<int,int>, int> cache;
        std::vector<std::array<int, 3>> new_faces;

        for (const auto& face : faces) {
            int a = get_middle_point(face[0], face[1], cache);
            int b = get_middle_point(face[1], face[2], cache);
            int c = get_middle_point(face[2], face[0], cache);

            new_faces.push_back({face[0], a, c});
            new_faces.push_back({face[1], b, a});
            new_faces.push_back({face[2], c, b});
            new_faces.push_back({a, b, c});
        }

        faces = std::move(new_faces);
    }
};

} // anonymous namespace

std::string surface_to_obj(
    const SurfaceGraph& graph,
    const SurfaceObjConfig& config) {

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);

    ss << "# Surface Relaxation OBJ Export\n";
    ss << "# Nodes: " << graph.node_count() << "\n";
    ss << "# Edges: " << graph.edge_count() << "\n\n";

    // Generate base icosphere
    IcoSphere sphere;
    sphere.generate_icosahedron();
    for (int i = 0; i < config.sphere_subdivisions; ++i) {
        sphere.subdivide();
    }

    int vertex_offset = 1;  // OBJ indices are 1-based

    // Track vertex ranges for each node (for potential future use)
    std::vector<std::pair<int, int>> node_vertex_ranges;

    // Export spheres for each node
    for (const auto& node : graph.nodes()) {
        int start_vertex = vertex_offset;

        // Determine color based on node type
        // Loop nodes: blue (0.2, 0.4, 0.8)
        // Non-loop nodes: orange (0.8, 0.5, 0.2)
        Vec3 color = node.forms_loop ? Vec3(0.2f, 0.4f, 0.8f) : Vec3(0.8f, 0.5f, 0.2f);

        ss << "# Node " << node.id << " (segment " << node.segment_id;
        if (node.forms_loop) ss << ", loop";
        ss << ")\n";

        // Output vertices for this sphere
        for (const auto& v : sphere.vertices) {
            Vec3 pos = node.position + v * config.node_radius;
            ss << "v " << pos.x << " " << pos.y << " " << pos.z;
            if (config.color_by_type) {
                ss << " " << color.x << " " << color.y << " " << color.z;
            }
            ss << "\n";
        }

        // Output faces for this sphere
        for (const auto& face : sphere.faces) {
            ss << "f " << (start_vertex + face[0])
               << " " << (start_vertex + face[1])
               << " " << (start_vertex + face[2]) << "\n";
        }

        node_vertex_ranges.push_back({start_vertex, vertex_offset + static_cast<int>(sphere.vertices.size()) - 1});
        vertex_offset += static_cast<int>(sphere.vertices.size());

        ss << "\n";
    }

    // Export edges as lines
    // We add vertices with colors for each edge endpoint
    ss << "# Edge lines\n";

    // Continuity edge color: bright green (0.2, 0.9, 0.2)
    Vec3 continuity_color(0.2f, 0.9f, 0.2f);
    // Passthrough edge color: bright red (0.9, 0.2, 0.2)
    Vec3 passthrough_color(0.9f, 0.2f, 0.2f);

    // Output continuity edges with colored vertices
    if (config.show_continuity) {
        ss << "# Continuity edges (yarn backbone) - green\n";
        for (const auto& edge : graph.edges()) {
            if (edge.type == EdgeType::YarnContinuity) {
                const auto& pos_a = graph.node(edge.node_a).position;
                const auto& pos_b = graph.node(edge.node_b).position;

                // Add two vertices for this edge
                ss << "v " << pos_a.x << " " << pos_a.y << " " << pos_a.z;
                if (config.color_by_type) {
                    ss << " " << continuity_color.x << " " << continuity_color.y << " " << continuity_color.z;
                }
                ss << "\n";

                ss << "v " << pos_b.x << " " << pos_b.y << " " << pos_b.z;
                if (config.color_by_type) {
                    ss << " " << continuity_color.x << " " << continuity_color.y << " " << continuity_color.z;
                }
                ss << "\n";

                // Line between the two new vertices
                ss << "l " << vertex_offset << " " << (vertex_offset + 1) << "\n";
                vertex_offset += 2;
            }
        }
        ss << "\n";
    }

    // Output passthrough edges with colored vertices
    if (config.show_passthrough) {
        ss << "# Passthrough edges (through-loop connections) - red\n";
        for (const auto& edge : graph.edges()) {
            if (edge.type == EdgeType::PassThrough) {
                const auto& pos_a = graph.node(edge.node_a).position;
                const auto& pos_b = graph.node(edge.node_b).position;

                // Add two vertices for this edge
                ss << "v " << pos_a.x << " " << pos_a.y << " " << pos_a.z;
                if (config.color_by_type) {
                    ss << " " << passthrough_color.x << " " << passthrough_color.y << " " << passthrough_color.z;
                }
                ss << "\n";

                ss << "v " << pos_b.x << " " << pos_b.y << " " << pos_b.z;
                if (config.color_by_type) {
                    ss << " " << passthrough_color.x << " " << passthrough_color.y << " " << passthrough_color.z;
                }
                ss << "\n";

                // Line between the two new vertices
                ss << "l " << vertex_offset << " " << (vertex_offset + 1) << "\n";
                vertex_offset += 2;
            }
        }
        ss << "\n";
    }

    return ss.str();
}

}  // namespace yarnpath
