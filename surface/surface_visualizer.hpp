#ifndef YARNPATH_SURFACE_VISUALIZER_HPP
#define YARNPATH_SURFACE_VISUALIZER_HPP

#include "surface_graph.hpp"
#include "surface_solver.hpp"
#include <yarn/yarn_properties.hpp>
#include <functional>
#include <string>

namespace yarnpath {

// Configuration for the visualizer
struct VisualizerConfig {
    int window_width = 800;
    int window_height = 600;
    std::string window_title = "YarnPath Surface Relaxation";

    // Rendering options
    float node_size = 0.1f;           // Size of node spheres
    bool show_continuity = true;      // Show continuity edges
    bool show_passthrough = true;     // Show passthrough edges
    bool color_by_type = true;        // Color nodes by type

    // Camera
    float camera_distance = 20.0f;    // Initial camera distance
    float rotation_speed = 0.5f;      // Mouse rotation speed
    float zoom_speed = 1.1f;          // Scroll zoom factor
    bool auto_fit = true;             // Auto-fit camera to show all nodes each frame

    // Simulation
    int steps_per_frame = 10;         // Solver steps per rendered frame
    bool auto_run = true;             // Start running immediately

    // History/playback
    int snapshot_interval = 1;        // Store snapshot every N frames
    int max_snapshots = 100000;       // Maximum snapshots to keep in memory
};

// Result of visualization session
struct VisualizerResult {
    bool completed = false;           // User closed window normally
    int total_iterations = 0;         // Total solver iterations
    float final_energy = 0.0f;        // Final system energy
};

// Run the interactive visualizer
// Returns when window is closed
VisualizerResult visualize_relaxation(
    SurfaceGraph& graph,
    const YarnProperties& yarn,
    const SolveConfig& solve_config,
    const VisualizerConfig& viz_config = VisualizerConfig{}
);

// Check if visualization is available (GLFW/OpenGL compiled in)
bool visualization_available();

}  // namespace yarnpath

#endif // YARNPATH_SURFACE_VISUALIZER_HPP
