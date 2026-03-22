#ifndef YARNPATH_SURFACE_SOLVER_HPP
#define YARNPATH_SURFACE_SOLVER_HPP

#include "surface_graph.hpp"
#include "surface_forces.hpp"
#include <yarn/yarn_properties.hpp>
#include <functional>

namespace yarnpath {

// Callback for each solver step - returns true to continue, false to stop
// Called with (graph, iteration_number, current_energy, energy_change)
using StepCallback = std::function<bool(const SurfaceGraph&, int, float, float)>;

// Callback function for frame capture during solving
// Called with (graph, iteration_number, total_iterations)
using FrameCallback = std::function<void(const SurfaceGraph&, int, int)>;

// Configuration for the solver
struct SolveConfig {
    // Time step for integration
    float dt = 0.01f;

    // Maximum number of iterations
    int max_iterations = 1000;

    // Convergence threshold (energy change per iteration)
    float convergence_threshold = 1e-4f;

    // Maximum displacement any node can move per step (mm)
    // Used for global normalization: if any node exceeds this, ALL scale down equally
    float max_displacement_per_step = 0.5f;

    // Force configuration
    ForceConfig force_config;

    // Parallelization settings
    int num_threads = 0;  // 0 = auto-detect, > 0 = use specific count
};

// Result of solving
struct SolveResult {
    bool converged = false;
    int iterations = 0;
    float final_energy = 0.0f;
    float initial_energy = 0.0f;
};

// Physics solver for the surface relaxation graph
class SurfaceSolver {
public:
    // Solve the graph to find equilibrium positions
    static SolveResult solve(SurfaceGraph& graph,
                             const YarnProperties& yarn,
                             const Gauge& gauge,
                             const SolveConfig& config = SolveConfig{},
                             const StepCallback& step_callback = nullptr);

    // Single step of the solver (for debugging or interactive use)
    static void step(SurfaceGraph& graph,
                     const YarnProperties& yarn,
                     const Gauge& gauge,
                     const SolveConfig& config,
                     const std::vector<std::vector<NodeId>>& collision_skip_list = {});

private:
    // Gradient descent step with global displacement normalization
    static void integrate_gradient_step(SurfaceGraph& graph, float dt,
                                         float max_displacement);

    // Compute fabric normals for all nodes after solve completes.
    // Uses stitch_axis and continuity neighbors to determine the fabric surface normal.
    static void compute_fabric_normals(SurfaceGraph& graph);
};

}  // namespace yarnpath

#endif // YARNPATH_SURFACE_SOLVER_HPP
