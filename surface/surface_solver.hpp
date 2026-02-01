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

    // Number of constraint projection iterations per step
    int constraint_iterations = 30;  // Increased for better constraint satisfaction

    // Number of constraint-only pre-solve iterations (before force relaxation)
    // This helps satisfy constraints before springs start pulling
    int pre_solve_iterations = 1000;

    // Force configuration
    ForceConfig force_config;

    // Step callback (optional) - called after each solver step
    // Returns true to continue, false to stop early
    StepCallback step_callback = nullptr;

    // Frame capture callback (optional) - called every frame_interval iterations
    FrameCallback frame_callback = nullptr;
    int frame_interval = 100;  // Capture a frame every N iterations

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
                             const SolveConfig& config = SolveConfig{});

    // Single step of the solver (for debugging or interactive use)
    static void step(SurfaceGraph& graph,
                     const YarnProperties& yarn,
                     const Gauge& gauge,
                     const SolveConfig& config);

private:
    // Verlet integration step
    static void integrate_verlet(SurfaceGraph& graph, float dt);

    // Project all constraints
    static void project_constraints(SurfaceGraph& graph, int iterations);

    // Project a single constraint
    static void project_constraint(SurfaceGraph& graph,
                                   const SurfaceConstraint& constraint);

    // Check if all constraints are satisfied
    static bool constraints_satisfied(const SurfaceGraph& graph, float tolerance = 1e-4f);
};

}  // namespace yarnpath

#endif // YARNPATH_SURFACE_SOLVER_HPP
