// fabrik_solver.hpp - Main FABRIK solver that orchestrates backward/forward iterations

#ifndef DELTA_FABRIK_SOLVER_HPP
#define DELTA_FABRIK_SOLVER_HPP

#include "fabrik_backward.hpp"
#include "fabrik_forward.hpp"
#include "fabrik_initialization.hpp"

namespace delta {

// Data for each physical segment's end-effector
struct SegmentEndEffectorData {
    int segment_number;                    // Which physical segment (1, 2, 3, etc.)
    Vector3 end_effector_position;         // 3D position of this segment's end-effector
    Vector3 direction_from_base;           // Direction vector from world origin to this end-effector
    double prismatic_length;               // This segment's prismatic length
    double h_to_g_distance;                // This segment's H→G distance
    double fabrik_distance_from_base;      // Distance along FABRIK chain from base to this end-effector
    
    SegmentEndEffectorData(int seg_num, const Vector3& pos, const Vector3& dir, 
                          double prismatic, double h_to_g, double fabrik_dist)
        : segment_number(seg_num), end_effector_position(pos), direction_from_base(dir)
        , prismatic_length(prismatic), h_to_g_distance(h_to_g), fabrik_distance_from_base(fabrik_dist) {}
};

// Overall FABRIK solution result
struct FabrikSolutionResult {
    FabrikChain final_chain;                    // Final solved chain
    Vector3 target_position;                    // Target that was solved for
    Vector3 achieved_position;                  // Actual end-effector position achieved
    bool converged;                             // Whether algorithm converged
    double final_error;                         // Distance between target and achieved
    int total_iterations;                       // Total iterations (backward + forward)
    int backward_iterations;                    // Number of backward iterations
    int forward_iterations;                     // Number of forward iterations
    std::vector<Vector3> convergence_history;   // End-effector position each iteration
    double solve_time_ms;                       // Time taken to solve
    std::vector<SegmentEndEffectorData> segment_end_effectors;  // NEW! Physical segment end-effectors
    
    FabrikSolutionResult(const FabrikChain& chain, const Vector3& target, const Vector3& achieved,
                        bool conv, double error, int total_iter)
        : final_chain(chain), target_position(target), achieved_position(achieved)
        , converged(conv), final_error(error), total_iterations(total_iter)
        , backward_iterations(0), forward_iterations(0), solve_time_ms(0.0) {}
};

// FABRIK solver configuration
struct FabrikSolverConfig {
    double tolerance;                           // Convergence tolerance
    int max_iterations;                         // Maximum total iterations
    int max_backward_forward_cycles;            // Max backward-forward cycles
    bool enable_constraints;                    // Enable/disable joint constraints
    bool track_convergence_history;             // Store position history
    bool verbose_logging;                       // Enable debug output
    
    // Default configuration using constants
    FabrikSolverConfig()
        : tolerance(FABRIK_TOLERANCE)                        // 0.01 from constants.hpp
        , max_iterations(FABRIK_MAX_ITERATIONS)              // 100 from constants.hpp  
        , max_backward_forward_cycles(FABRIK_MAX_ITERATIONS / 2)  // 50 (half of max iterations)
        , enable_constraints(true)
        , track_convergence_history(false)
        , verbose_logging(false) {}
};

class FabrikSolver {
public:
    // Main solving interface
    static FabrikSolutionResult solve_to_target(const FabrikChain& initial_chain,
                                               const Vector3& target_position,
                                               const FabrikSolverConfig& config = FabrikSolverConfig());
    
    // Convenience method with default config
    static FabrikSolutionResult solve(const FabrikChain& initial_chain,
                                    const Vector3& target_position,
                                    double tolerance = FABRIK_TOLERANCE,
                                    int max_iterations = FABRIK_MAX_ITERATIONS);
    
    // Single backward-forward cycle
    static FabrikChain single_fabrik_cycle(const FabrikChain& chain,
                                          const Vector3& target_position,
                                          const FabrikSolverConfig& config);
    
    // Validation methods
    static bool is_solution_valid(const FabrikChain& chain, double tolerance = 0.01);
    static double calculate_chain_error(const FabrikChain& chain);
    static Vector3 get_end_effector_position(const FabrikChain& chain);
    
    // Utility methods
    static FabrikSolverConfig create_fast_config();     // Fast solving (loose tolerance)
    static FabrikSolverConfig create_precise_config();  // Precise solving (tight tolerance)
    static FabrikSolverConfig create_debug_config();    // Debug mode (verbose logging)
    
private:
    // Core algorithm implementation
    static FabrikSolutionResult run_fabrik_algorithm(const FabrikChain& initial_chain,
                                                    const Vector3& target_position,
                                                    const FabrikSolverConfig& config);
    
    // Segment length management - CRITICAL for preventing oscillation
    static void update_segment_lengths(FabrikChain& chain, const std::vector<double>& new_lengths);
    
    // NEW! Extract segment end-effector positions from solved chain
    static std::vector<SegmentEndEffectorData> extract_segment_end_effectors(const FabrikChain& solved_chain);
    
    // Convergence checking
    static bool has_converged(const Vector3& end_effector, const Vector3& target, double tolerance);
    static bool has_stalled(const std::vector<Vector3>& history, int min_history = 5);
    
    // Logging and debugging
    static void log_iteration(int iteration, const Vector3& end_effector, 
                            const Vector3& target, double error, bool verbose);
};

// High-level convenience functions for common use cases
namespace fabrik_utils {
    
    // Solve for delta robot with N segments
    FabrikSolutionResult solve_delta_robot(int num_segments, 
                                         const Vector3& target,
                                         double tolerance = FABRIK_TOLERANCE);
    
    // Batch solve multiple targets
    std::vector<FabrikSolutionResult> solve_multiple_targets(const FabrikChain& initial_chain,
                                                           const std::vector<Vector3>& targets,
                                                           const FabrikSolverConfig& config = FabrikSolverConfig());
    
    // Find workspace boundary (maximum reach in given direction)
    Vector3 find_max_reach(const FabrikChain& chain, const Vector3& direction);
    
    // Validate if target is reachable
    bool is_target_reachable(const FabrikChain& chain, const Vector3& target);
}

} // namespace delta

#endif // DELTA_FABRIK_SOLVER_HPP