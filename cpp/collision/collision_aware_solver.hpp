#ifndef DELTA_COLLISION_AWARE_SOLVER_HPP
#define DELTA_COLLISION_AWARE_SOLVER_HPP

#include "../core/math_utils.hpp"
#include "../fabrik/fabrik_solver.hpp"
#include "u_points_extractor.hpp"
#include "collision_detector.hpp"
#include "waypoint_converter.hpp"
#include <vector>

namespace delta {

// Result from collision-aware solving
struct CollisionAwareSolutionResult {
    FabrikSolutionResult fabrik_result;         // Final FABRIK solution
    bool collision_free;                        // Whether final solution is collision-free
    int collision_iterations;                   // Number of collision avoidance iterations
    double total_collision_time_ms;             // Time spent on collision detection
    std::vector<CollisionResult> collision_history; // History of collision detection results
    bool conversion_successful;                 // Whether waypoint conversion worked
    
    CollisionAwareSolutionResult(const FabrikSolutionResult& fabrik_res, bool collision_free_flag)
        : fabrik_result(fabrik_res), collision_free(collision_free_flag)
        , collision_iterations(0), total_collision_time_ms(0.0), conversion_successful(true) {}
};

// Configuration for collision-aware solving
struct CollisionAwareConfig {
    int max_collision_iterations;               // Maximum collision avoidance iterations (default: 3)
    double spline_diameter;                     // Robot spline diameter for collision detection
    bool enable_collision_detection;            // Enable/disable collision detection
    bool verbose_logging;                       // Enable debug output
    FabrikSolverConfig fabrik_config;           // FABRIK solver configuration
    
    CollisionAwareConfig()
        : max_collision_iterations(3)
        , spline_diameter(DEFAULT_SPLINE_DIAMETER)
        , enable_collision_detection(true)
        , verbose_logging(false) {}
};

// Main collision-aware solver that orchestrates all collision detection phases
class CollisionAwareSolver {
public:
    // Main interface: solve with collision avoidance
    static CollisionAwareSolutionResult solve_with_collision_avoidance(
        const Vector3& target_position,
        const std::vector<Obstacle>& obstacles,
        const CollisionAwareConfig& config = CollisionAwareConfig());
    
    // Solve with initial joint positions (for motor module integration)
    static CollisionAwareSolutionResult solve_with_collision_avoidance(
        const Vector3& target_position,
        const std::vector<Obstacle>& obstacles,
        const std::vector<Vector3>& initial_joint_positions,
        int num_robot_segments,
        const CollisionAwareConfig& config = CollisionAwareConfig());
    
    // Convenience method with simple parameters
    static CollisionAwareSolutionResult solve(
        const Vector3& target_position,
        const std::vector<Obstacle>& obstacles,
        int max_iterations = 3);
    
    // Check if a FABRIK solution has collisions
    static bool has_collision(const FabrikChain& chain, 
                             const std::vector<Obstacle>& obstacles,
                             double spline_diameter = DEFAULT_SPLINE_DIAMETER);

private:
    // Core iteration loop implementation
    static CollisionAwareSolutionResult run_collision_aware_algorithm(
        const FabrikChain& initial_chain,
        const Vector3& target_position,
        const std::vector<Obstacle>& obstacles,
        const CollisionAwareConfig& config);
    
    // Single collision avoidance iteration
    static std::pair<FabrikChain, CollisionResult> single_collision_iteration(
        const FabrikChain& current_chain,
        const Vector3& target_position,
        const std::vector<Obstacle>& obstacles,
        const CollisionAwareConfig& config);
    
    // Logging and debugging
    static void log_collision_iteration(int iteration, const CollisionResult& collision_result, 
                                       bool verbose);
};

} // namespace delta

#endif // DELTA_COLLISION_AWARE_SOLVER_HPP