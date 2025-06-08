#include "collision_aware_solver.hpp"
#include "../fabrik/fabrik_initialization.hpp"
#include <chrono>
#include <iostream>

namespace delta {

CollisionAwareSolutionResult CollisionAwareSolver::solve_with_collision_avoidance(
    const Vector3& target_position,
    const std::vector<Obstacle>& obstacles,
    const CollisionAwareConfig& config) {
    
    // Initialize chain using straight-up initialization
    FabrikInitResult init_result = FabrikInitialization::initialize_straight_up(DEFAULT_ROBOT_SEGMENTS);
    
    return run_collision_aware_algorithm(init_result.chain, target_position, obstacles, config);
}

CollisionAwareSolutionResult CollisionAwareSolver::solve_with_collision_avoidance(
    const Vector3& target_position,
    const std::vector<Obstacle>& obstacles,
    const std::vector<Vector3>& initial_joint_positions,
    int num_robot_segments,
    const CollisionAwareConfig& config) {
    
    // Initialize chain from provided joint positions
    FabrikInitResult init_result = FabrikInitialization::initialize_from_joint_positions(
        num_robot_segments, initial_joint_positions);
    
    return run_collision_aware_algorithm(init_result.chain, target_position, obstacles, config);
}

CollisionAwareSolutionResult CollisionAwareSolver::solve(
    const Vector3& target_position,
    const std::vector<Obstacle>& obstacles,
    int max_iterations) {
    
    CollisionAwareConfig config;
    config.max_collision_iterations = max_iterations;
    
    return solve_with_collision_avoidance(target_position, obstacles, config);
}

bool CollisionAwareSolver::has_collision(const FabrikChain& chain, 
                                        const std::vector<Obstacle>& obstacles,
                                        double spline_diameter) {
    
    if (obstacles.empty()) {
        return false;
    }
    
    // Extract U points from chain
    std::vector<Vector3> u_points = UPointsExtractor::extract_u_points(chain);
    
    // Quick collision check (no optimization)
    CollisionResult collision_result = CollisionDetector::check_and_avoid(u_points, obstacles, spline_diameter);
    
    return collision_result.has_collision;
}

// Private methods

CollisionAwareSolutionResult CollisionAwareSolver::run_collision_aware_algorithm(
    const FabrikChain& initial_chain,
    const Vector3& target_position,
    const std::vector<Obstacle>& obstacles,
    const CollisionAwareConfig& config) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    if (config.verbose_logging) {
        std::cout << "Starting collision-aware solving..." << std::endl;
        std::cout << "Target: (" << target_position.x() << ", " << target_position.y() << ", " << target_position.z() << ")" << std::endl;
        std::cout << "Obstacles: " << obstacles.size() << std::endl;
        std::cout << "Max collision iterations: " << config.max_collision_iterations << std::endl;
    }
    
    // If collision detection is disabled, just run normal FABRIK
    if (!config.enable_collision_detection || obstacles.empty()) {
        if (config.verbose_logging) {
            std::cout << "Collision detection disabled or no obstacles - running normal FABRIK" << std::endl;
        }
        
        FabrikSolutionResult fabrik_result = FabrikSolver::solve_to_target(
            initial_chain, target_position, config.fabrik_config);
        
        return CollisionAwareSolutionResult(fabrik_result, true);
    }
    
    FabrikChain current_chain = initial_chain;
    std::vector<CollisionResult> collision_history;
    double total_collision_time = 0.0;
    
    for (int iteration = 0; iteration < config.max_collision_iterations; ++iteration) {
        if (config.verbose_logging) {
            std::cout << "\n--- Collision Iteration " << iteration + 1 << " ---" << std::endl;
        }
        
        // Step 1: Run FABRIK solver on current chain
        FabrikSolutionResult fabrik_result = FabrikSolver::solve_to_target(
            current_chain, target_position, config.fabrik_config);
        
        if (!fabrik_result.converged) {
            if (config.verbose_logging) {
                std::cout << "FABRIK failed to converge in iteration " << iteration + 1 << std::endl;
            }
            // Return best effort result
            CollisionAwareSolutionResult result(fabrik_result, false);
            result.collision_iterations = iteration + 1;
            result.total_collision_time_ms = total_collision_time;
            result.collision_history = collision_history;
            return result;
        }
        
        // Step 2: Extract U points from solved FABRIK chain
        std::vector<Vector3> u_points = UPointsExtractor::extract_u_points(fabrik_result.final_chain);
        
        if (config.verbose_logging) {
            std::cout << "Extracted " << u_points.size() << " U points" << std::endl;
        }
        
        // Step 3: Check collision using ConicalSwarmSplineAvoider
        CollisionResult collision_result = CollisionDetector::check_and_avoid(
            u_points, obstacles, config.spline_diameter);
        
        collision_history.push_back(collision_result);
        total_collision_time += collision_result.computation_time;
        
        log_collision_iteration(iteration + 1, collision_result, config.verbose_logging);
        
        // Step 4: If no collision → we're done!
        if (!collision_result.has_collision) {
            if (config.verbose_logging) {
                std::cout << "✓ No collision detected - solution is collision-free!" << std::endl;
            }
            
            CollisionAwareSolutionResult result(fabrik_result, true);
            result.collision_iterations = iteration + 1;
            result.total_collision_time_ms = total_collision_time;
            result.collision_history = collision_history;
            return result;
        }
        
        // Step 5: Convert waypoints back to joint positions
        WaypointConversionResult conversion_result = WaypointConverter::convert_waypoints_to_joints(
            collision_result.waypoints);
        
        if (!conversion_result.conversion_successful) {
            if (config.verbose_logging) {
                std::cout << "⚠️  Waypoint conversion failed in iteration " << iteration + 1 << std::endl;
            }
            
            // Return best effort result with collision
            CollisionAwareSolutionResult result(fabrik_result, false);
            result.collision_iterations = iteration + 1;
            result.total_collision_time_ms = total_collision_time;
            result.collision_history = collision_history;
            result.conversion_successful = false;
            return result;
        }
        
        if (config.verbose_logging) {
            std::cout << "Converted waypoints to " << conversion_result.joint_positions.size() << " joint positions" << std::endl;
        }
        
        // Step 6: Create new FABRIK chain from converted joint positions
        FabrikInitResult new_init_result = FabrikInitialization::initialize_from_joint_positions(
            initial_chain.num_robot_segments, conversion_result.joint_positions);
        
        current_chain = new_init_result.chain;
        
        if (config.verbose_logging) {
            std::cout << "Re-initialized FABRIK chain for next iteration" << std::endl;
        }
    }
    
    // Maximum iterations reached - return final attempt
    if (config.verbose_logging) {
        std::cout << "⚠️  Maximum collision iterations reached (" << config.max_collision_iterations << ")" << std::endl;
    }
    
    // Run final FABRIK solve on the last collision-optimized chain
    FabrikSolutionResult final_fabrik_result = FabrikSolver::solve_to_target(
        current_chain, target_position, config.fabrik_config);
    
    // Check if final result has collision
    bool final_collision_free = !has_collision(final_fabrik_result.final_chain, obstacles, config.spline_diameter);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    total_collision_time += duration.count();
    
    CollisionAwareSolutionResult result(final_fabrik_result, final_collision_free);
    result.collision_iterations = config.max_collision_iterations;
    result.total_collision_time_ms = total_collision_time;
    result.collision_history = collision_history;
    
    if (config.verbose_logging) {
        std::cout << "Final result: " << (final_collision_free ? "COLLISION-FREE" : "HAS COLLISION") << std::endl;
        std::cout << "Total collision detection time: " << total_collision_time << "ms" << std::endl;
    }
    
    return result;
}

std::pair<FabrikChain, CollisionResult> CollisionAwareSolver::single_collision_iteration(
    const FabrikChain& current_chain,
    const Vector3& target_position,
    const std::vector<Obstacle>& obstacles,
    const CollisionAwareConfig& config) {
    
    // Step 1: Run FABRIK solver
    FabrikSolutionResult fabrik_result = FabrikSolver::solve_to_target(
        current_chain, target_position, config.fabrik_config);
    
    // Step 2: Extract U points and check collision
    std::vector<Vector3> u_points = UPointsExtractor::extract_u_points(fabrik_result.final_chain);
    CollisionResult collision_result = CollisionDetector::check_and_avoid(
        u_points, obstacles, config.spline_diameter);
    
    return std::make_pair(fabrik_result.final_chain, collision_result);
}

void CollisionAwareSolver::log_collision_iteration(int iteration, const CollisionResult& collision_result, 
                                                  bool verbose) {
    if (verbose) {
        std::cout << "Collision iteration " << iteration << ": ";
        if (collision_result.has_collision) {
            std::cout << "COLLISION DETECTED (min_dist: " << collision_result.min_distance << "mm, ";
            std::cout << "time: " << collision_result.computation_time << "ms)" << std::endl;
        } else {
            std::cout << "NO COLLISION (min_dist: " << collision_result.min_distance << "mm, ";
            std::cout << "time: " << collision_result.computation_time << "ms)" << std::endl;
        }
    }
}

} // namespace delta