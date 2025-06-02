#include "fabrik_solver.hpp"
#include <chrono>
#include <iostream>

namespace delta {

FabrikSolutionResult FabrikSolver::solve_to_target(const FabrikChain& initial_chain,
                                                  const Vector3& target_position,
                                                  const FabrikSolverConfig& config) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Run the main FABRIK algorithm
    FabrikSolutionResult result = run_fabrik_algorithm(initial_chain, target_position, config);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    result.solve_time_ms = duration.count() / 1000.0;
    
    return result;
}

FabrikSolutionResult FabrikSolver::solve(const FabrikChain& initial_chain,
                                        const Vector3& target_position,
                                        double tolerance,
                                        int max_iterations) {
    
    FabrikSolverConfig config;
    config.tolerance = tolerance;
    config.max_iterations = max_iterations;
    
    return solve_to_target(initial_chain, target_position, config);
}

FabrikChain FabrikSolver::single_fabrik_cycle(const FabrikChain& chain,
                                             const Vector3& target_position,
                                             const FabrikSolverConfig& config) {
    
    // Step 1: Backward iteration
    FabrikBackwardResult backward_result = FabrikBackward::iterate_to_target(
        chain, target_position, config.tolerance, config.max_iterations);
    
    // Step 2: Forward iteration  
    FabrikForwardResult forward_result = FabrikForward::iterate_from_base(
        backward_result.updated_chain, config.tolerance, config.max_iterations);
    
    return forward_result.updated_chain;
}

bool FabrikSolver::is_solution_valid(const FabrikChain& chain, double tolerance) {
    // Check if base is at origin
    Vector3 base_pos = chain.joints[0].position;
    if (base_pos.norm() > tolerance) {
        return false;
    }
    
    // Check if segment lengths are preserved
    for (const auto& segment : chain.segments) {
        Vector3 start_pos = chain.joints[segment.start_joint_index].position;
        Vector3 end_pos = chain.joints[segment.end_joint_index].position;
        
        double actual_length = (end_pos - start_pos).norm();
        double expected_length = segment.length;
        
        if (std::abs(actual_length - expected_length) > tolerance) {
            return false;
        }
    }
    
    return true;
}

double FabrikSolver::calculate_chain_error(const FabrikChain& chain) {
    double total_error = 0.0;
    
    // Base position error
    Vector3 base_pos = chain.joints[0].position;
    total_error += base_pos.norm();
    
    // Segment length errors
    for (const auto& segment : chain.segments) {
        Vector3 start_pos = chain.joints[segment.start_joint_index].position;
        Vector3 end_pos = chain.joints[segment.end_joint_index].position;
        
        double actual_length = (end_pos - start_pos).norm();
        double expected_length = segment.length;
        
        total_error += std::abs(actual_length - expected_length);
    }
    
    return total_error;
}

Vector3 FabrikSolver::get_end_effector_position(const FabrikChain& chain) {
    if (chain.joints.empty()) return Vector3(0, 0, 0);
    return chain.joints.back().position;
}

FabrikSolverConfig FabrikSolver::create_fast_config() {
    FabrikSolverConfig config;
    config.tolerance = 0.1;  // Loose tolerance
    config.max_iterations = 20;
    config.max_backward_forward_cycles = 10;
    config.enable_constraints = true;
    config.track_convergence_history = false;
    config.verbose_logging = false;
    return config;
}

FabrikSolverConfig FabrikSolver::create_precise_config() {
    FabrikSolverConfig config;
    config.tolerance = 0.001;  // Tight tolerance
    config.max_iterations = 200;
    config.max_backward_forward_cycles = 100;
    config.enable_constraints = true;
    config.track_convergence_history = true;
    config.verbose_logging = false;
    return config;
}

FabrikSolverConfig FabrikSolver::create_debug_config() {
    FabrikSolverConfig config;
    config.tolerance = FABRIK_TOLERANCE;
    config.max_iterations = FABRIK_MAX_ITERATIONS;
    config.max_backward_forward_cycles = 50;
    config.enable_constraints = true;
    config.track_convergence_history = true;
    config.verbose_logging = true;
    return config;
}

// Private methods

FabrikSolutionResult FabrikSolver::run_fabrik_algorithm(const FabrikChain& initial_chain,
                                                       const Vector3& target_position,
                                                       const FabrikSolverConfig& config) {
    
    FabrikChain current_chain = initial_chain;
    std::vector<Vector3> convergence_history;
    
    int total_backward_iterations = 0;
    int total_forward_iterations = 0;
    
    if (config.verbose_logging) {
        std::cout << "Starting FABRIK algorithm..." << std::endl;
        std::cout << "Target: (" << target_position.x << ", " << target_position.y << ", " << target_position.z << ")" << std::endl;
    }
    
    // Main FABRIK loop: alternate between backward and forward iterations
    for (int cycle = 0; cycle < config.max_backward_forward_cycles; cycle++) {
        
        // Step 1: Backward iteration (move end-effector toward target)
        FabrikBackwardResult backward_result = FabrikBackward::iterate_to_target(
            current_chain, target_position, config.tolerance, config.max_iterations);
        
        total_backward_iterations += backward_result.iterations_used;
        current_chain = backward_result.updated_chain;
        
        // Step 2: Forward iteration (fix base at origin)
        FabrikForwardResult forward_result = FabrikForward::iterate_from_base(
            current_chain, config.tolerance, config.max_iterations);
        
        total_forward_iterations += forward_result.iterations_used;
        current_chain = forward_result.updated_chain;
        
        // Get current end-effector position
        Vector3 current_end_effector = get_end_effector_position(current_chain);
        
        // Track convergence history
        if (config.track_convergence_history) {
            convergence_history.push_back(current_end_effector);
        }
        
        // Calculate current error
        double current_error = (current_end_effector - target_position).norm();
        
        // Log progress
        if (config.verbose_logging) {
            log_iteration(cycle, current_end_effector, target_position, current_error, true);
        }
        
        // Check convergence
        if (has_converged(current_end_effector, target_position, config.tolerance)) {
            if (config.verbose_logging) {
                std::cout << "✓ Converged after " << cycle + 1 << " cycles!" << std::endl;
            }
            
            Vector3 achieved_position = current_end_effector;
            double final_error = current_error;
            
            FabrikSolutionResult result(current_chain, target_position, achieved_position, 
                                      true, final_error, total_backward_iterations + total_forward_iterations);
            result.backward_iterations = total_backward_iterations;
            result.forward_iterations = total_forward_iterations;
            result.convergence_history = convergence_history;
            
            return result;
        }
        
        // Check for stalling
        if (config.track_convergence_history && has_stalled(convergence_history)) {
            if (config.verbose_logging) {
                std::cout << "⚠️  Algorithm stalled after " << cycle + 1 << " cycles" << std::endl;
            }
            break;
        }
    }
    
    // Did not converge within max cycles
    if (config.verbose_logging) {
        std::cout << "✗ Did not converge within " << config.max_backward_forward_cycles << " cycles" << std::endl;
    }
    
    Vector3 achieved_position = get_end_effector_position(current_chain);
    double final_error = (achieved_position - target_position).norm();
    
    FabrikSolutionResult result(current_chain, target_position, achieved_position, 
                              false, final_error, total_backward_iterations + total_forward_iterations);
    result.backward_iterations = total_backward_iterations;
    result.forward_iterations = total_forward_iterations;
    result.convergence_history = convergence_history;
    
    return result;
}

bool FabrikSolver::has_converged(const Vector3& end_effector, const Vector3& target, double tolerance) {
    double distance = (end_effector - target).norm();
    return distance <= tolerance;
}

bool FabrikSolver::has_stalled(const std::vector<Vector3>& history, int min_history) {
    if (static_cast<int>(history.size()) < min_history) {
        return false;
    }
    
    // Check if last few positions are very similar (indicating stalling)
    int check_count = std::min(min_history, static_cast<int>(history.size()));
    Vector3 last_pos = history.back();
    
    for (int i = 1; i < check_count; i++) {
        Vector3 prev_pos = history[history.size() - 1 - i];
        double distance = (last_pos - prev_pos).norm();
        
        if (distance > 0.001) {  // Still making progress
            return false;
        }
    }
    
    return true;  // Stalled
}

void FabrikSolver::log_iteration(int iteration, const Vector3& end_effector, 
                                const Vector3& target, double error, bool verbose) {
    if (verbose) {
        std::cout << "  Cycle " << iteration << ": End-effector=(" 
                  << end_effector.x << ", " << end_effector.y << ", " << end_effector.z 
                  << "), Error=" << error << std::endl;
    }
}

// Utility functions
namespace fabrik_utils {

FabrikSolutionResult solve_delta_robot(int num_segments, 
                                     const Vector3& target,
                                     double tolerance) {
    
    // Initialize chain
    FabrikInitResult init_result = FabrikInitialization::initialize_straight_up(num_segments);
    
    // Solve
    return FabrikSolver::solve(init_result.chain, target, tolerance);
}

std::vector<FabrikSolutionResult> solve_multiple_targets(const FabrikChain& initial_chain,
                                                       const std::vector<Vector3>& targets,
                                                       const FabrikSolverConfig& config) {
    
    std::vector<FabrikSolutionResult> results;
    results.reserve(targets.size());
    
    for (const Vector3& target : targets) {
        FabrikSolutionResult result = FabrikSolver::solve_to_target(initial_chain, target, config);
        results.push_back(result);
    }
    
    return results;
}

Vector3 find_max_reach(const FabrikChain& chain, const Vector3& direction) {
    // Calculate total reach
    double total_reach = 0.0;
    for (const auto& segment : chain.segments) {
        total_reach += segment.length;
    }
    
    // Return maximum reach in given direction
    Vector3 normalized_direction = direction.normalized();
    Vector3 base_position = chain.joints[0].position;
    
    return base_position + normalized_direction * total_reach;
}

bool is_target_reachable(const FabrikChain& chain, const Vector3& target) {
    return FabrikBackward::is_target_reachable(chain, target);
}

} // namespace fabrik_utils

} // namespace delta