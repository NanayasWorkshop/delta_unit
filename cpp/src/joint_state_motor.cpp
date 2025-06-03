#include "joint_state_motor.hpp"
#include <iostream>
#include <chrono>

namespace delta {

JointStateMotorResult JointStateMotorModule::calculate_motors(double target_x, double target_y, double target_z) {
    return calculate_motors(Vector3(target_x, target_y, target_z));
}

JointStateMotorResult JointStateMotorModule::calculate_motors(const Vector3& target_position) {
    // Use default constants from constants.hpp
    return calculate_motors_advanced(target_position, 
                                   DEFAULT_ROBOT_SEGMENTS, 
                                   FABRIK_TOLERANCE, 
                                   FABRIK_MAX_ITERATIONS);
}

JointStateMotorResult JointStateMotorModule::calculate_motors_advanced(const Vector3& target_position,
                                                                      int num_segments,
                                                                      double tolerance,
                                                                      int max_iterations) {
    
    std::cout << "=== JointState Motor Module ===" << std::endl;
    std::cout << "Target: (" << target_position.x << ", " << target_position.y << ", " << target_position.z << ")" << std::endl;
    std::cout << "Using " << num_segments << " segments, tolerance: " << tolerance << std::endl;
    
    // Call FABRIK solver internally
    JointStateMotorResult result = solve_with_fabrik(target_position, num_segments, tolerance, max_iterations);
    
    // Print results for testing
    std::cout << "\n=== FABRIK Solution ===" << std::endl;
    std::cout << "Converged: " << (result.fabrik_converged ? "YES" : "NO") << std::endl;
    std::cout << "Final error: " << result.fabrik_error << std::endl;
    std::cout << "Iterations: " << result.fabrik_iterations << std::endl;
    std::cout << "Solve time: " << result.solve_time_ms << " ms" << std::endl;
    
    std::cout << "\n=== End-Effector Positions ===" << std::endl;
    std::cout << "Target:   (" << result.target_position.x << ", " << result.target_position.y << ", " << result.target_position.z << ")" << std::endl;
    std::cout << "Achieved: (" << result.achieved_end_effector.x << ", " << result.achieved_end_effector.y << ", " << result.achieved_end_effector.z << ")" << std::endl;
    
    // Print segment end-effectors if available
    if (!result.fabrik_result.segment_end_effectors.empty()) {
        std::cout << "\n=== Segment End-Effectors ===" << std::endl;
        for (const auto& seg_data : result.fabrik_result.segment_end_effectors) {
            Vector3 pos = seg_data.end_effector_position;
            std::cout << "Segment " << seg_data.segment_number << ": (" 
                      << pos.x << ", " << pos.y << ", " << pos.z << ")" 
                      << " [Prismatic: " << seg_data.prismatic_length << "]" << std::endl;
        }
    }
    
    std::cout << "\n=== Motor Positions (TODO) ===" << std::endl;
    std::cout << "Motor position calculations will be implemented here" << std::endl;
    std::cout << "For now, using end-effector positions from FABRIK solver" << std::endl;
    
    std::cout << "===============================\n" << std::endl;
    
    return result;
}

bool JointStateMotorModule::is_target_reachable(const Vector3& target_position, int num_segments) {
    // Initialize chain to check reachability
    FabrikInitResult init_result = FabrikInitialization::initialize_straight_up(num_segments);
    return fabrik_utils::is_target_reachable(init_result.chain, target_position);
}

JointStateMotorResult JointStateMotorModule::solve_with_fabrik(const Vector3& target_position,
                                                              int num_segments,
                                                              double tolerance,
                                                              int max_iterations) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Initialize FABRIK chain
    FabrikInitResult init_result = FabrikInitialization::initialize_straight_up(num_segments);
    
    // Check reachability first
    if (!fabrik_utils::is_target_reachable(init_result.chain, target_position)) {
        std::cout << "Warning: Target may be outside workspace (distance: " 
                  << target_position.norm() << ", max reach: " << init_result.total_reach << ")" << std::endl;
    }
    
    // Solve with FABRIK
    FabrikSolutionResult fabrik_result = FabrikSolver::solve(init_result.chain, target_position, tolerance, max_iterations);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double solve_time_ms = duration.count() / 1000.0;
    
    // Create motor result
    JointStateMotorResult motor_result(
        target_position,
        fabrik_result.achieved_position,
        fabrik_result.converged,
        fabrik_result.final_error,
        fabrik_result.total_iterations,
        solve_time_ms,
        fabrik_result
    );
    
    return motor_result;
}

// Utility functions
namespace joint_state_utils {

JointStateMotorResult solve_target(double x, double y, double z) {
    return JointStateMotorModule::calculate_motors(x, y, z);
}

std::vector<JointStateMotorResult> solve_multiple_targets(const std::vector<Vector3>& targets) {
    std::vector<JointStateMotorResult> results;
    results.reserve(targets.size());
    
    std::cout << "Solving " << targets.size() << " targets..." << std::endl;
    
    for (size_t i = 0; i < targets.size(); i++) {
        std::cout << "\n--- Target " << (i + 1) << "/" << targets.size() << " ---" << std::endl;
        JointStateMotorResult result = JointStateMotorModule::calculate_motors(targets[i]);
        results.push_back(result);
    }
    
    return results;
}

bool test_workspace_point(double x, double y, double z) {
    Vector3 target(x, y, z);
    bool reachable = JointStateMotorModule::is_target_reachable(target);
    
    std::cout << "Target (" << x << ", " << y << ", " << z << "): " 
              << (reachable ? "REACHABLE" : "OUT OF REACH") << std::endl;
    
    return reachable;
}

} // namespace joint_state_utils

} // namespace delta