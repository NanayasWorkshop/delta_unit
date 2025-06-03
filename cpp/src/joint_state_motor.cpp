// SIMPLIFIED joint_state_motor.cpp - SINGLE SEGMENT ONLY

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
    
    std::cout << "=== SIMPLE Single Segment Motor Module ===" << std::endl;
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
        std::cout << "\n=== FABRIK Segment End-Effectors ===" << std::endl;
        for (const auto& seg_data : result.fabrik_result.segment_end_effectors) {
            Vector3 pos = seg_data.end_effector_position;
            std::cout << "Segment " << seg_data.segment_number << ": (" 
                      << pos.x << ", " << pos.y << ", " << pos.z << ")" 
                      << " [Prismatic: " << seg_data.prismatic_length << "]" << std::endl;
        }
    }
    
    // SIMPLIFIED: Only process FIRST segment
    if (!result.fabrik_result.segment_end_effectors.empty()) {
        std::cout << "\n=== PROCESSING FIRST SEGMENT ONLY ===" << std::endl;
        result.all_segment_motors = calculate_single_segment_only(result.fabrik_result.segment_end_effectors);
    }
    
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

// SIMPLIFIED: Process only the first segment
std::vector<SegmentMotorData> JointStateMotorModule::calculate_single_segment_only(
    const std::vector<SegmentEndEffectorData>& segment_end_effectors) {
    
    std::vector<SegmentMotorData> motor_data_list;
    
    if (segment_end_effectors.empty()) {
        std::cout << "No segment end-effectors available" << std::endl;
        return motor_data_list;
    }
    
    // Get ONLY the first segment
    const SegmentEndEffectorData& first_segment = segment_end_effectors[0];
    Vector3 segment_position = first_segment.end_effector_position;
    
    std::cout << "\n=== SINGLE SEGMENT ANALYSIS ===" << std::endl;
    std::cout << "Processing Segment " << first_segment.segment_number << std::endl;
    std::cout << "Position: (" << segment_position.x << ", " << segment_position.y << ", " << segment_position.z << ")" << std::endl;
    
    // Calculate motors for this segment
    SegmentMotorData motor_data = calculate_single_segment_motors(
        first_segment.segment_number,
        segment_position,  // FABRIK position
        segment_position   // Local position (same as FABRIK - no transformation)
    );
    
    motor_data_list.push_back(motor_data);
    
    // Print detailed results
    std::cout << "\n=== KINEMATICS RESULTS ===" << std::endl;
    std::cout << "Base Motors: z_A=" << motor_data.z_A << ", z_B=" << motor_data.z_B << ", z_C=" << motor_data.z_C << std::endl;
    std::cout << "Joint Motors: prismatic=" << motor_data.prismatic_joint 
              << ", roll=" << motor_data.roll_joint * 180/M_PI << "°" 
              << ", pitch=" << motor_data.pitch_joint * 180/M_PI << "°" << std::endl;
    
    std::cout << "\n=== ORIENTATION RESULTS ===" << std::endl;
    CoordinateFrame uvw = motor_data.uvw_frame;
    std::cout << "UVW Frame Origin: (" << uvw.origin.x << ", " << uvw.origin.y << ", " << uvw.origin.z << ")" << std::endl;
    std::cout << "U-axis: (" << uvw.u_axis.x << ", " << uvw.u_axis.y << ", " << uvw.u_axis.z << ")" << std::endl;
    std::cout << "V-axis: (" << uvw.v_axis.x << ", " << uvw.v_axis.y << ", " << uvw.v_axis.z << ")" << std::endl;
    std::cout << "W-axis: (" << uvw.w_axis.x << ", " << uvw.w_axis.y << ", " << uvw.w_axis.z << ")" << std::endl;
    
    // Check if UVW frame origin matches segment position
    double position_match_distance = (uvw.origin - segment_position).norm();
    std::cout << "Position match distance: " << position_match_distance << std::endl;
    if (position_match_distance > 0.001) {
        std::cout << "⚠️  UVW frame origin doesn't match segment position!" << std::endl;
    } else {
        std::cout << "✓ UVW frame origin matches segment position" << std::endl;
    }
    
    return motor_data_list;
}

SegmentMotorData JointStateMotorModule::calculate_single_segment_motors(
    int segment_number,
    const Vector3& fabrik_position,
    const Vector3& local_position) {
    
    // Create motor data structure
    SegmentMotorData motor_data(segment_number, fabrik_position);
    motor_data.local_position = local_position;
    
    std::cout << "\n--- Running Kinematics on Position: (" 
              << local_position.x << ", " << local_position.y << ", " << local_position.z << ") ---" << std::endl;
    
    // Run kinematics calculation on local position
    KinematicsResult kinematics_result = KinematicsModule::calculate(local_position);
    
    // Extract motor positions
    motor_data.z_A = kinematics_result.fermat_data.z_A;
    motor_data.z_B = kinematics_result.fermat_data.z_B;
    motor_data.z_C = kinematics_result.fermat_data.z_C;
    motor_data.prismatic_joint = kinematics_result.joint_state_data.prismatic_joint;
    motor_data.roll_joint = kinematics_result.joint_state_data.roll_joint;
    motor_data.pitch_joint = kinematics_result.joint_state_data.pitch_joint;
    
    std::cout << "--- Running Orientation on Kinematics Result ---" << std::endl;
    
    // Get orientation data (UVW frame) from the local position
    OrientationResult orientation_result = OrientationModule::calculate_from_kinematics(kinematics_result);
    motor_data.uvw_frame = orientation_result.final_frame;
    motor_data.local_to_world = orientation_result.transformation_matrix;
    
    return motor_data;
}

// Utility functions
namespace joint_state_utils {

JointStateMotorResult solve_target(double x, double y, double z) {
    return JointStateMotorModule::calculate_motors(x, y, z);
}

std::vector<JointStateMotorResult> solve_multiple_targets(const std::vector<Vector3>& targets) {
    std::vector<JointStateMotorResult> results;
    results.reserve(targets.size());
    
    std::cout << "Solving " << targets.size() << " targets with SINGLE SEGMENT calculation..." << std::endl;
    
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