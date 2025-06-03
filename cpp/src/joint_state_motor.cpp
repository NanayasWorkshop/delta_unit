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
    
    std::cout << "=== Sequential Joint State Motor Module ===" << std::endl;
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
    
    // Print motor calculations
    if (!result.all_segment_motors.empty()) {
        std::cout << "\n=== Sequential Motor Calculations ===" << std::endl;
        for (const auto& motor_data : result.all_segment_motors) {
            std::cout << "\nSegment " << motor_data.segment_number << " Motors:" << std::endl;
            std::cout << "  FABRIK Position: (" << motor_data.fabrik_position.x << ", " 
                      << motor_data.fabrik_position.y << ", " << motor_data.fabrik_position.z << ")" << std::endl;
            std::cout << "  Local Position:  (" << motor_data.local_position.x << ", " 
                      << motor_data.local_position.y << ", " << motor_data.local_position.z << ")" << std::endl;
            std::cout << "  Base Motors: z_A=" << motor_data.z_A << ", z_B=" << motor_data.z_B 
                      << ", z_C=" << motor_data.z_C << std::endl;
            std::cout << "  Joint Motors: prismatic=" << motor_data.prismatic_joint 
                      << ", roll=" << motor_data.roll_joint * 180/M_PI << "°" 
                      << ", pitch=" << motor_data.pitch_joint * 180/M_PI << "°" << std::endl;
        }
    } else {
        std::cout << "\n=== Motor Positions (Not Calculated) ===" << std::endl;
        std::cout << "Enable motor calculations by ensuring FABRIK has segment end-effectors" << std::endl;
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
    
    // Calculate motor positions for all segments using CORRECTED sequential method
    if (!fabrik_result.segment_end_effectors.empty()) {
        motor_result.all_segment_motors = calculate_all_segment_motors_sequential(fabrik_result.segment_end_effectors);
    }
    
    return motor_result;
}

// CORRECTED: Sequential calculation with proper UVW→XYZ alignment
std::vector<SegmentMotorData> JointStateMotorModule::calculate_all_segment_motors_sequential(
    const std::vector<SegmentEndEffectorData>& segment_end_effectors) {
    
    std::vector<SegmentMotorData> all_motor_data;
    std::vector<Vector3> current_positions;
    
    // Extract all segment positions from FABRIK
    for (const auto& seg : segment_end_effectors) {
        current_positions.push_back(seg.end_effector_position);
    }
    
    std::cout << "\n=== Sequential Transformation Process ===" << std::endl;
    
    // Sequential calculation for each segment
    for (size_t i = 0; i < segment_end_effectors.size(); i++) {
        Vector3 current_position = current_positions[i];
        
        std::cout << "\n--- Processing Segment " << (i+1) << " ---" << std::endl;
        std::cout << "Current position: (" << current_position.x << ", " << current_position.y << ", " << current_position.z << ")" << std::endl;
        
        // Calculate motor data for current segment using current position
        SegmentMotorData motor_data = calculate_single_segment_motors(
            segment_end_effectors[i].segment_number,
            segment_end_effectors[i].end_effector_position,  // Original FABRIK position
            current_position                                 // Current (possibly transformed) position
        );
        
        all_motor_data.push_back(motor_data);
        
        // If not the last segment, transform remaining positions
        if (i < segment_end_effectors.size() - 1) {
            std::cout << "Transforming remaining " << (segment_end_effectors.size() - i - 1) << " segments" << std::endl;
            std::cout << "Using UVW frame from Segment " << (i+1) << " to align with XYZ" << std::endl;
            
            // Transform remaining positions to align UVW→XYZ
            for (size_t j = i + 1; j < current_positions.size(); j++) {
                Vector3 transformed_position = transform_to_local_coordinates(
                    current_positions[j],           // World position to transform
                    motor_data.fabrik_position,     // Reference position (current segment)
                    motor_data.uvw_frame           // Reference UVW frame
                );
                
                std::cout << "  Segment " << (j+1) << ": (" 
                          << current_positions[j].x << ", " << current_positions[j].y << ", " << current_positions[j].z 
                          << ") → (" << transformed_position.x << ", " << transformed_position.y << ", " << transformed_position.z << ")" << std::endl;
                
                current_positions[j] = transformed_position;
            }
        }
    }
    
    return all_motor_data;
}

// Transform world coordinates to local coordinate system where UVW aligns with XYZ
Vector3 JointStateMotorModule::transform_to_local_coordinates(const Vector3& world_position, 
                                                             const Vector3& reference_position,
                                                             const CoordinateFrame& reference_frame) {
    
    // Step 1: Translate to make reference position the origin
    Vector3 translated = world_position - reference_position;
    
    // Step 2: Rotate to align UVW with XYZ
    // We want to transform from world coordinates to local coordinates where:
    // - reference_frame.u_axis becomes (1,0,0)
    // - reference_frame.v_axis becomes (0,1,0)  
    // - reference_frame.w_axis becomes (0,0,1)
    
    // Create transformation matrix (inverse of UVW frame)
    // Since UVW is orthonormal, inverse = transpose
    double local_x = translated.x * reference_frame.u_axis.x + 
                     translated.y * reference_frame.u_axis.y + 
                     translated.z * reference_frame.u_axis.z;
    
    double local_y = translated.x * reference_frame.v_axis.x + 
                     translated.y * reference_frame.v_axis.y + 
                     translated.z * reference_frame.v_axis.z;
    
    double local_z = translated.x * reference_frame.w_axis.x + 
                     translated.y * reference_frame.w_axis.y + 
                     translated.z * reference_frame.w_axis.z;
    
    return Vector3(local_x, local_y, local_z);
}

SegmentMotorData JointStateMotorModule::calculate_single_segment_motors(
    int segment_number,
    const Vector3& fabrik_position,
    const Vector3& local_position) {
    
    // Create motor data structure
    SegmentMotorData motor_data(segment_number, fabrik_position);
    motor_data.local_position = local_position;
    
    // Run kinematics calculation on local position (transformed coordinates)
    KinematicsResult kinematics_result = KinematicsModule::calculate(local_position);
    
    // Extract motor positions
    motor_data.z_A = kinematics_result.fermat_data.z_A;
    motor_data.z_B = kinematics_result.fermat_data.z_B;
    motor_data.z_C = kinematics_result.fermat_data.z_C;
    motor_data.prismatic_joint = kinematics_result.joint_state_data.prismatic_joint;
    motor_data.roll_joint = kinematics_result.joint_state_data.roll_joint;
    motor_data.pitch_joint = kinematics_result.joint_state_data.pitch_joint;
    
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
    
    std::cout << "Solving " << targets.size() << " targets with sequential calculation..." << std::endl;
    
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