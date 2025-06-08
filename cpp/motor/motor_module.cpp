#include "motor_module.hpp"
#include "../fabrik/fabrik_initialization.hpp"
#include <cmath>
#include <iostream>
#include <optional>

namespace delta {

MotorResult MotorModule::calculate_motors(double target_x, double target_y, double target_z) {
    return calculate_motors(Vector3(target_x, target_y, target_z));
}

MotorResult MotorModule::calculate_motors(const Vector3& target_position) {
    return calculate_motors(target_position, std::nullopt);
}

MotorResult MotorModule::calculate_motors(double target_x, double target_y, double target_z,
                                        const std::optional<std::vector<Vector3>>& current_joint_positions) {
    return calculate_motors(Vector3(target_x, target_y, target_z), current_joint_positions);
}

MotorResult MotorModule::calculate_motors(const Vector3& target_position,
                                        const std::optional<std::vector<Vector3>>& current_joint_positions) {
    // Step 1: Initialize FABRIK chain
    FabrikInitResult init_result = (current_joint_positions.has_value()) 
        ? FabrikInitialization::initialize_from_joint_positions(DEFAULT_ROBOT_SEGMENTS, current_joint_positions.value())
        : FabrikInitialization::initialize_straight_up(DEFAULT_ROBOT_SEGMENTS);
    
    if (current_joint_positions.has_value()) {
        std::cout << "Initializing FABRIK from provided joint positions (" 
                  << current_joint_positions->size() << " positions)" << std::endl;
    } else {
        std::cout << "Initializing FABRIK from straight-up configuration" << std::endl;
    }
    
    // Step 2: Solve FABRIK to target
    FabrikSolutionResult fabrik_result = FabrikSolver::solve(init_result.chain, target_position, 
                                                           FABRIK_TOLERANCE, FABRIK_MAX_ITERATIONS);
    
    // Pass timing information from FABRIK result
    MotorResult motor_result(target_position, fabrik_result.converged, fabrik_result.final_error, fabrik_result.solve_time_ms);
    
    // NEW: Extract segment data using SegmentCalculator (separated for performance)
    extract_original_segment_data(fabrik_result, motor_result);
    extract_fabrik_joint_positions(fabrik_result, motor_result);
    
    if (motor_result.original_segment_positions.empty()) {
        return motor_result; // No segments, nothing to process
    }

    // These are the positions that will be processed at the start of each level.
    // Initially, they are the original global positions.
    // In subsequent iterations, they are the transformed positions from the PREVIOUS level.
    std::vector<Vector3> current_input_positions = motor_result.original_segment_positions;
    std::vector<int> current_input_original_numbers = motor_result.original_segment_numbers;

    while (!current_input_positions.empty()) {
        LevelData current_level_data;

        // The "base" for this level is the first segment in the current_input_positions.
        Vector3 base_pos_for_this_level = current_input_positions[0];
        
        current_level_data.base_segment_position = base_pos_for_this_level;

        // Calculate Kinematics for this base_pos_for_this_level
        KinematicsResult kinematics_res = KinematicsModule::calculate(base_pos_for_this_level);
        current_level_data.z_A = kinematics_res.fermat_data.z_A;
        current_level_data.z_B = kinematics_res.fermat_data.z_B;
        current_level_data.z_C = kinematics_res.fermat_data.z_C;
        current_level_data.prismatic_joint = kinematics_res.joint_state_data.prismatic_joint;
        current_level_data.roll_joint = rad_to_deg(kinematics_res.joint_state_data.roll_joint);
        current_level_data.pitch_joint = rad_to_deg(kinematics_res.joint_state_data.pitch_joint);

        // Calculate Orientation for this base_pos_for_this_level
        OrientationResult orientation_res = OrientationModule::calculate(base_pos_for_this_level);
        current_level_data.uvw_origin = orientation_res.final_frame.origin;
        current_level_data.uvw_u_axis = orientation_res.final_frame.u_axis;
        current_level_data.uvw_v_axis = orientation_res.final_frame.v_axis;
        current_level_data.uvw_w_axis = orientation_res.final_frame.w_axis;

        // Prepare for the next level's input
        std::vector<Vector3> next_level_input_positions;
        std::vector<int> next_level_input_original_numbers;

        // If there are subsequent segments in current_input_positions (more than just the base), transform them
        if (current_input_positions.size() > 1) {
            Matrix3 rotation_matrix;
            create_uvw_to_xyz_rotation_matrix(current_level_data.uvw_u_axis,
                                            current_level_data.uvw_v_axis,
                                            current_level_data.uvw_w_axis,
                                            rotation_matrix);

            // Iterate from the second segment onwards in the current_input_positions
            for (size_t i = 1; i < current_input_positions.size(); ++i) {
                Vector3 segment_to_transform = current_input_positions[i];
                int original_segment_number = current_input_original_numbers[i];

                // Step 1: Translate so base_pos_for_this_level is at origin
                Vector3 translated_pos = segment_to_transform - base_pos_for_this_level;
                
                // Step 2: Rotate to align this level's UVW with XYZ
                Vector3 transformed_pos = rotation_matrix * translated_pos;

                // Store in the current level's "transformed_segment_*" lists
                current_level_data.transformed_segment_original_numbers.push_back(original_segment_number);
                current_level_data.transformed_segment_positions.push_back(transformed_pos);

                // This transformed position and its original number become part of the input for the *next* level
                next_level_input_positions.push_back(transformed_pos);
                next_level_input_original_numbers.push_back(original_segment_number);
            }
        }
        
        motor_result.levels.push_back(current_level_data);

        // The transformed segments from this level become the input for the next level.
        // If next_level_input_positions is empty (because current_input_positions had only 1 element,
        // or it was the last segment), the loop will terminate.
        current_input_positions = next_level_input_positions;
        current_input_original_numbers = next_level_input_original_numbers;
    }
    
    return motor_result;
}

void MotorModule::extract_original_segment_data(const FabrikSolutionResult& fabrik_result, MotorResult& motor_result) {
    // NEW: Use SegmentCalculator instead of FABRIK segment extraction
    
    motor_result.original_segment_numbers.clear();
    motor_result.original_segment_positions.clear();
    
    // Calculate segment end-effectors using the optimized SegmentCalculator
    SegmentCalculationResult segment_result = SegmentCalculator::calculate_segment_end_effectors(fabrik_result.final_chain);
    
    // Store timing information
    motor_result.segment_calculation_time_ms = segment_result.calculation_time_ms;
    
    if (!segment_result.calculation_successful) {
        std::cerr << "Warning: SegmentCalculator failed to calculate segment end-effectors" << std::endl;
        return;
    }
    
    // Extract segment data from SegmentCalculator result
    for (const auto& seg_data : segment_result.segment_end_effectors) {
        motor_result.original_segment_numbers.push_back(seg_data.segment_number);
        motor_result.original_segment_positions.push_back(seg_data.end_effector_position);
    }
    
    if (motor_result.original_segment_positions.size() > 0) {
        std::cout << "SegmentCalculator: Calculated " << motor_result.original_segment_positions.size() 
                  << " segment end-effectors in " << segment_result.calculation_time_ms << "ms" << std::endl;
    }
}

void MotorModule::extract_fabrik_joint_positions(const FabrikSolutionResult& fabrik_result, MotorResult& motor_result) {
    // Extract joint positions from the solved FABRIK chain
    motor_result.fabrik_joint_positions.clear();
    
    for (const auto& joint : fabrik_result.final_chain.joints) {
        motor_result.fabrik_joint_positions.push_back(joint.position);
    }
}

void MotorModule::create_uvw_to_xyz_rotation_matrix(const Vector3& u_axis, const Vector3& v_axis, const Vector3& w_axis, Matrix3& rotation_matrix) {
    // Ensure we align U+ with X+, V+ with Y+, W+ with Z+
    // We need to check the dot products and flip axes if necessary
    
    Vector3 aligned_u = u_axis;
    Vector3 aligned_v = v_axis;
    Vector3 aligned_w = w_axis;
    
    // Check if U-axis is pointing in the negative X direction
    if (u_axis.x() < 0) {
        aligned_u = -u_axis;
    }
    
    // Check if V-axis is pointing in the negative Y direction
    if (v_axis.y() < 0) {
        aligned_v = -v_axis;
    }
    
    // Check if W-axis is pointing in the negative Z direction
    if (w_axis.z() < 0) {
        aligned_w = -w_axis;
    }
    
    // Create rotation matrix with aligned axes
    // The rows of this matrix are the aligned U, V, W vectors
    rotation_matrix.row(0) = aligned_u.transpose();
    rotation_matrix.row(1) = aligned_v.transpose();
    rotation_matrix.row(2) = aligned_w.transpose();
}

} // namespace delta