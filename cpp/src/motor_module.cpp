#include "motor_module.hpp"
#include "fabrik_initialization.hpp"
#include <cmath>

namespace delta {

MotorResult MotorModule::calculate_motors(double target_x, double target_y, double target_z) {
    return calculate_motors(Vector3(target_x, target_y, target_z));
}

MotorResult MotorModule::calculate_motors(const Vector3& target_position) {
    // Step 1: Initialize FABRIK chain
    FabrikInitResult init_result = FabrikInitialization::initialize_straight_up(DEFAULT_ROBOT_SEGMENTS);
    
    // Step 2: Solve FABRIK to target
    FabrikSolutionResult fabrik_result = FabrikSolver::solve(init_result.chain, target_position, 
                                                           FABRIK_TOLERANCE, FABRIK_MAX_ITERATIONS);
    
    MotorResult motor_result(target_position, fabrik_result.converged, fabrik_result.final_error);
    
    extract_original_segment_data(fabrik_result, motor_result);
    
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
            double rotation_matrix[3][3];
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
                Vector3 transformed_pos = apply_rotation_matrix(translated_pos, rotation_matrix);

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
    motor_result.original_segment_numbers.clear();
    motor_result.original_segment_positions.clear();
    for (const auto& seg_data : fabrik_result.segment_end_effectors) {
        motor_result.original_segment_numbers.push_back(seg_data.segment_number);
        motor_result.original_segment_positions.push_back(seg_data.end_effector_position);
    }
}

void MotorModule::create_uvw_to_xyz_rotation_matrix(const Vector3& u_axis, const Vector3& v_axis, const Vector3& w_axis, double rotation_matrix[3][3]) {
    // Ensure we align U+ with X+, V+ with Y+, W+ with Z+
    // We need to check the dot products and flip axes if necessary
    
    Vector3 aligned_u = u_axis;
    Vector3 aligned_v = v_axis;
    Vector3 aligned_w = w_axis;
    
    // Check if U-axis is pointing in the negative X direction
    if (u_axis.x < 0) {
        aligned_u = Vector3(-u_axis.x, -u_axis.y, -u_axis.z);
    }
    
    // Check if V-axis is pointing in the negative Y direction
    if (v_axis.y < 0) {
        aligned_v = Vector3(-v_axis.x, -v_axis.y, -v_axis.z);
    }
    
    // Check if W-axis is pointing in the negative Z direction
    if (w_axis.z < 0) {
        aligned_w = Vector3(-w_axis.x, -w_axis.y, -w_axis.z);
    }
    
    // Create rotation matrix with aligned axes
    // The rows of this matrix are the aligned U, V, W vectors
    rotation_matrix[0][0] = aligned_u.x; rotation_matrix[0][1] = aligned_u.y; rotation_matrix[0][2] = aligned_u.z;
    rotation_matrix[1][0] = aligned_v.x; rotation_matrix[1][1] = aligned_v.y; rotation_matrix[1][2] = aligned_v.z;
    rotation_matrix[2][0] = aligned_w.x; rotation_matrix[2][1] = aligned_w.y; rotation_matrix[2][2] = aligned_w.z;
}

Vector3 MotorModule::apply_rotation_matrix(const Vector3& vector, const double rotation_matrix[3][3]) {
    return Vector3(
        rotation_matrix[0][0] * vector.x + rotation_matrix[0][1] * vector.y + rotation_matrix[0][2] * vector.z,
        rotation_matrix[1][0] * vector.x + rotation_matrix[1][1] * vector.y + rotation_matrix[1][2] * vector.z,
        rotation_matrix[2][0] * vector.x + rotation_matrix[2][1] * vector.y + rotation_matrix[2][2] * vector.z
    );
}

} // namespace delta