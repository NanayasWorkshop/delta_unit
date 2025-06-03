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
    
    // Step 3: Create motor result
    MotorResult motor_result(target_position, fabrik_result.converged, fabrik_result.final_error);
    
    // Step 4: Extract segment data
    extract_segment_data(fabrik_result, motor_result);
    
    // Step 5: Calculate kinematics for first segment (if available)
    if (!motor_result.segment_positions.empty()) {
        motor_result.first_segment_position = motor_result.segment_positions[0];
        calculate_first_segment_kinematics(motor_result.first_segment_position, motor_result);
        calculate_first_segment_orientation(motor_result.first_segment_position, motor_result);
        
        // Step 6: Transform segments 2-8 to UVW-aligned coordinate system
        transform_segments_to_uvw_aligned(motor_result);
        
        // Step 7: Calculate kinematics and orientation for second segment (if available)
        if (!motor_result.transformed_segment_positions.empty()) {
            calculate_second_segment_kinematics(motor_result);
            calculate_second_segment_orientation(motor_result);
            
            // Step 8: Transform segments 3'-8' relative to S2' UVW coordinate system
            transform_segments_to_second_level_uvw_aligned(motor_result);
            
            // Step 9: Calculate kinematics and orientation for third segment (if available)
            if (!motor_result.second_level_transformed_segment_positions.empty()) {
                calculate_third_segment_kinematics(motor_result);
                calculate_third_segment_orientation(motor_result);
            }
        }
    }
    
    return motor_result;
}

void MotorModule::extract_segment_data(const FabrikSolutionResult& fabrik_result, MotorResult& motor_result) {
    // Extract actual segment end-effector positions from FABRIK result
    for (const auto& seg_data : fabrik_result.segment_end_effectors) {
        motor_result.segment_numbers.push_back(seg_data.segment_number);
        motor_result.segment_positions.push_back(seg_data.end_effector_position);
    }
}

void MotorModule::calculate_first_segment_kinematics(const Vector3& first_segment_pos, MotorResult& motor_result) {
    // Run kinematics module on first segment position
    KinematicsResult kinematics_result = KinematicsModule::calculate(first_segment_pos);
    
    // Extract required data
    motor_result.z_A = kinematics_result.fermat_data.z_A;
    motor_result.z_B = kinematics_result.fermat_data.z_B;
    motor_result.z_C = kinematics_result.fermat_data.z_C;
    motor_result.prismatic_joint = kinematics_result.joint_state_data.prismatic_joint;
    motor_result.roll_joint = rad_to_deg(kinematics_result.joint_state_data.roll_joint);   // Convert to degrees
    motor_result.pitch_joint = rad_to_deg(kinematics_result.joint_state_data.pitch_joint); // Convert to degrees
}

void MotorModule::calculate_first_segment_orientation(const Vector3& first_segment_pos, MotorResult& motor_result) {
    // Run orientation module on first segment position
    OrientationResult orientation_result = OrientationModule::calculate(first_segment_pos);
    
    // Extract Final UVW frame (U''V''W'')
    motor_result.uvw_origin = orientation_result.final_frame.origin;
    motor_result.uvw_u_axis = orientation_result.final_frame.u_axis;
    motor_result.uvw_v_axis = orientation_result.final_frame.v_axis;
    motor_result.uvw_w_axis = orientation_result.final_frame.w_axis;
}

void MotorModule::transform_segments_to_uvw_aligned(MotorResult& motor_result) {
    // Clear any existing transformed data
    motor_result.transformed_segment_numbers.clear();
    motor_result.transformed_segment_positions.clear();
    
    // Need at least 2 segments (first segment + at least one more)
    if (motor_result.segment_positions.size() < 2) {
        return;
    }
    
    // Get first segment position for translation
    Vector3 first_segment_pos = motor_result.first_segment_position;
    
    // Create rotation matrix to align UVW with XYZ
    double rotation_matrix[3][3];
    create_uvw_to_xyz_rotation_matrix(motor_result.uvw_u_axis, 
                                    motor_result.uvw_v_axis, 
                                    motor_result.uvw_w_axis, 
                                    rotation_matrix);
    
    // Transform segments 2 through N (skip segment 1 at index 0)
    for (size_t i = 1; i < motor_result.segment_positions.size(); ++i) {
        // Step 1: Translate so first segment is at origin
        Vector3 translated_pos = motor_result.segment_positions[i] - first_segment_pos;
        
        // Step 2: Rotate to align UVW with XYZ
        Vector3 transformed_pos = apply_rotation_matrix(translated_pos, rotation_matrix);
        
        // Store transformed data
        motor_result.transformed_segment_numbers.push_back(motor_result.segment_numbers[i]);
        motor_result.transformed_segment_positions.push_back(transformed_pos);
    }
}

void MotorModule::create_uvw_to_xyz_rotation_matrix(const Vector3& u_axis, const Vector3& v_axis, const Vector3& w_axis, double rotation_matrix[3][3]) {
    // Create the rotation matrix from UVW to XYZ
    // The columns of this matrix are the UVW axes expressed in XYZ coordinates
    double uvw_to_xyz[3][3] = {
        {u_axis.x, v_axis.x, w_axis.x},
        {u_axis.y, v_axis.y, w_axis.y},
        {u_axis.z, v_axis.z, w_axis.z}
    };
    
    // We want the inverse transformation (to align UVW with XYZ)
    // Since the matrix is orthogonal, inverse = transpose
    rotation_matrix[0][0] = uvw_to_xyz[0][0]; rotation_matrix[0][1] = uvw_to_xyz[1][0]; rotation_matrix[0][2] = uvw_to_xyz[2][0];
    rotation_matrix[1][0] = uvw_to_xyz[0][1]; rotation_matrix[1][1] = uvw_to_xyz[1][1]; rotation_matrix[1][2] = uvw_to_xyz[2][1];
    rotation_matrix[2][0] = uvw_to_xyz[0][2]; rotation_matrix[2][1] = uvw_to_xyz[1][2]; rotation_matrix[2][2] = uvw_to_xyz[2][2];
}

Vector3 MotorModule::apply_rotation_matrix(const Vector3& vector, const double rotation_matrix[3][3]) {
    return Vector3(
        rotation_matrix[0][0] * vector.x + rotation_matrix[0][1] * vector.y + rotation_matrix[0][2] * vector.z,
        rotation_matrix[1][0] * vector.x + rotation_matrix[1][1] * vector.y + rotation_matrix[1][2] * vector.z,
        rotation_matrix[2][0] * vector.x + rotation_matrix[2][1] * vector.y + rotation_matrix[2][2] * vector.z
    );
}

void MotorModule::calculate_second_segment_kinematics(MotorResult& motor_result) {
    // Use the first transformed segment (which is segment 2')
    if (motor_result.transformed_segment_positions.empty()) {
        return;
    }
    
    Vector3 second_segment_pos = motor_result.transformed_segment_positions[0];
    motor_result.second_segment_position = second_segment_pos;
    
    // Run kinematics module on second segment position
    KinematicsResult kinematics_result = KinematicsModule::calculate(second_segment_pos);
    
    // Extract required data
    motor_result.second_z_A = kinematics_result.fermat_data.z_A;
    motor_result.second_z_B = kinematics_result.fermat_data.z_B;
    motor_result.second_z_C = kinematics_result.fermat_data.z_C;
    motor_result.second_prismatic_joint = kinematics_result.joint_state_data.prismatic_joint;
    motor_result.second_roll_joint = rad_to_deg(kinematics_result.joint_state_data.roll_joint);   // Convert to degrees
    motor_result.second_pitch_joint = rad_to_deg(kinematics_result.joint_state_data.pitch_joint); // Convert to degrees
}

void MotorModule::calculate_second_segment_orientation(MotorResult& motor_result) {
    // Use the first transformed segment (which is segment 2')
    if (motor_result.transformed_segment_positions.empty()) {
        return;
    }
    
    Vector3 second_segment_pos = motor_result.transformed_segment_positions[0];
    
    // Run orientation module on second segment position
    OrientationResult orientation_result = OrientationModule::calculate(second_segment_pos);
    
    // Extract Final UVW frame (U''V''W'') for second segment
    motor_result.second_uvw_origin = orientation_result.final_frame.origin;
    motor_result.second_uvw_u_axis = orientation_result.final_frame.u_axis;
    motor_result.second_uvw_v_axis = orientation_result.final_frame.v_axis;
    motor_result.second_uvw_w_axis = orientation_result.final_frame.w_axis;
}

void MotorModule::transform_segments_to_second_level_uvw_aligned(MotorResult& motor_result) {
    // Clear any existing second-level transformed data
    motor_result.second_level_transformed_segment_numbers.clear();
    motor_result.second_level_transformed_segment_positions.clear();
    
    // Need at least 2 transformed segments (S2' + at least one more)
    if (motor_result.transformed_segment_positions.size() < 2) {
        return;
    }
    
    // Get second segment position (S2') for translation
    Vector3 second_segment_pos = motor_result.second_segment_position;
    
    // Create rotation matrix to align S2' UVW with XYZ
    double rotation_matrix[3][3];
    create_uvw_to_xyz_rotation_matrix(motor_result.second_uvw_u_axis, 
                                    motor_result.second_uvw_v_axis, 
                                    motor_result.second_uvw_w_axis, 
                                    rotation_matrix);
    
    // Transform segments 3' through N' (skip segment 2' at index 0)
    for (size_t i = 1; i < motor_result.transformed_segment_positions.size(); ++i) {
        // Step 1: Translate so second segment (S2') is at origin
        Vector3 translated_pos = motor_result.transformed_segment_positions[i] - second_segment_pos;
        
        // Step 2: Rotate to align S2' UVW with XYZ
        Vector3 transformed_pos = apply_rotation_matrix(translated_pos, rotation_matrix);
        
        // Store second-level transformed data
        motor_result.second_level_transformed_segment_numbers.push_back(motor_result.transformed_segment_numbers[i]);
        motor_result.second_level_transformed_segment_positions.push_back(transformed_pos);
    }
}

void MotorModule::calculate_third_segment_kinematics(MotorResult& motor_result) {
    // Use the first second-level transformed segment (which is segment 3'')
    if (motor_result.second_level_transformed_segment_positions.empty()) {
        return;
    }
    
    Vector3 third_segment_pos = motor_result.second_level_transformed_segment_positions[0];
    motor_result.third_segment_position = third_segment_pos;
    
    // Run kinematics module on third segment position
    KinematicsResult kinematics_result = KinematicsModule::calculate(third_segment_pos);
    
    // Extract required data
    motor_result.third_z_A = kinematics_result.fermat_data.z_A;
    motor_result.third_z_B = kinematics_result.fermat_data.z_B;
    motor_result.third_z_C = kinematics_result.fermat_data.z_C;
    motor_result.third_prismatic_joint = kinematics_result.joint_state_data.prismatic_joint;
    motor_result.third_roll_joint = rad_to_deg(kinematics_result.joint_state_data.roll_joint);   // Convert to degrees
    motor_result.third_pitch_joint = rad_to_deg(kinematics_result.joint_state_data.pitch_joint); // Convert to degrees
}

void MotorModule::calculate_third_segment_orientation(MotorResult& motor_result) {
    // Use the first second-level transformed segment (which is segment 3'')
    if (motor_result.second_level_transformed_segment_positions.empty()) {
        return;
    }
    
    Vector3 third_segment_pos = motor_result.second_level_transformed_segment_positions[0];
    
    // Run orientation module on third segment position
    OrientationResult orientation_result = OrientationModule::calculate(third_segment_pos);
    
    // Extract Final UVW frame (U''V''W'') for third segment
    motor_result.third_uvw_origin = orientation_result.final_frame.origin;
    motor_result.third_uvw_u_axis = orientation_result.final_frame.u_axis;
    motor_result.third_uvw_v_axis = orientation_result.final_frame.v_axis;
    motor_result.third_uvw_w_axis = orientation_result.final_frame.w_axis;
}

} // namespace delta