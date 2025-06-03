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

} // namespace delta