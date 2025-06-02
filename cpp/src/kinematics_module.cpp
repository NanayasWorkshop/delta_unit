#include "kinematics_module.hpp"
#include <cmath>

namespace delta {

KinematicsResult KinematicsModule::calculate(double x, double y, double z) {
    return calculate(Vector3(x, y, z));
}

KinematicsResult KinematicsModule::calculate(const Vector3& input_vector) {
    // Step 1: Calculate angle between input vector and +Z axis
    double angle_from_z = calculate_angle_from_z_axis(input_vector);
    
    // Step 2: Create transformed vector at half the angle from +Z axis
    Vector3 transformed_vector = create_half_angle_vector(input_vector, angle_from_z);
    
    // Step 3: Call Fermat module with transformed vector
    FermatResult fermat_result = FermatModule::calculate(transformed_vector);
    
    // Step 4: Call Joint State module with fermat results
    JointStateResult joint_state_result = JointStateModule::calculate_from_fermat(
        transformed_vector, fermat_result);
    
    // Step 5: Calculate end-effector position using prismatic joint length and ORIGINAL input
    Vector3 end_effector = calculate_end_effector_position(
        joint_state_result.prismatic_joint, input_vector);  // Use original input, not transformed
    
    return KinematicsResult(
        end_effector,
        joint_state_result.prismatic_joint,
        transformed_vector,
        input_vector,
        angle_from_z,
        fermat_result,
        joint_state_result
    );
}

double KinematicsModule::calculate_angle_from_z_axis(const Vector3& vector) {
    Vector3 z_axis(0, 0, 1);
    Vector3 normalized_input = vector.normalized();
    
    // Calculate angle using dot product: cos(θ) = v·z / (|v|·|z|)
    // Since z_axis is unit vector and input is normalized: cos(θ) = v.z
    double dot_product = normalized_input.dot(z_axis);
    
    // Clamp to avoid numerical errors in acos
    dot_product = std::max(-1.0, std::min(1.0, dot_product));
    
    return std::acos(dot_product);
}

Vector3 KinematicsModule::create_half_angle_vector(const Vector3& input_vector, double angle_from_z) {
    Vector3 z_axis(0, 0, 1);
    Vector3 normalized_input = input_vector.normalized();
    
    // If input is already along Z axis, return Z axis
    if (angle_from_z < 1e-10) {
        return z_axis;
    }
    
    // Calculate half angle
    double half_angle = angle_from_z / 2.0;
    
    // Create vector at half angle between Z axis and input vector
    // Using spherical linear interpolation (slerp) concept
    
    // Find axis of rotation (cross product of z_axis and input)
    Vector3 rotation_axis(
        z_axis.y * normalized_input.z - z_axis.z * normalized_input.y,
        z_axis.z * normalized_input.x - z_axis.x * normalized_input.z,
        z_axis.x * normalized_input.y - z_axis.y * normalized_input.x
    );
    
    // If vectors are opposite, choose arbitrary perpendicular axis
    if (rotation_axis.norm() < 1e-10) {
        rotation_axis = Vector3(1, 0, 0);
    } else {
        rotation_axis = rotation_axis.normalized();
    }
    
    // Rotate Z axis by half_angle around rotation_axis
    // Using Rodrigues' rotation formula: v' = v*cos(θ) + (k×v)*sin(θ) + k*(k·v)*(1-cos(θ))
    double cos_half = std::cos(half_angle);
    double sin_half = std::sin(half_angle);
    
    Vector3 k_cross_z(
        rotation_axis.y * z_axis.z - rotation_axis.z * z_axis.y,
        rotation_axis.z * z_axis.x - rotation_axis.x * z_axis.z,
        rotation_axis.x * z_axis.y - rotation_axis.y * z_axis.x
    );
    
    double k_dot_z = rotation_axis.dot(z_axis);
    
    Vector3 result = z_axis * cos_half + 
                    k_cross_z * sin_half + 
                    rotation_axis * (k_dot_z * (1.0 - cos_half));
    
    return result.normalized();
}

Vector3 KinematicsModule::calculate_end_effector_position(double prismatic_length, 
                                                        const Vector3& original_input) {
    // Step 1: Calculate Joint H position
    Vector3 joint_H(0, 0, WORKING_HEIGHT);  // (0, 0, 11.5)
    
    // Step 2: Calculate vector length H→G
    double vector_length = MIN_HEIGHT + 2.0 * MOTOR_LIMIT + prismatic_length;
    
    // Step 3: Calculate Point G position
    Vector3 normalized_direction = original_input.normalized();
    Vector3 point_G = joint_H + normalized_direction * vector_length;
    
    // Step 4: Calculate midpoint between H and G
    Vector3 midpoint = (joint_H + point_G) * 0.5;
    
    // Step 5: Mirror base point (0,0,0) across plane through midpoint with normal = direction
    Vector3 base_point(0, 0, 0);
    
    // Mirror formula: P' = P - 2 * ((P - point_on_plane) · normal) * normal
    // Where: P = base_point, point_on_plane = midpoint, normal = normalized_direction
    Vector3 to_plane = base_point - midpoint;
    double distance_to_plane = to_plane.dot(normalized_direction);
    Vector3 end_effector = base_point - normalized_direction * (2.0 * distance_to_plane);
    
    return end_effector;
}

} // namespace delta