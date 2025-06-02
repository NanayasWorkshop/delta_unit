#include "joint_state.hpp"
#include "fermat_module.hpp"

namespace delta {

JointStateResult JointStateModule::calculate(const Vector3& direction_vector,
                                           const Vector3& fermat_point,
                                           double z_A, double z_B, double z_C) {
    
    // Calculate joint states
    double prismatic = calculate_prismatic_joint(fermat_point);
    double roll = calculate_roll_joint(direction_vector);
    double pitch = calculate_pitch_joint(direction_vector);
    
    return JointStateResult(prismatic, roll, pitch,
                           direction_vector, fermat_point,
                           z_A, z_B, z_C);
}

JointStateResult JointStateModule::calculate_from_fermat(const Vector3& direction_vector,
                                                       const FermatResult& fermat_result) {
    return calculate(direction_vector, 
                    fermat_result.fermat_point,
                    fermat_result.z_A, 
                    fermat_result.z_B, 
                    fermat_result.z_C);
}

// Actual implementations
double JointStateModule::calculate_prismatic_joint(const Vector3& fermat_point) {
    // Prismatic joint = 2 × Z value of Fermat point
    return 2.0 * fermat_point.z;
}

double JointStateModule::calculate_roll_joint(const Vector3& direction_vector) {
    // Normalize the direction vector
    Vector3 normalized = direction_vector.normalized();
    
    // Roll = -atan2(y, z) - rotation around X-axis
    return -std::atan2(normalized.y, normalized.z);
}

double JointStateModule::calculate_pitch_joint(const Vector3& direction_vector) {
    // Normalize the direction vector
    Vector3 normalized = direction_vector.normalized();
    
    // Pitch = atan2(x, z) - rotation around Y-axis
    return std::atan2(normalized.x, normalized.z);
}

} // namespace delta