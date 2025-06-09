#include "joint_state.hpp"
#include "fermat_module.hpp"
#include <chrono>
#include <cmath>

namespace delta {

JointStateResult JointStateSolver::calculate(const Vector3& direction_vector,
                                           const Vector3& fermat_point,
                                           double z_A, double z_B, double z_C) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Validate input
    if (!is_input_valid(direction_vector, fermat_point)) {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        return JointStateResult::failed(direction_vector, duration.count() / 1000.0);
    }
    
    // Perform calculation
    JointStateResult result = calculate_internal(direction_vector, fermat_point, z_A, z_B, z_C);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    result.computation_time_ms = duration.count() / 1000.0;
    
    return result;
}

JointStateResult JointStateSolver::calculate_from_fermat(const Vector3& direction_vector,
                                                       const FermatResult& fermat_result) {
    // Check if fermat calculation was successful
    if (!fermat_result.calculation_successful) {
        return JointStateResult::failed(direction_vector, 0.0);
    }
    
    return calculate(direction_vector, 
                    fermat_result.fermat_point,
                    fermat_result.z_A, 
                    fermat_result.z_B, 
                    fermat_result.z_C);
}

bool JointStateSolver::is_input_valid(const Vector3& direction_vector, const Vector3& fermat_point) {
    // Check direction vector
    if (direction_vector.norm() < EPSILON_MATH) {
        return false;
    }
    
    // Check fermat point (basic sanity check)
    if (fermat_point.norm() > 10000.0) {  // Unreasonably large
        return false;
    }
    
    return true;
}

JointStateResult JointStateSolver::calculate_internal(const Vector3& direction_vector,
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

// Private calculation methods (unchanged core logic)
double JointStateSolver::calculate_prismatic_joint(const Vector3& fermat_point) {
    return 2.0 * fermat_point.z();
}

double JointStateSolver::calculate_roll_joint(const Vector3& direction_vector) {
    Vector3 normalized = direction_vector.normalized();
    return -std::atan2(normalized.y(), normalized.z());
}

double JointStateSolver::calculate_pitch_joint(const Vector3& direction_vector) {
    Vector3 normalized = direction_vector.normalized();
    return std::atan2(normalized.x(), normalized.z());
}

} // namespace delta