#ifndef DELTA_KINEMATICS_MODULE_HPP
#define DELTA_KINEMATICS_MODULE_HPP

#include "../core/math_utils.hpp"
#include "fermat_module.hpp"
#include "joint_state.hpp"

namespace delta {

struct KinematicsResult {
    Vector3 end_effector_position;      // Final end-effector position
    double prismatic_joint_length;      // Prismatic joint length from joint state
    Vector3 transformed_vector;         // Input vector (normalized, no half-angle)
    Vector3 original_input;             // Original input vector
    double input_angle_from_z;          // Angle between input and +Z axis (radians)
    
    // Reference data from internal calculations
    FermatResult fermat_data;           // Fermat calculation results
    JointStateResult joint_state_data;  // Joint state calculation results
    
    // NEW: Timing and validation (Step 1.2)
    double computation_time_ms;         // Total computation time
    bool calculation_successful;        // Whether calculation completed successfully
    
    KinematicsResult(const Vector3& end_effector, double prismatic_length,
                    const Vector3& transformed, const Vector3& original,
                    double angle, const FermatResult& fermat,
                    const JointStateResult& joint_state, double time_ms = 0.0, bool success = true)
        : end_effector_position(end_effector), prismatic_joint_length(prismatic_length)
        , transformed_vector(transformed), original_input(original)
        , input_angle_from_z(angle), fermat_data(fermat), joint_state_data(joint_state)
        , computation_time_ms(time_ms), calculation_successful(success) {}
    
    // Failed result constructor
    static KinematicsResult failed(const Vector3& input = Vector3(0,0,0), double time_ms = 0.0) {
        FermatResult failed_fermat = FermatResult::failed();
        JointStateResult failed_joint = JointStateResult::failed(input);
        return KinematicsResult(Vector3(0,0,0), 0.0, Vector3(0,0,1), input, 0.0, 
                               failed_fermat, failed_joint, time_ms, false);
    }
};

// NEW: Step 1.2 - Enhanced KinematicsModule with timing and validation
class KinematicsModule {
public:
    // Main interface: input direction vector, get end-effector position with timing
    static KinematicsResult calculate(double x, double y, double z);
    static KinematicsResult calculate(const Vector3& input_vector);
    
    // NEW: Validation (Step 1.2)
    static bool is_input_valid(const Vector3& input_vector);
    
    // NEW: Configuration (Step 1.2 - for future extensions)
    static void set_use_half_angle_transform(bool enable) { use_half_angle_transform_ = enable; }
    static bool get_use_half_angle_transform() { return use_half_angle_transform_; }

private:
    // Core calculation with timing
    static KinematicsResult calculate_internal(const Vector3& input_vector);
    
    // Internal helper methods
    static double calculate_angle_from_z_axis(const Vector3& vector);
    static Vector3 calculate_end_effector_position(double prismatic_length, 
                                                  const Vector3& original_input);
    
    // Configuration
    static bool use_half_angle_transform_;  // Currently false (no half-angle transformation)
};

// NEW: Step 1.2 - Alias for clean naming
using KinematicsSolver = KinematicsModule;

} // namespace delta

#endif // DELTA_KINEMATICS_MODULE_HPP