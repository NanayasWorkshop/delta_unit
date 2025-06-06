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
    
    KinematicsResult(const Vector3& end_effector, double prismatic_length,
                    const Vector3& transformed, const Vector3& original,
                    double angle, const FermatResult& fermat,
                    const JointStateResult& joint_state)
        : end_effector_position(end_effector), prismatic_joint_length(prismatic_length)
        , transformed_vector(transformed), original_input(original)
        , input_angle_from_z(angle), fermat_data(fermat), joint_state_data(joint_state) {}
};

class KinematicsModule {
public:
    // Main interface: input direction vector, get end-effector position
    static KinematicsResult calculate(double x, double y, double z);
    static KinematicsResult calculate(const Vector3& input_vector);
    
private:
    // Internal helper methods
    static double calculate_angle_from_z_axis(const Vector3& vector);
    static Vector3 calculate_end_effector_position(double prismatic_length, 
                                                  const Vector3& original_input);
};

} // namespace delta

#endif // DELTA_KINEMATICS_MODULE_HPP