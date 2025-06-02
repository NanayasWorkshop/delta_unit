#ifndef DELTA_JOINT_STATE_HPP
#define DELTA_JOINT_STATE_HPP

#include "math_utils.hpp"
#include "fermat_module.hpp"

namespace delta {

struct JointStateResult {
    double prismatic_joint;                // Prismatic joint position
    double roll_joint;                     // Roll joint (rotation around X-axis)
    double pitch_joint;                    // Pitch joint (rotation around Y-axis)
    Vector3 direction_vector;              // Input direction vector
    Vector3 fermat_point;                  // Input fermat point
    double z_A, z_B, z_C;                 // Input Z positions
    
    JointStateResult(double prismatic, double roll, double pitch, 
                    const Vector3& direction, const Vector3& fermat,
                    double zA, double zB, double zC)
        : prismatic_joint(prismatic), roll_joint(roll), pitch_joint(pitch)
        , direction_vector(direction), fermat_point(fermat)
        , z_A(zA), z_B(zB), z_C(zC) {}
};

class JointStateModule {
public:
    // Main interface: calculate joint states from fermat results
    static JointStateResult calculate(const Vector3& direction_vector,
                                    const Vector3& fermat_point,
                                    double z_A, double z_B, double z_C);
    
    // Convenience method to work directly with FermatResult
    static JointStateResult calculate_from_fermat(const Vector3& direction_vector,
                                                const FermatResult& fermat_result);

private:
    // Internal calculation methods
    static double calculate_prismatic_joint(const Vector3& fermat_point);
    static double calculate_roll_joint(const Vector3& direction_vector);
    static double calculate_pitch_joint(const Vector3& direction_vector);
};

} // namespace delta

#endif // DELTA_JOINT_STATE_HPP