#ifndef DELTA_JOINT_STATE_HPP
#define DELTA_JOINT_STATE_HPP

#include "../core/math_utils.hpp"
#include <chrono>

namespace delta {

// Forward declaration to avoid circular dependency
struct FermatResult;

struct JointStateResult {
    double prismatic_joint;                // Prismatic joint position
    double roll_joint;                     // Roll joint (rotation around X-axis)
    double pitch_joint;                    // Pitch joint (rotation around Y-axis)
    Vector3 direction_vector;              // Input direction vector
    Vector3 fermat_point;                  // Input fermat point
    double z_A, z_B, z_C;                 // Input Z positions
    double computation_time_ms;            // Time taken for calculation
    bool calculation_successful;           // Whether calculation completed successfully
    
    JointStateResult(double prismatic, double roll, double pitch, 
                    const Vector3& direction, const Vector3& fermat,
                    double zA, double zB, double zC, double time_ms = 0.0, bool success = true)
        : prismatic_joint(prismatic), roll_joint(roll), pitch_joint(pitch)
        , direction_vector(direction), fermat_point(fermat)
        , z_A(zA), z_B(zB), z_C(zC), computation_time_ms(time_ms), calculation_successful(success) {}
    
    // Failed result constructor
    static JointStateResult failed(const Vector3& direction = Vector3(0,0,0), double time_ms = 0.0) {
        return JointStateResult(0.0, 0.0, 0.0, direction, Vector3(0,0,0), 0.0, 0.0, 0.0, time_ms, false);
    }
};

class JointStateSolver {
public:
    // Main interface: calculate joint states from components
    static JointStateResult calculate(const Vector3& direction_vector,
                                    const Vector3& fermat_point,
                                    double z_A, double z_B, double z_C);
    
    // Convenience method to work directly with FermatResult
    static JointStateResult calculate_from_fermat(const Vector3& direction_vector,
                                                const FermatResult& fermat_result);
    
    // Validation
    static bool is_input_valid(const Vector3& direction_vector, const Vector3& fermat_point);

private:
    // Core calculation with timing
    static JointStateResult calculate_internal(const Vector3& direction_vector,
                                             const Vector3& fermat_point,
                                             double z_A, double z_B, double z_C);
    
    // Internal calculation methods
    static double calculate_prismatic_joint(const Vector3& fermat_point);
    static double calculate_roll_joint(const Vector3& direction_vector);
    static double calculate_pitch_joint(const Vector3& direction_vector);
};

// Backward compatibility alias
using JointStateModule = JointStateSolver;

} // namespace delta

#endif // DELTA_JOINT_STATE_HPP