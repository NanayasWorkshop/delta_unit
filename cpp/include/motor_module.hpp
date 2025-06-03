#ifndef DELTA_MOTOR_MODULE_HPP
#define DELTA_MOTOR_MODULE_HPP

#include "math_utils.hpp"
#include "fabrik_solver.hpp"
#include "kinematics_module.hpp"
#include "orientation_module.hpp"

namespace delta {

struct MotorResult {
    Vector3 target_position;                    // Input target
    bool fabrik_converged;                      // FABRIK convergence status
    double fabrik_error;                        // FABRIK final error
    
    // Segment data (only segment number and position)
    std::vector<int> segment_numbers;           // [1, 2, 3, ...]
    std::vector<Vector3> segment_positions;     // Actual segment end-effector positions
    
    // First segment kinematics data
    Vector3 first_segment_position;             // Position of first segment
    double z_A, z_B, z_C;                      // Motor Z positions
    double prismatic_joint;                     // Prismatic joint value
    double roll_joint;                          // Roll joint (degrees)
    double pitch_joint;                         // Pitch joint (degrees)
    
    // First segment orientation data (Final UVW)
    Vector3 uvw_origin;                         // U''V''W'' origin
    Vector3 uvw_u_axis;                         // U''V''W'' U-axis
    Vector3 uvw_v_axis;                         // U''V''W'' V-axis
    Vector3 uvw_w_axis;                         // U''V''W'' W-axis
    
    MotorResult(const Vector3& target, bool converged, double error)
        : target_position(target), fabrik_converged(converged), fabrik_error(error)
        , z_A(0), z_B(0), z_C(0), prismatic_joint(0), roll_joint(0), pitch_joint(0) {}
};

class MotorModule {
public:
    // Main interface: calculate motor positions for target
    static MotorResult calculate_motors(double target_x, double target_y, double target_z);
    static MotorResult calculate_motors(const Vector3& target_position);
    
private:
    // Extract segment positions from FABRIK result
    static void extract_segment_data(const FabrikSolutionResult& fabrik_result, MotorResult& motor_result);
    
    // Calculate kinematics for first segment
    static void calculate_first_segment_kinematics(const Vector3& first_segment_pos, MotorResult& motor_result);
    
    // Calculate orientation for first segment
    static void calculate_first_segment_orientation(const Vector3& first_segment_pos, MotorResult& motor_result);
};

} // namespace delta

#endif // DELTA_MOTOR_MODULE_HPP