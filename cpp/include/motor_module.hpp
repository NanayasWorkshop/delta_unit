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
    
    // UVW-aligned transformed segments (segments 2-8 transformed)
    std::vector<int> transformed_segment_numbers;      // [2, 3, 4, ...]
    std::vector<Vector3> transformed_segment_positions; // Segments 2-8 after UVW->XYZ transformation
    
    // Second segment kinematics data (for transformed Segment 2')
    Vector3 second_segment_position;                   // Position of second segment (transformed)
    double second_z_A, second_z_B, second_z_C;        // Motor Z positions for segment 2'
    double second_prismatic_joint;                     // Prismatic joint value for segment 2'
    double second_roll_joint;                          // Roll joint (degrees) for segment 2'
    double second_pitch_joint;                         // Pitch joint (degrees) for segment 2'
    
    // Second segment orientation data (Final UVW for segment 2')
    Vector3 second_uvw_origin;                         // U''V''W'' origin for segment 2'
    Vector3 second_uvw_u_axis;                         // U''V''W'' U-axis for segment 2'
    Vector3 second_uvw_v_axis;                         // U''V''W'' V-axis for segment 2'
    Vector3 second_uvw_w_axis;                         // U''V''W'' W-axis for segment 2'
    
    // Second-level transformed segments (segments 3'-8' transformed relative to S2')
    std::vector<int> second_level_transformed_segment_numbers;      // [3, 4, 5, ...]
    std::vector<Vector3> second_level_transformed_segment_positions; // Segments 3'-8' after S2' UVW->XYZ transformation
    
    // Third segment kinematics data (for transformed Segment 3'')
    Vector3 third_segment_position;                    // Position of third segment (transformed)
    double third_z_A, third_z_B, third_z_C;          // Motor Z positions for segment 3''
    double third_prismatic_joint;                      // Prismatic joint value for segment 3''
    double third_roll_joint;                           // Roll joint (degrees) for segment 3''
    double third_pitch_joint;                          // Pitch joint (degrees) for segment 3''
    
    // Third segment orientation data (Final UVW for segment 3'')
    Vector3 third_uvw_origin;                          // U''V''W'' origin for segment 3''
    Vector3 third_uvw_u_axis;                          // U''V''W'' U-axis for segment 3''
    Vector3 third_uvw_v_axis;                          // U''V''W'' V-axis for segment 3''
    Vector3 third_uvw_w_axis;                          // U''V''W'' W-axis for segment 3''
    
    MotorResult(const Vector3& target, bool converged, double error)
        : target_position(target), fabrik_converged(converged), fabrik_error(error)
        , z_A(0), z_B(0), z_C(0), prismatic_joint(0), roll_joint(0), pitch_joint(0)
        , second_z_A(0), second_z_B(0), second_z_C(0), second_prismatic_joint(0), second_roll_joint(0), second_pitch_joint(0)
        , third_z_A(0), third_z_B(0), third_z_C(0), third_prismatic_joint(0), third_roll_joint(0), third_pitch_joint(0) {}
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
    
    // Transform segments 2-8 to UVW-aligned coordinate system
    static void transform_segments_to_uvw_aligned(MotorResult& motor_result);
    
    // Calculate kinematics for second segment (transformed)
    static void calculate_second_segment_kinematics(MotorResult& motor_result);
    
    // Calculate orientation for second segment (transformed)
    static void calculate_second_segment_orientation(MotorResult& motor_result);
    
    // Transform segments 3'-8' to second-level UVW-aligned coordinate system (relative to S2')
    static void transform_segments_to_second_level_uvw_aligned(MotorResult& motor_result);
    
    // Calculate kinematics for third segment (second-level transformed)
    static void calculate_third_segment_kinematics(MotorResult& motor_result);
    
    // Calculate orientation for third segment (second-level transformed)
    static void calculate_third_segment_orientation(MotorResult& motor_result);
    
    // Helper: Create rotation matrix from UVW to XYZ alignment
    static void create_uvw_to_xyz_rotation_matrix(const Vector3& u_axis, const Vector3& v_axis, const Vector3& w_axis, double rotation_matrix[3][3]);
    
    // Helper: Apply rotation matrix to a vector
    static Vector3 apply_rotation_matrix(const Vector3& vector, const double rotation_matrix[3][3]);
};

} // namespace delta

#endif // DELTA_MOTOR_MODULE_HPP