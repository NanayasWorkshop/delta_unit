#ifndef DELTA_MOTOR_MODULE_HPP
#define DELTA_MOTOR_MODULE_HPP

#include "math_utils.hpp"
#include "fabrik_solver.hpp"
#include "kinematics_module.hpp"
#include "orientation_module.hpp"
#include <vector>
#include <optional>

namespace delta {

// Data for each level of transformation/analysis
struct LevelData {
    // --- Data for the 'base' segment of this level (e.g., S1, S2', S3'') ---
    Vector3 base_segment_position;         // Position of the segment this level is based on (S1 global, S2' local, S3'' local, etc.)
    
    // Kinematics calculated for base_segment_position
    double z_A, z_B, z_C;                  // Motor Z positions
    double prismatic_joint;                 // Prismatic joint value
    double roll_joint;                      // Roll joint (degrees)
    double pitch_joint;                     // Pitch joint (degrees)
    
    // Orientation (Final UVW) calculated for base_segment_position
    Vector3 uvw_origin;                     // UVW origin
    Vector3 uvw_u_axis;                     // UVW U-axis
    Vector3 uvw_v_axis;                     // UVW V-axis
    Vector3 uvw_w_axis;                     // UVW W-axis
    
    // --- Segments transformed relative to this level's base_segment ---
    // e.g., for Level 0 (base S1): these are S2', S3', S4', ...
    // e.g., for Level 1 (base S2'): these are S3'', S4'', S5'', ...
    std::vector<int> transformed_segment_original_numbers; // Stores original segment numbers (e.g., 2, 3, 4...)
    std::vector<Vector3> transformed_segment_positions;   // Their positions after transformation

    LevelData() : z_A(0), z_B(0), z_C(0), prismatic_joint(0), roll_joint(0), pitch_joint(0) {}
};

struct MotorResult {
    Vector3 target_position;                    // Input target
    bool fabrik_converged;                      // FABRIK convergence status
    double fabrik_error;                        // FABRIK final error
    double solve_time_ms;                       // FABRIK solve time in milliseconds
    
    // Original segment data from FABRIK
    std::vector<int> original_segment_numbers;      // [1, 2, 3, ...]
    std::vector<Vector3> original_segment_positions; // Actual segment end-effector positions in global frame
    
    // NEW: FABRIK joint positions from solved chain
    std::vector<Vector3> fabrik_joint_positions;    // Joint positions from final_chain.joints
    
    // Data for each level of transformation
    std::vector<LevelData> levels;

    MotorResult(const Vector3& target, bool converged, double error, double time_ms = 0.0)
        : target_position(target), fabrik_converged(converged), fabrik_error(error), solve_time_ms(time_ms) {}
};

class MotorModule {
public:
    // Main interface: calculate motor positions for target
    static MotorResult calculate_motors(double target_x, double target_y, double target_z);
    static MotorResult calculate_motors(const Vector3& target_position);
    
    // NEW: Calculate motor positions with optional current joint positions
    static MotorResult calculate_motors(double target_x, double target_y, double target_z,
                                      const std::optional<std::vector<Vector3>>& current_joint_positions);
    static MotorResult calculate_motors(const Vector3& target_position,
                                      const std::optional<std::vector<Vector3>>& current_joint_positions);
    
private:
    // Extract segment positions from FABRIK result
    static void extract_original_segment_data(const FabrikSolutionResult& fabrik_result, MotorResult& motor_result);
    
    // NEW: Extract joint positions from FABRIK result
    static void extract_fabrik_joint_positions(const FabrikSolutionResult& fabrik_result, MotorResult& motor_result);
    
    // Helper: Create rotation matrix from UVW to XYZ alignment
    static void create_uvw_to_xyz_rotation_matrix(const Vector3& u_axis, const Vector3& v_axis, const Vector3& w_axis, Matrix3& rotation_matrix);
};

} // namespace delta

#endif // DELTA_MOTOR_MODULE_HPP