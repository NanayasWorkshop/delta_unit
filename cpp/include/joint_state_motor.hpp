#ifndef DELTA_JOINT_STATE_MOTOR_HPP
#define DELTA_JOINT_STATE_MOTOR_HPP

#include "math_utils.hpp"
#include "fabrik_solver.hpp"
#include "kinematics_module.hpp"
#include "orientation_module.hpp"

namespace delta {

// Motor data for a single segment
struct SegmentMotorData {
    int segment_number;
    Vector3 fabrik_position;            // Original position from FABRIK
    Vector3 local_position;             // Position in local coordinate system
    
    // Motor positions
    double z_A, z_B, z_C;               // Base actuator positions
    double prismatic_joint;             // Prismatic actuator
    double roll_joint, pitch_joint;     // Orientation joints
    
    // Coordinate frame data
    CoordinateFrame uvw_frame;          // UVW frame at this segment
    Matrix4x4 local_to_world;           // Transform from local to world coordinates
    
    SegmentMotorData(int seg_num, const Vector3& fabrik_pos)
        : segment_number(seg_num), fabrik_position(fabrik_pos), local_position(fabrik_pos)
        , z_A(0), z_B(0), z_C(0), prismatic_joint(0), roll_joint(0), pitch_joint(0) {}
};

struct JointStateMotorResult {
    Vector3 target_position;           // Input target
    Vector3 achieved_end_effector;     // What FABRIK achieved
    bool fabrik_converged;             // Did FABRIK converge?
    double fabrik_error;               // FABRIK final error
    int fabrik_iterations;             // How many iterations FABRIK used
    double solve_time_ms;              // Time to solve
    
    // FABRIK solution details
    FabrikSolutionResult fabrik_result; // Complete FABRIK result
    
    // Motor data for all segments
    std::vector<SegmentMotorData> all_segment_motors;  // Motor data for each segment
    
    JointStateMotorResult(const Vector3& target, const Vector3& achieved,
                         bool converged, double error, int iterations,
                         double time_ms, const FabrikSolutionResult& fabrik_res)
        : target_position(target), achieved_end_effector(achieved)
        , fabrik_converged(converged), fabrik_error(error)
        , fabrik_iterations(iterations), solve_time_ms(time_ms)
        , fabrik_result(fabrik_res) {}
};

class JointStateMotorModule {
public:
    // Main interface: sequential calculation (the ONLY correct method)
    static JointStateMotorResult calculate_motors(double target_x, double target_y, double target_z);
    static JointStateMotorResult calculate_motors(const Vector3& target_position);
    
    // Advanced interface: specify FABRIK configuration
    static JointStateMotorResult calculate_motors_advanced(const Vector3& target_position,
                                                          int num_segments,
                                                          double tolerance,
                                                          int max_iterations);
    
    // Utility: Check if target is reachable before solving
    static bool is_target_reachable(const Vector3& target_position, int num_segments = DEFAULT_ROBOT_SEGMENTS);
    
private:
    // Internal: Call FABRIK solver and process results
    static JointStateMotorResult solve_with_fabrik(const Vector3& target_position,
                                                   int num_segments,
                                                   double tolerance,
                                                   int max_iterations);
    
    // CORRECTED: Sequential calculation with proper coordinate system progression
    static std::vector<SegmentMotorData> calculate_all_segment_motors_sequential(
        const std::vector<SegmentEndEffectorData>& segment_end_effectors);
    
    // Transform coordinates using relative positioning instead of absolute transformation
    static Vector3 transform_to_local_coordinates(const Vector3& world_position, 
                                                  const Vector3& reference_position,
                                                  const CoordinateFrame& reference_frame);
    
    static SegmentMotorData calculate_single_segment_motors(
        int segment_number,
        const Vector3& fabrik_position,
        const Vector3& local_position);
};

// Convenience functions
namespace joint_state_utils {
    
    // Quick solve with default parameters
    JointStateMotorResult solve_target(double x, double y, double z);
    
    // Batch solve multiple targets
    std::vector<JointStateMotorResult> solve_multiple_targets(const std::vector<Vector3>& targets);
    
    // Test workspace reach
    bool test_workspace_point(double x, double y, double z);
}

} // namespace delta

#endif // DELTA_JOINT_STATE_MOTOR_HPP