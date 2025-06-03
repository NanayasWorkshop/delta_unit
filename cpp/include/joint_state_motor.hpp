#ifndef DELTA_JOINT_STATE_MOTOR_HPP
#define DELTA_JOINT_STATE_MOTOR_HPP

#include "math_utils.hpp"
#include "fabrik_solver.hpp"

namespace delta {

struct JointStateMotorResult {
    Vector3 target_position;           // Input target
    Vector3 achieved_end_effector;     // What FABRIK achieved
    bool fabrik_converged;             // Did FABRIK converge?
    double fabrik_error;               // FABRIK final error
    int fabrik_iterations;             // How many iterations FABRIK used
    double solve_time_ms;              // Time to solve
    
    // FABRIK solution details
    FabrikSolutionResult fabrik_result; // Complete FABRIK result
    
    // TODO: Later will add real motor positions here
    // std::vector<double> motor_positions;
    // std::vector<double> joint_angles;
    
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
    // Main interface: input target, get motor positions
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
    
    // TODO: Later will add motor position calculation methods
    // static std::vector<double> convert_to_motor_positions(const FabrikSolutionResult& fabrik_result);
    // static std::vector<double> calculate_joint_angles(const FabrikSolutionResult& fabrik_result);
};

// Convenience functions for common use cases
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