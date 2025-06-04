#ifndef DELTA_CORE_CONSTANTS_HPP
#define DELTA_CORE_CONSTANTS_HPP

#include <cmath>

namespace delta::core {

// Robot Physical Constants
namespace robot {
    constexpr double RADIUS = 24.8;
    constexpr double MIN_HEIGHT = 101.0;
    constexpr double WORKING_HEIGHT = 11.5; 
    constexpr double MOTOR_LIMIT = 11.0;
    constexpr int DEFAULT_SEGMENTS = 8;
}

// FABRIK Algorithm Constants
namespace fabrik {
    constexpr double TOLERANCE = 0.01;
    constexpr int MAX_ITERATIONS = 100;
    constexpr int MAX_CYCLES = 50;  // DEFAULT_MAX_BACKWARD_FORWARD_CYCLES
    constexpr double SPHERICAL_JOINT_CONE_ANGLE_RAD = 2.0 * M_PI / 3.0;  // 120 degrees
}

// Mathematical Constants
namespace math {
    constexpr double EPSILON = 1e-9;
    constexpr double PI = M_PI;
    constexpr double TWO_PI = 2.0 * M_PI;
    constexpr double HALF_PI = M_PI / 2.0;
}

// Geometry Constants - Base actuator positions (angles in radians)
namespace geometry {
    constexpr double BASE_A_ANGLE = M_PI / 2.0;        // 90 degrees (top)
    constexpr double BASE_B_ANGLE = -M_PI / 6.0;       // -30 degrees (bottom right)
    constexpr double BASE_C_ANGLE = -5.0 * M_PI / 6.0; // -150 degrees (bottom left)
}

// Solver Configuration Presets
namespace solver_presets {
    // Fast solving (loose tolerance, fewer iterations)
    namespace fast {
        constexpr double TOLERANCE = fabrik::TOLERANCE * 10.0;  // 0.1
        constexpr int MAX_ITERATIONS = fabrik::MAX_ITERATIONS / 5;  // 20
        constexpr int MAX_CYCLES = fabrik::MAX_CYCLES / 5;  // 10
    }
    
    // Precise solving (tight tolerance, more iterations)
    namespace precise {
        constexpr double TOLERANCE = fabrik::TOLERANCE / 10.0;  // 0.001
        constexpr int MAX_ITERATIONS = fabrik::MAX_ITERATIONS * 2;  // 200
        constexpr int MAX_CYCLES = fabrik::MAX_CYCLES * 2;  // 100
    }
}

// Utility functions
constexpr double rad_to_deg(double rad) {
    return rad * 180.0 / math::PI;
}

constexpr double deg_to_rad(double deg) {
    return deg * math::PI / 180.0;
}

// Validation functions
constexpr bool is_valid_tolerance(double tolerance) {
    return tolerance > 0.0 && tolerance < 1.0;
}

constexpr bool is_valid_iterations(int iterations) {
    return iterations > 0 && iterations <= 10000;
}

constexpr bool is_valid_segments(int segments) {
    return segments > 0 && segments <= 100;
}

} // namespace delta::core

// Backward compatibility aliases (to be removed in future versions)
namespace delta {
    // Legacy constants for smooth transition
    constexpr double ROBOT_RADIUS = core::robot::RADIUS;
    constexpr double MIN_HEIGHT = core::robot::MIN_HEIGHT;
    constexpr double WORKING_HEIGHT = core::robot::WORKING_HEIGHT;
    constexpr double MOTOR_LIMIT = core::robot::MOTOR_LIMIT;
    constexpr int DEFAULT_ROBOT_SEGMENTS = core::robot::DEFAULT_SEGMENTS;
    
    constexpr double FABRIK_TOLERANCE = core::fabrik::TOLERANCE;
    constexpr int FABRIK_MAX_ITERATIONS = core::fabrik::MAX_ITERATIONS;
    constexpr double SPHERICAL_JOINT_CONE_ANGLE_RAD = core::fabrik::SPHERICAL_JOINT_CONE_ANGLE_RAD;
    constexpr double EPSILON_MATH = core::math::EPSILON;
    
    constexpr double BASE_A_ANGLE = core::geometry::BASE_A_ANGLE;
    constexpr double BASE_B_ANGLE = core::geometry::BASE_B_ANGLE;
    constexpr double BASE_C_ANGLE = core::geometry::BASE_C_ANGLE;
    
    constexpr double rad_to_deg(double rad) { return core::rad_to_deg(rad); }
    constexpr double deg_to_rad(double deg) { return core::deg_to_rad(deg); }
}

#endif // DELTA_CORE_CONSTANTS_HPP