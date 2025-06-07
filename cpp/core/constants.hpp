#ifndef DELTA_CONSTANTS_HPP
#define DELTA_CONSTANTS_HPP

#include <cmath>

namespace delta {

// Robot Physical Constants
constexpr double ROBOT_RADIUS = 24.8;
constexpr double MIN_HEIGHT = 101.0;
constexpr double WORKING_HEIGHT = 11.5; 
constexpr double MOTOR_LIMIT = 11.0;

// FABRIK Configuration Constants
constexpr int DEFAULT_ROBOT_SEGMENTS = 7;                              // Default number of stacked segments
constexpr double SPHERICAL_JOINT_CONE_ANGLE_RAD = 2.0 * M_PI / 3.0;   // 120 degrees

// FABRIK Solver Constants
constexpr double FABRIK_TOLERANCE = 0.01;                              // Convergence tolerance
constexpr int FABRIK_MAX_ITERATIONS = 100;                             // Maximum solver iterations
constexpr double EPSILON_MATH = 1e-9;                                  // Mathematical epsilon for floating point comparisons

// Spline and Collision Avoidance Constants
constexpr double SPLINE_THICKNESS = 48.0;                             // Robot thickness for collision detection (mm)
constexpr double COLLISION_SAFETY_MARGIN = 10.0;                      // Extra clearance beyond obstacle radius (mm)

// Collision Avoidance Performance Constants
constexpr double COLLISION_AVOIDANCE_TARGET_TIME_MS = 0.3;             // Target execution time
constexpr int COLLISION_MAX_SOLUTION_CANDIDATES = 7;                   // Maximum bending alternatives to try
constexpr double COLLISION_INFLUENCE_RADIUS = 50.0;                    // Gaussian falloff distance for bending
constexpr double COLLISION_GAUSSIAN_FALLOFF = 3.0;                     // Gaussian falloff sharpness

// Constraint Preservation Weights
constexpr double WEIGHT_TOTAL_MOVEMENT = 1.0;                          // Primary: minimize displacement
constexpr double WEIGHT_SPACING_DEVIATION = 0.5;                       // Secondary: preserve relative spacing  
constexpr double WEIGHT_LENGTH_DEVIATION = 0.3;                        // Tertiary: preserve total length

// Geometry Constants - Base actuator positions (angles in radians)
constexpr double BASE_A_ANGLE = M_PI / 2.0;                // 90 degrees (top)
constexpr double BASE_B_ANGLE = -M_PI / 6.0;               // -30 degrees (bottom right)  
constexpr double BASE_C_ANGLE = -5.0 * M_PI / 6.0;         // -150 degrees (bottom left)

// Utility functions
constexpr double rad_to_deg(double rad) {
    return rad * 180.0 / M_PI;
}

constexpr double deg_to_rad(double deg) {
    return deg * M_PI / 180.0;
}

} // namespace delta

#endif // DELTA_CONSTANTS_HPP