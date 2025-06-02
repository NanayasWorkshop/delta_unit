#ifndef DELTA_CONSTANTS_HPP
#define DELTA_CONSTANTS_HPP

#include <cmath>

namespace delta {

// Robot Physical Constants
constexpr double ROBOT_RADIUS = 24.8;
constexpr double MIN_HEIGHT = 101.0;
constexpr double WORKING_HEIGHT = 11.5; 
constexpr double MOTOR_LIMIT = 11.0;
constexpr double WORKSPACE_CONE_ANGLE_RAD = M_PI / 6;  // 30 degrees

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