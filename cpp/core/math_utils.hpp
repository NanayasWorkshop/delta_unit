#ifndef DELTA_MATH_UTILS_HPP
#define DELTA_MATH_UTILS_HPP

#include <Eigen/Dense>
#include <cmath>
#include "../core/constants.hpp"

namespace delta {

// Use Eigen types as our foundation
using Vector3 = Eigen::Vector3d;
using Matrix3 = Eigen::Matrix3d;
using Matrix4 = Eigen::Matrix4d;

// Calculate base positions in XY plane
inline Vector3 get_base_position_A() {
    return Vector3(0, ROBOT_RADIUS, 0);
}

inline Vector3 get_base_position_B() {
    return Vector3(ROBOT_RADIUS * std::cos(-M_PI/6), ROBOT_RADIUS * std::sin(-M_PI/6), 0);
}

inline Vector3 get_base_position_C() {
    return Vector3(ROBOT_RADIUS * std::cos(-5*M_PI/6), ROBOT_RADIUS * std::sin(-5*M_PI/6), 0);
}

// Calculate Z intersection with plane defined by normal vector
double calculate_z_intersection(double base_x, double base_y, const Vector3& normal);

// Main calculation structure
struct FermatCalculation {
    Vector3 A_point, B_point, C_point;
    Vector3 AB, BC, CA;
    double side_a, side_b, side_c;
    double alpha, beta, gamma;
    double lambda_A, lambda_B, lambda_C;
    Vector3 fermat_point;
    
    FermatCalculation(const Vector3& direction);
};

} // namespace delta

#endif // DELTA_MATH_UTILS_HPP