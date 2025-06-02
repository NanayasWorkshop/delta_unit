#ifndef DELTA_MATH_UTILS_HPP
#define DELTA_MATH_UTILS_HPP

#include <array>
#include <cmath>
#include "constants.hpp"

namespace delta {

// 3D Point/Vector structure
struct Vector3 {
    double x, y, z;
    
    Vector3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    
    Vector3 operator-(const Vector3& other) const {
        return Vector3(x - other.x, y - other.y, z - other.z);
    }
    
    Vector3 operator+(const Vector3& other) const {
        return Vector3(x + other.x, y + other.y, z + other.z);
    }
    
    Vector3 operator*(double scalar) const {
        return Vector3(x * scalar, y * scalar, z * scalar);
    }
    
    double dot(const Vector3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
    
    double norm() const {
        return std::sqrt(x*x + y*y + z*z);
    }
    
    Vector3 normalized() const {
        double n = norm();
        return (n > 1e-10) ? Vector3(x/n, y/n, z/n) : Vector3(0, 0, 0);
    }
};

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