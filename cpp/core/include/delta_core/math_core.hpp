#ifndef DELTA_CORE_MATH_CORE_HPP
#define DELTA_CORE_MATH_CORE_HPP

#include "types.hpp"
#include "constants.hpp"
#include <cmath>
#include <algorithm>

namespace delta::core::math {

// ============================================================================
// VECTOR OPERATIONS
// ============================================================================

class VectorOps {
public:
    // Basic vector operations
    static double angle_between(const Vector3& a, const Vector3& b);
    static Vector3 project_onto(const Vector3& vector, const Vector3& onto);
    static Vector3 reject_from(const Vector3& vector, const Vector3& from);
    
    // Distance and magnitude
    static double distance(const Vector3& a, const Vector3& b);
    static double distance_squared(const Vector3& a, const Vector3& b);
    
    // Vector validation
    static bool is_unit_vector(const Vector3& v, double tolerance = math::EPSILON);
    static bool is_zero_vector(const Vector3& v, double tolerance = math::EPSILON);
    static bool are_parallel(const Vector3& a, const Vector3& b, double tolerance = math::EPSILON);
    static bool are_perpendicular(const Vector3& a, const Vector3& b, double tolerance = math::EPSILON);
    
    // Safe normalization (returns zero vector if input is too small)
    static Vector3 safe_normalize(const Vector3& v, double min_norm = math::EPSILON);
    
    // Clamp vector magnitude
    static Vector3 clamp_magnitude(const Vector3& v, double max_magnitude);
    static Vector3 clamp_magnitude(const Vector3& v, double min_magnitude, double max_magnitude);
};

// ============================================================================
// ROTATION OPERATIONS
// ============================================================================

class RotationOps {
public:
    // Rodrigues rotation formula
    static Vector3 rodrigues_rotation(const Vector3& v, const Vector3& axis, double angle);
    
    // Rotation matrix creation
    static Matrix3 rotation_matrix_from_axis_angle(const Vector3& axis, double angle);
    static Matrix3 rotation_matrix_from_vectors(const Vector3& from, const Vector3& to);
    
    // Quaternion operations
    static Quaternion quaternion_from_axis_angle(const Vector3& axis, double angle);
    static Quaternion quaternion_from_vectors(const Vector3& from, const Vector3& to);
    
    // Rotation validation
    static bool is_rotation_matrix(const Matrix3& matrix, double tolerance = math::EPSILON);
    
    // Decompose rotation
    static Vector3 rotation_to_euler_xyz(const Matrix3& rotation);
    static Vector3 rotation_to_axis_angle(const Matrix3& rotation);
};

// ============================================================================
// GEOMETRIC OPERATIONS
// ============================================================================

class GeometryOps {
public:
    // Plane operations
    static Vector3 calculate_plane_normal(const Vector3& a, const Vector3& b, const Vector3& c);
    static double point_to_plane_distance(const Vector3& point, const Vector3& plane_point, const Vector3& plane_normal);
    static Vector3 project_point_onto_plane(const Vector3& point, const Vector3& plane_point, const Vector3& plane_normal);
    static Vector3 mirror_point_across_plane(const Vector3& point, const Vector3& plane_point, const Vector3& plane_normal);
    
    // Line operations
    static Vector3 closest_point_on_line(const Vector3& point, const Vector3& line_start, const Vector3& line_direction);
    static double point_to_line_distance(const Vector3& point, const Vector3& line_start, const Vector3& line_direction);
    
    // Intersection calculations
    static double calculate_z_intersection(double base_x, double base_y, const Vector3& normal);
    static Result<Vector3> line_plane_intersection(const Vector3& line_start, const Vector3& line_direction,
                                                  const Vector3& plane_point, const Vector3& plane_normal);
    
    // Cone constraint operations
    static Vector3 project_direction_onto_cone(const Vector3& desired_direction,
                                              const Vector3& cone_axis_normalized,
                                              double cone_half_angle_rad);
    
    // Validation
    static bool is_point_on_plane(const Vector3& point, const Vector3& plane_point, 
                                 const Vector3& plane_normal, double tolerance = math::EPSILON);
};

// ============================================================================
// NUMERICAL UTILITIES
// ============================================================================

class NumericalOps {
public:
    // Safe mathematical operations
    static double safe_acos(double value);
    static double safe_asin(double value);
    static double safe_sqrt(double value);
    static double safe_divide(double numerator, double denominator, double default_value = 0.0);
    
    // Comparison with tolerance
    static bool nearly_equal(double a, double b, double tolerance = math::EPSILON);
    static bool nearly_zero(double value, double tolerance = math::EPSILON);
    
    // Clamping and wrapping
    static double clamp(double value, double min_value, double max_value);
    static double wrap_angle(double angle);  // Wrap to [-π, π]
    static double wrap_angle_0_2pi(double angle);  // Wrap to [0, 2π]
    
    // Interpolation
    static double lerp(double a, double b, double t);
    static Vector3 lerp(const Vector3& a, const Vector3& b, double t);
    static Vector3 slerp(const Vector3& a, const Vector3& b, double t);  // Spherical linear interpolation
    
    // Statistics
    static double mean(const std::vector<double>& values);
    static double standard_deviation(const std::vector<double>& values);
    static Vector3 centroid(const std::vector<Vector3>& points);
};

// ============================================================================
// BASE POSITION UTILITIES
// ============================================================================

class BasePositions {
public:
    // Get base positions in XY plane using constants
    static Vector3 get_base_position_A();
    static Vector3 get_base_position_B();
    static Vector3 get_base_position_C();
    
    // Get all base positions at once
    static std::vector<Vector3> get_all_base_positions();
    
    // Validate base configuration
    static bool validate_base_triangle(double tolerance = math::EPSILON);
    static double calculate_base_triangle_area();
};

// ============================================================================
// COORDINATE FRAME OPERATIONS
// ============================================================================

class FrameOps {
public:
    // Frame creation
    static CoordinateFrame create_frame_from_z_axis(const Vector3& origin, const Vector3& z_axis);
    static CoordinateFrame create_frame_from_two_vectors(const Vector3& origin, 
                                                        const Vector3& primary_axis,
                                                        const Vector3& secondary_hint);
    
    // Frame transformations
    static CoordinateFrame transform_frame(const CoordinateFrame& frame, const Matrix4& transform);
    static Matrix4 frame_to_frame_transform(const CoordinateFrame& from, const CoordinateFrame& to);
    
    // Frame validation
    static bool is_valid_frame(const CoordinateFrame& frame, double tolerance = math::EPSILON);
    static CoordinateFrame orthonormalize_frame(const CoordinateFrame& frame);
};

// ============================================================================
// INLINE IMPLEMENTATIONS (for performance-critical functions)
// ============================================================================

// Simple distance calculation (inlined for performance)
inline double VectorOps::distance_squared(const Vector3& a, const Vector3& b) {
    return (a - b).squaredNorm();
}

inline double VectorOps::distance(const Vector3& a, const Vector3& b) {
    return (a - b).norm();
}

inline bool VectorOps::is_zero_vector(const Vector3& v, double tolerance) {
    return v.norm() < tolerance;
}

inline bool NumericalOps::nearly_equal(double a, double b, double tolerance) {
    return std::abs(a - b) < tolerance;
}

inline bool NumericalOps::nearly_zero(double value, double tolerance) {
    return std::abs(value) < tolerance;
}

inline double NumericalOps::clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
}

} // namespace delta::core::math

// Backward compatibility (legacy functions from math_utils.hpp)
namespace delta {
    // Legacy base position functions
    inline Vector3 get_base_position_A() { return core::math::BasePositions::get_base_position_A(); }
    inline Vector3 get_base_position_B() { return core::math::BasePositions::get_base_position_B(); }
    inline Vector3 get_base_position_C() { return core::math::BasePositions::get_base_position_C(); }
    
    // Legacy intersection function
    inline double calculate_z_intersection(double base_x, double base_y, const Vector3& normal) {
        return core::math::GeometryOps::calculate_z_intersection(base_x, base_y, normal);
    }
}

#endif // DELTA_CORE_MATH_CORE_HPP