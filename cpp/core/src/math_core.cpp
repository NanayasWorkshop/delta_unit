#include "delta_core/math_core.hpp"
#include "delta_core/error_handling.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace delta::core::math {

// ============================================================================
// VECTOR OPERATIONS IMPLEMENTATION
// ============================================================================

double VectorOps::angle_between(const Vector3& a, const Vector3& b) {
    Vector3 norm_a = safe_normalize(a);
    Vector3 norm_b = safe_normalize(b);
    
    if (is_zero_vector(norm_a) || is_zero_vector(norm_b)) {
        return 0.0;
    }
    
    double dot_product = norm_a.dot(norm_b);
    return NumericalOps::safe_acos(dot_product);
}

Vector3 VectorOps::project_onto(const Vector3& vector, const Vector3& onto) {
    double onto_norm_sq = onto.squaredNorm();
    if (onto_norm_sq < math::EPSILON) {
        return Vector3::Zero();
    }
    
    double projection_length = vector.dot(onto) / onto_norm_sq;
    return projection_length * onto;
}

Vector3 VectorOps::reject_from(const Vector3& vector, const Vector3& from) {
    return vector - project_onto(vector, from);
}

bool VectorOps::is_unit_vector(const Vector3& v, double tolerance) {
    return NumericalOps::nearly_equal(v.norm(), 1.0, tolerance);
}

bool VectorOps::are_parallel(const Vector3& a, const Vector3& b, double tolerance) {
    Vector3 cross = a.cross(b);
    return cross.norm() < tolerance;
}

bool VectorOps::are_perpendicular(const Vector3& a, const Vector3& b, double tolerance) {
    return NumericalOps::nearly_zero(a.dot(b), tolerance);
}

Vector3 VectorOps::safe_normalize(const Vector3& v, double min_norm) {
    double norm = v.norm();
    if (norm < min_norm) {
        return Vector3::Zero();
    }
    return v / norm;
}

Vector3 VectorOps::clamp_magnitude(const Vector3& v, double max_magnitude) {
    double norm = v.norm();
    if (norm <= max_magnitude) {
        return v;
    }
    return v * (max_magnitude / norm);
}

Vector3 VectorOps::clamp_magnitude(const Vector3& v, double min_magnitude, double max_magnitude) {
    double norm = v.norm();
    if (norm < min_magnitude) {
        return norm > math::EPSILON ? v * (min_magnitude / norm) : Vector3::Zero();
    }
    if (norm > max_magnitude) {
        return v * (max_magnitude / norm);
    }
    return v;
}

// ============================================================================
// ROTATION OPERATIONS IMPLEMENTATION
// ============================================================================

Vector3 RotationOps::rodrigues_rotation(const Vector3& v, const Vector3& axis, double angle) {
    Vector3 norm_axis = VectorOps::safe_normalize(axis);
    if (VectorOps::is_zero_vector(norm_axis)) {
        return v;  // No rotation if axis is zero
    }
    
    double cos_angle = std::cos(angle);
    double sin_angle = std::sin(angle);
    
    Vector3 v_parallel = norm_axis * norm_axis.dot(v);
    Vector3 v_perpendicular = v - v_parallel;
    Vector3 w = norm_axis.cross(v_perpendicular);
    
    return v_parallel + v_perpendicular * cos_angle + w * sin_angle;
}

Matrix3 RotationOps::rotation_matrix_from_axis_angle(const Vector3& axis, double angle) {
    Vector3 norm_axis = VectorOps::safe_normalize(axis);
    if (VectorOps::is_zero_vector(norm_axis)) {
        return Matrix3::Identity();
    }
    
    double cos_angle = std::cos(angle);
    double sin_angle = std::sin(angle);
    double one_minus_cos = 1.0 - cos_angle;
    
    double x = norm_axis.x();
    double y = norm_axis.y();
    double z = norm_axis.z();
    
    Matrix3 rotation;
    rotation << cos_angle + x*x*one_minus_cos,    x*y*one_minus_cos - z*sin_angle, x*z*one_minus_cos + y*sin_angle,
                y*x*one_minus_cos + z*sin_angle, cos_angle + y*y*one_minus_cos,    y*z*one_minus_cos - x*sin_angle,
                z*x*one_minus_cos - y*sin_angle, z*y*one_minus_cos + x*sin_angle, cos_angle + z*z*one_minus_cos;
    
    return rotation;
}

Matrix3 RotationOps::rotation_matrix_from_vectors(const Vector3& from, const Vector3& to) {
    Vector3 norm_from = VectorOps::safe_normalize(from);
    Vector3 norm_to = VectorOps::safe_normalize(to);
    
    if (VectorOps::is_zero_vector(norm_from) || VectorOps::is_zero_vector(norm_to)) {
        return Matrix3::Identity();
    }
    
    // Check if vectors are already aligned
    if (VectorOps::are_parallel(norm_from, norm_to)) {
        return norm_from.dot(norm_to) > 0 ? Matrix3::Identity() : -Matrix3::Identity();
    }
    
    Vector3 axis = norm_from.cross(norm_to).normalized();
    double angle = VectorOps::angle_between(norm_from, norm_to);
    
    return rotation_matrix_from_axis_angle(axis, angle);
}

bool RotationOps::is_rotation_matrix(const Matrix3& matrix, double tolerance) {
    // Check if determinant is 1
    if (!NumericalOps::nearly_equal(matrix.determinant(), 1.0, tolerance)) {
        return false;
    }
    
    // Check if matrix is orthogonal (R * R^T = I)
    Matrix3 should_be_identity = matrix * matrix.transpose();
    Matrix3 identity = Matrix3::Identity();
    
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (!NumericalOps::nearly_equal(should_be_identity(i,j), identity(i,j), tolerance)) {
                return false;
            }
        }
    }
    
    return true;
}

// ============================================================================
// GEOMETRIC OPERATIONS IMPLEMENTATION
// ============================================================================

Vector3 GeometryOps::calculate_plane_normal(const Vector3& a, const Vector3& b, const Vector3& c) {
    Vector3 ab = b - a;
    Vector3 ac = c - a;
    
    // Use AC × AB (reversed order from original) for consistent orientation
    Vector3 normal = ac.cross(ab);
    
    return VectorOps::safe_normalize(normal);
}

double GeometryOps::point_to_plane_distance(const Vector3& point, const Vector3& plane_point, const Vector3& plane_normal) {
    Vector3 norm_normal = VectorOps::safe_normalize(plane_normal);
    if (VectorOps::is_zero_vector(norm_normal)) {
        return 0.0;
    }
    
    Vector3 to_point = point - plane_point;
    return to_point.dot(norm_normal);
}

Vector3 GeometryOps::project_point_onto_plane(const Vector3& point, const Vector3& plane_point, const Vector3& plane_normal) {
    double distance = point_to_plane_distance(point, plane_point, plane_normal);
    Vector3 norm_normal = VectorOps::safe_normalize(plane_normal);
    
    return point - distance * norm_normal;
}

Vector3 GeometryOps::mirror_point_across_plane(const Vector3& point, const Vector3& plane_point, const Vector3& plane_normal) {
    double distance = point_to_plane_distance(point, plane_point, plane_normal);
    Vector3 norm_normal = VectorOps::safe_normalize(plane_normal);
    
    return point - 2.0 * distance * norm_normal;
}

double GeometryOps::calculate_z_intersection(double base_x, double base_y, const Vector3& normal) {
    if (NumericalOps::nearly_zero(normal.z())) {
        return 0.0;  // No intersection or infinite intersections
    }
    
    return -(normal.x() * base_x + normal.y() * base_y) / normal.z();
}

Vector3 GeometryOps::project_direction_onto_cone(const Vector3& desired_direction,
                                                 const Vector3& cone_axis_normalized,
                                                 double cone_half_angle_rad) {
    if (VectorOps::is_zero_vector(desired_direction)) {
        return desired_direction;
    }
    
    Vector3 dir_normalized = VectorOps::safe_normalize(desired_direction);
    
    // Calculate angle between desired direction and cone axis
    double dot_product = dir_normalized.dot(cone_axis_normalized);
    double angle_to_axis = NumericalOps::safe_acos(dot_product);
    
    // If direction is within cone, use it directly
    if (angle_to_axis <= cone_half_angle_rad) {
        return desired_direction;
    }
    
    // Project direction onto cone surface
    Vector3 along_axis = cone_axis_normalized * dot_product;
    Vector3 perpendicular = dir_normalized - along_axis;
    
    if (VectorOps::is_zero_vector(perpendicular)) {
        return desired_direction;  // Direction is exactly along cone axis
    }
    
    Vector3 perp_normalized = VectorOps::safe_normalize(perpendicular);
    double cos_half_angle = std::cos(cone_half_angle_rad);
    double sin_half_angle = std::sin(cone_half_angle_rad);
    
    Vector3 projected_normalized = cone_axis_normalized * cos_half_angle + perp_normalized * sin_half_angle;
    
    return projected_normalized * desired_direction.norm();
}

// ============================================================================
// NUMERICAL OPERATIONS IMPLEMENTATION
// ============================================================================

double NumericalOps::safe_acos(double value) {
    return std::acos(clamp(value, -1.0, 1.0));
}

double NumericalOps::safe_asin(double value) {
    return std::asin(clamp(value, -1.0, 1.0));
}

double NumericalOps::safe_sqrt(double value) {
    return std::sqrt(std::max(0.0, value));
}

double NumericalOps::safe_divide(double numerator, double denominator, double default_value) {
    return nearly_zero(denominator) ? default_value : numerator / denominator;
}

double NumericalOps::wrap_angle(double angle) {
    while (angle > math::PI) angle -= math::TWO_PI;
    while (angle < -math::PI) angle += math::TWO_PI;
    return angle;
}

double NumericalOps::wrap_angle_0_2pi(double angle) {
    while (angle >= math::TWO_PI) angle -= math::TWO_PI;
    while (angle < 0.0) angle += math::TWO_PI;
    return angle;
}

double NumericalOps::lerp(double a, double b, double t) {
    return a + t * (b - a);
}

Vector3 NumericalOps::lerp(const Vector3& a, const Vector3& b, double t) {
    return a + t * (b - a);
}

Vector3 NumericalOps::slerp(const Vector3& a, const Vector3& b, double t) {
    Vector3 norm_a = VectorOps::safe_normalize(a);
    Vector3 norm_b = VectorOps::safe_normalize(b);
    
    double angle = VectorOps::angle_between(norm_a, norm_b);
    
    if (nearly_zero(angle)) {
        return lerp(a, b, t);  // Vectors are nearly parallel, use linear interpolation
    }
    
    double sin_angle = std::sin(angle);
    double factor_a = std::sin((1.0 - t) * angle) / sin_angle;
    double factor_b = std::sin(t * angle) / sin_angle;
    
    return factor_a * a + factor_b * b;
}

double NumericalOps::mean(const std::vector<double>& values) {
    if (values.empty()) return 0.0;
    return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
}

double NumericalOps::standard_deviation(const std::vector<double>& values) {
    if (values.size() <= 1) return 0.0;
    
    double avg = mean(values);
    double sum_sq_diff = 0.0;
    
    for (double value : values) {
        double diff = value - avg;
        sum_sq_diff += diff * diff;
    }
    
    return std::sqrt(sum_sq_diff / (values.size() - 1));
}

Vector3 NumericalOps::centroid(const std::vector<Vector3>& points) {
    if (points.empty()) return Vector3::Zero();
    
    Vector3 sum = Vector3::Zero();
    for (const Vector3& point : points) {
        sum += point;
    }
    
    return sum / static_cast<double>(points.size());
}

// ============================================================================
// BASE POSITIONS IMPLEMENTATION
// ============================================================================

Vector3 BasePositions::get_base_position_A() {
    return Vector3(0, robot::RADIUS, 0);
}

Vector3 BasePositions::get_base_position_B() {
    return Vector3(robot::RADIUS * std::cos(geometry::BASE_B_ANGLE),
                   robot::RADIUS * std::sin(geometry::BASE_B_ANGLE), 
                   0);
}

Vector3 BasePositions::get_base_position_C() {
    return Vector3(robot::RADIUS * std::cos(geometry::BASE_C_ANGLE),
                   robot::RADIUS * std::sin(geometry::BASE_C_ANGLE), 
                   0);
}

std::vector<Vector3> BasePositions::get_all_base_positions() {
    return {get_base_position_A(), get_base_position_B(), get_base_position_C()};
}

bool BasePositions::validate_base_triangle(double tolerance) {
    std::vector<Vector3> bases = get_all_base_positions();
    
    // Check that no two bases are at the same location
    for (size_t i = 0; i < bases.size(); ++i) {
        for (size_t j = i + 1; j < bases.size(); ++j) {
            if (VectorOps::distance(bases[i], bases[j]) < tolerance) {
                return false;
            }
        }
    }
    
    // Check that all bases are in the XY plane (z = 0)
    for (const Vector3& base : bases) {
        if (!NumericalOps::nearly_zero(base.z(), tolerance)) {
            return false;
        }
    }
    
    return true;
}

double BasePositions::calculate_base_triangle_area() {
    std::vector<Vector3> bases = get_all_base_positions();
    
    Vector3 ab = bases[1] - bases[0];
    Vector3 ac = bases[2] - bases[0];
    
    return 0.5 * ab.cross(ac).norm();
}

} // namespace delta::core::math