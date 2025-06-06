#include "constraint_utils.hpp"
#include <cmath>
#include <algorithm>

namespace delta {

Vector3 project_direction_onto_cone(const Vector3& desired_direction,
                                   const Vector3& cone_axis_normalized,
                                   double cone_half_angle_rad) {
    
    if (desired_direction.norm() < EPSILON_MATH) {
        return desired_direction;
    }
    
    Vector3 dir_normalized = desired_direction.normalized();
    
    // Calculate angle between desired direction and cone axis
    double dot_product = dir_normalized.dot(cone_axis_normalized);
    double angle_to_axis = std::acos(std::max(-1.0, std::min(1.0, dot_product)));
    
    // If direction is within cone, use it directly
    if (angle_to_axis <= cone_half_angle_rad) {
        return desired_direction;
    }
    
    // Project direction onto cone surface
    // Find component along cone axis
    Vector3 along_axis = cone_axis_normalized * dot_product;
    
    // Find perpendicular component
    Vector3 perpendicular = dir_normalized - along_axis;
    
    if (perpendicular.norm() < EPSILON_MATH) {
        // Direction is exactly along cone axis - use it
        return desired_direction;
    }
    
    // Create new direction on cone surface
    Vector3 perp_normalized = perpendicular.normalized();
    double cos_half_angle = std::cos(cone_half_angle_rad);
    double sin_half_angle = std::sin(cone_half_angle_rad);
    
    // New direction on cone surface closest to desired direction
    Vector3 projected_normalized = cone_axis_normalized * cos_half_angle + perp_normalized * sin_half_angle;
    
    // Scale back to original magnitude
    return projected_normalized * desired_direction.norm();
}

} // namespace delta