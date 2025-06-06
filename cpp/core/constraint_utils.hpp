#ifndef DELTA_CONSTRAINT_UTILS_HPP
#define DELTA_CONSTRAINT_UTILS_HPP

#include "math_utils.hpp"
#include "constants.hpp"

namespace delta {

// Shared cone constraint projection function
Vector3 project_direction_onto_cone(const Vector3& desired_direction,
                                   const Vector3& cone_axis_normalized,
                                   double cone_half_angle_rad);

} // namespace delta

#endif // DELTA_CONSTRAINT_UTILS_HPP
