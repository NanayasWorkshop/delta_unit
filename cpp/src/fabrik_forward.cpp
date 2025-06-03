#include "fabrik_forward.hpp"
#include "kinematics_module.hpp"
#include <cmath>
#include <algorithm>

namespace delta {

// Helper function for projecting a direction onto cone constraints (same as backward)
static Vector3 project_direction_onto_cone(const Vector3& desired_direction,
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

FabrikForwardResult FabrikForward::iterate_from_base(const FabrikChain& backward_result_chain,
                                                    double tolerance,
                                                    int max_iterations) {
    
    // Step 1: Calculate new segment lengths from backward result
    std::vector<double> new_segment_lengths = calculate_new_segment_lengths(backward_result_chain);
    
    FabrikChain current_chain = backward_result_chain;
    std::vector<Vector3> iteration_history;
    
    int iteration = 0;
    Vector3 current_base = get_base_position(current_chain);
    iteration_history.push_back(current_base);
    
    // Iterate until base is at origin or max iterations
    for (iteration = 0; iteration < max_iterations; iteration++) {
        // Perform single forward iteration
        current_chain = single_forward_iteration(current_chain, new_segment_lengths);
        
        // Update base position
        current_base = get_base_position(current_chain);
        iteration_history.push_back(current_base);
        
        // Check convergence (base should be at origin)
        if (has_converged_forward(current_base, Vector3(0, 0, 0), tolerance)) {
            break;
        }
    }
    
    Vector3 final_end_effector = current_chain.joints.back().position;
    bool constraints_ok = is_base_at_origin(current_chain, tolerance);
    
    FabrikForwardResult result(current_chain, current_base, final_end_effector, constraints_ok, iteration + 1);
    result.iteration_history = iteration_history;
    result.recalculated_lengths = new_segment_lengths;
    
    return result;
}

FabrikChain FabrikForward::single_forward_iteration(const FabrikChain& chain_state_before_pass,
                                                  const std::vector<double>& target_segment_lengths) {
    
    FabrikChain updated_chain = chain_state_before_pass; // Make a working copy

    int num_joints = static_cast<int>(updated_chain.joints.size());
    if (num_joints == 0) {
        return updated_chain;
    }

    // --- Base-to-End Approach ---

    // Step 1: Fix base (J_0) at origin
    updated_chain.joints[0].position = Vector3(0, 0, 0);

    // Step 2: Iterate forwards from J_1 up to J_N-1
    for (int i = 1; i < num_joints; ++i) {
        // J_i-1' (J_prev_prime): The joint already positioned in this forward pass
        const Vector3& J_prev_prime = updated_chain.joints[i - 1].position;

        // J_i_orig (J_current_original): The original position of joint J_i before this pass
        const Vector3& J_current_original = chain_state_before_pass.joints[i].position;

        // Segment J_i-1 -- J_i and its defined length
        double segment_length = target_segment_lengths[i - 1];

        if (segment_length < EPSILON_MATH) {
            // If segment length is zero, place J_i at J_i-1'
            updated_chain.joints[i].position = J_prev_prime;
            continue;
        }

        // SPECIAL CASE: First joint after base (J_1) must always go straight up in Z+
        if (i == 1) {
            // Force the first joint to be directly above the base in Z+ direction
            Vector3 final_placement_direction = Vector3(0, 0, 1); // Straight up in Z+
            updated_chain.joints[i].position = J_prev_prime + final_placement_direction * segment_length;
            continue; // Skip cone constraint logic for first joint
        }

        // Calculate desired direction: from J_prev_prime toward J_current_original
        Vector3 desired_direction = J_current_original - J_prev_prime;
        
        // Apply cone constraint if applicable
        bool can_apply_cone_constraint = (i - 2 >= 0) &&
                                         (updated_chain.joints[i - 1].type == JointType::SPHERICAL_120);

        Vector3 final_direction = desired_direction;

        if (can_apply_cone_constraint && desired_direction.norm() > EPSILON_MATH) {
            const Vector3& J_apex_cone = J_prev_prime;                                    // This is J_i-1'
            const Vector3& J_cone_direction_ref_point = updated_chain.joints[i - 2].position; // This is J_i-2'

            // Cone axis points from J_cone_direction_ref_point towards J_apex_cone
            Vector3 cone_axis_vec = J_apex_cone - J_cone_direction_ref_point;

            if (cone_axis_vec.norm() > EPSILON_MATH) {
                Vector3 cone_axis_normalized = cone_axis_vec.normalized();
                double cone_half_angle_rad = SPHERICAL_JOINT_CONE_ANGLE_RAD / 2.0;

                // ✅ FIX: Project the DIRECTION onto cone constraints (not the point!)
                final_direction = project_direction_onto_cone(
                    desired_direction, 
                    cone_axis_normalized, 
                    cone_half_angle_rad
                );
            }
        }

        // Place J_i' at segment_length distance from J_i-1' in the final direction
        if (final_direction.norm() > EPSILON_MATH) {
            Vector3 final_placement_direction = final_direction.normalized();
            updated_chain.joints[i].position = J_prev_prime + final_placement_direction * segment_length;
        } else {
            // Fallback when direction is zero or invalid
            Vector3 fallback_direction;
            
            if (i + 1 < num_joints) {
                // Point toward next joint's original position
                fallback_direction = chain_state_before_pass.joints[i+1].position - J_prev_prime;
                if (fallback_direction.norm() < EPSILON_MATH) {
                    fallback_direction = Vector3(0, 0, 1); // Ultimate fallback
                }
            } else {
                // Last joint - point upward
                fallback_direction = Vector3(0, 0, 1);
            }
            
            updated_chain.joints[i].position = J_prev_prime + fallback_direction.normalized() * segment_length;
        }
    }

    return updated_chain;
}

std::vector<double> FabrikForward::calculate_new_segment_lengths(const FabrikChain& backward_result) {
    
    // Step 1: Extract direction pairs
    std::vector<SegmentDirectionPair> direction_pairs = extract_direction_pairs(backward_result);
    
    // Step 2: Calculate segment properties
    std::vector<SegmentProperties> segment_properties = calculate_segment_properties(direction_pairs);
    
    // Step 3: Convert to FABRIK segment lengths
    std::vector<double> fabrik_lengths = convert_to_fabrik_lengths(segment_properties, backward_result.num_robot_segments);
    
    return fabrik_lengths;
}

Vector3 FabrikForward::get_base_position(const FabrikChain& chain) {
    if (chain.joints.empty()) return Vector3(0, 0, 0);
    return chain.joints[0].position;
}

bool FabrikForward::is_base_at_origin(const FabrikChain& chain, double tolerance) {
    Vector3 base_pos = get_base_position(chain);
    return base_pos.norm() <= tolerance;
}

// UPDATED! Now public methods for use by FabrikSolver

std::vector<SegmentDirectionPair> FabrikForward::extract_direction_pairs(const FabrikChain& chain) {
    std::vector<SegmentDirectionPair> pairs;
    
    int num_joints = static_cast<int>(chain.joints.size());
    int num_robot_segments = chain.num_robot_segments;
    
    // Extract direction pairs for each robot segment
    for (int seg = 0; seg < num_robot_segments; seg++) {
        // Direction pair indices
        int ref_start = seg;
        int ref_end = seg + 1;
        int target_start = seg + 1;
        int target_end = seg + 2;
        
        // Ensure indices are valid
        if (target_end >= num_joints) {
            // For last segment, use same direction as reference
            target_start = ref_start;
            target_end = ref_end;
        }
        
        Vector3 ref_dir = (chain.joints[ref_end].position - chain.joints[ref_start].position).normalized();
        Vector3 target_dir = (chain.joints[target_end].position - chain.joints[target_start].position).normalized();
        
        pairs.push_back(SegmentDirectionPair(ref_dir, target_dir, seg));
    }
    
    return pairs;
}

std::vector<SegmentProperties> FabrikForward::calculate_segment_properties(
    const std::vector<SegmentDirectionPair>& direction_pairs) {
    
    std::vector<SegmentProperties> properties;
    
    for (const auto& pair : direction_pairs) {
        // Transform target direction to Z+ reference coordinate system
        Vector3 transformed_dir = transform_to_z_reference(pair.reference_direction, pair.target_direction);
        
        // Calculate prismatic length using existing modules with FABRIK-specific half-angle
        double prismatic_length = calculate_prismatic_from_direction(transformed_dir);
        
        // Calculate H→G distance
        double h_to_g_distance = fabrik_forward_utils::calculate_h_to_g_distance(prismatic_length);
        
        // For now, set fabrik_segment_length to 0 (will be calculated later)
        properties.push_back(SegmentProperties(prismatic_length, h_to_g_distance, 0.0, transformed_dir));
    }
    
    return properties;
}

// Private methods

Vector3 FabrikForward::transform_to_z_reference(const Vector3& reference_direction, 
                                               const Vector3& target_direction) {
    
    Vector3 ref_norm = reference_direction.normalized();
    Vector3 target_norm = target_direction.normalized();
    Vector3 z_axis(0, 0, 1);
    
    // If reference is already Z+, no transformation needed
    if ((ref_norm - z_axis).norm() < 1e-6) {
        return target_norm;
    }
    
    // If reference is -Z, simple flip
    if ((ref_norm + z_axis).norm() < 1e-6) {
        return Vector3(-target_norm.x, -target_norm.y, -target_norm.z);
    }
    
    // Calculate rotation axis (cross product of reference and Z+)
    Vector3 rotation_axis = cross_product(ref_norm, z_axis);
    double rotation_axis_length = rotation_axis.norm();
    
    if (rotation_axis_length < 1e-6) {
        // Vectors are parallel, handled above
        return target_norm;
    }
    
    rotation_axis = rotation_axis / rotation_axis_length;
    
    // Calculate rotation angle
    double cos_angle = ref_norm.dot(z_axis);
    double angle = std::acos(std::max(-1.0, std::min(1.0, cos_angle)));
    
    // Apply rotation to target direction using Rodrigues' rotation formula
    Vector3 rotated = rodrigues_rotation(target_norm, rotation_axis, angle);
    
    return rotated;
}

double FabrikForward::calculate_prismatic_from_direction(const Vector3& transformed_direction) {
    // FABRIK-SPECIFIC: Apply half-angle transformation before calling kinematics
    // This is where the half-angle logic belongs - in FABRIK, not in general kinematics
    
    // Calculate angle from Z axis
    Vector3 z_axis(0, 0, 1);
    Vector3 normalized_input = transformed_direction.normalized();
    double dot_product = normalized_input.dot(z_axis);
    double angle_from_z = std::acos(std::max(-1.0, std::min(1.0, dot_product)));
    
    // Create half-angle vector (FABRIK algorithm requirement) - inline implementation
    Vector3 half_angle_vector;
    
    if (angle_from_z < 1e-10) {
        // Input is already along Z axis, return Z axis
        half_angle_vector = z_axis;
    } else {
        // Calculate half angle
        double half_angle = angle_from_z / 2.0;
        
        // Find axis of rotation (cross product of z_axis and input)
        Vector3 rotation_axis = cross_product(z_axis, normalized_input);
        
        // If vectors are opposite, choose arbitrary perpendicular axis
        if (rotation_axis.norm() < 1e-10) {
            rotation_axis = Vector3(1, 0, 0);
        } else {
            rotation_axis = rotation_axis.normalized();
        }
        
        // Rotate Z axis by half_angle around rotation_axis using Rodrigues' rotation
        double cos_half = std::cos(half_angle);
        double sin_half = std::sin(half_angle);
        
        Vector3 k_cross_z = cross_product(rotation_axis, z_axis);
        double k_dot_z = rotation_axis.dot(z_axis);
        
        half_angle_vector = z_axis * cos_half + 
                           k_cross_z * sin_half + 
                           rotation_axis * (k_dot_z * (1.0 - cos_half));
        
        half_angle_vector = half_angle_vector.normalized();
    }
    
    // Use KinematicsModule with half-angle vector (kinematics will NOT apply additional half-angle)
    KinematicsResult kinematics_result = KinematicsModule::calculate(half_angle_vector);
    
    // Extract prismatic length from kinematics result
    return kinematics_result.prismatic_joint_length;
}

std::vector<double> FabrikForward::convert_to_fabrik_lengths(
    const std::vector<SegmentProperties>& segment_properties,
    int num_robot_segments) {
    
    // Extract H→G distances
    std::vector<double> h_to_g_distances;
    for (const auto& prop : segment_properties) {
        h_to_g_distances.push_back(prop.h_to_g_distance);
    }
    
    // Convert to FABRIK segment lengths
    return fabrik_forward_utils::physical_to_fabrik_lengths(h_to_g_distances);
}

bool FabrikForward::has_converged_forward(const Vector3& current_base,
                                        const Vector3& target_base,
                                        double tolerance) {
    
    double distance = (current_base - target_base).norm();
    return distance <= tolerance;
}

Vector3 FabrikForward::cross_product(const Vector3& a, const Vector3& b) {
    return Vector3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

Vector3 FabrikForward::rodrigues_rotation(const Vector3& v, const Vector3& axis, double angle) {
    double cos_angle = std::cos(angle);
    double sin_angle = std::sin(angle);
    
    Vector3 v_parallel = axis * axis.dot(v);
    Vector3 v_perpendicular = v - v_parallel;
    Vector3 w = cross_product(axis, v_perpendicular);
    
    return v_parallel + v_perpendicular * cos_angle + w * sin_angle;
}

} // namespace delta