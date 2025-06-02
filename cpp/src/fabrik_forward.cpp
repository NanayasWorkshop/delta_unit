#include "fabrik_forward.hpp"
#include "kinematics_module.hpp"
#include <cmath>
#include <algorithm>

namespace delta {

// Static helper function for cone surface projection (same as backward)
static Vector3 project_point_to_cone_surface_internal(
    const Vector3& point_P,
    const Vector3& apex_A,
    const Vector3& cone_axis_v_normalized, // Normalized axis, points from apex towards cone opening
    double half_angle_alpha_rad) {

    Vector3 AP = point_P - apex_A;
    double dist_AP_along_axis = AP.dot(cone_axis_v_normalized);

    // If point is at or "behind" the apex (relative to cone direction),
    // the nearest point on the cone surface is the apex itself.
    if (dist_AP_along_axis <= EPSILON_MATH) {
        return apex_A;
    }

    // Point on the cone axis that is "level" with P's projection onto the axis
    Vector3 P_on_axis = apex_A + cone_axis_v_normalized * dist_AP_along_axis;

    Vector3 vec_axis_to_P = point_P - P_on_axis;
    double dist_P_to_axis = vec_axis_to_P.norm();

    // Radius of the cone at the height of P_on_axis
    double cone_radius_at_height = dist_AP_along_axis * std::tan(half_angle_alpha_rad);

    // If P is on the axis (and past the apex, handled by dist_AP_along_axis > EPSILON_MATH),
    // it's considered on the cone degenerately.
    if (dist_P_to_axis < EPSILON_MATH) {
        return P_on_axis; // Already on the center line of the cone
    }

    // If P is outside (radially), project it onto the surface by scaling its radial component.
    // The calling function should have already determined if P is angularly outside.
    // This function just finds the point on the surface along the radial direction from P to the axis.
    Vector3 dir_axis_to_P_component_normalized = vec_axis_to_P.normalized();
    Vector3 projected_point = P_on_axis + dir_axis_to_P_component_normalized * cone_radius_at_height;
    return projected_point;
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
        // Note: Segments are indexed such that segment[i-1] connects joint[i-1] and joint[i]
        double segment_length = target_segment_lengths[i - 1];

        if (segment_length < EPSILON_MATH) {
            // If segment length is zero, place J_i at J_i-1'
            updated_chain.joints[i].position = J_prev_prime;
            continue;
        }

        Vector3 guidance_point_for_direction = J_current_original;

        // SPECIAL CASE: First joint after base (J_1) must always go straight up in Z+
        if (i == 1) {
            // Force the first joint to be directly above the base in Z+ direction
            Vector3 final_placement_direction = Vector3(0, 0, 1); // Straight up in Z+
            updated_chain.joints[i].position = J_prev_prime + final_placement_direction * segment_length;
            continue; // Skip cone constraint logic for first joint
        }

        // Cone Constraint Logic:
        // The constraint is at joint J_i-1 (which is J_prev_prime).
        // The cone's apex is at J_i-1'.
        // The cone opens along the direction J_i-2' -> J_i-1'.
        // We check if J_i_orig is outside this cone.
        // This check is only relevant if:
        //   a) Joint J_i-1 has a cone constraint (e.g., SPHERICAL_120).
        //   b) Joint J_i-2 exists (i.e., i - 2 >= 0).
        bool can_apply_cone_constraint = (i - 2 >= 0) &&
                                         (updated_chain.joints[i - 1].type == JointType::SPHERICAL_120);

        if (can_apply_cone_constraint) {
            const Vector3& J_apex_cone = J_prev_prime;                                    // This is J_i-1'
            const Vector3& J_cone_direction_ref_point = updated_chain.joints[i - 2].position; // This is J_i-2'

            // Cone axis points from J_cone_direction_ref_point towards J_apex_cone
            // (e.g., from J_i-2' towards J_i-1')
            Vector3 cone_axis_vec = J_apex_cone - J_cone_direction_ref_point;

            if (cone_axis_vec.norm() > EPSILON_MATH) { // Ensure cone axis is well-defined
                Vector3 cone_axis_normalized = cone_axis_vec.normalized();

                // SPHERICAL_JOINT_CONE_ANGLE_RAD should be the *full* angle (e.g., 120 degrees)
                double cone_half_angle_rad = SPHERICAL_JOINT_CONE_ANGLE_RAD / 2.0;

                // Vector from cone apex (J_i-1') to the point to check (J_i_orig)
                Vector3 vec_apex_to_J_curr_orig = J_current_original - J_apex_cone;

                if (vec_apex_to_J_curr_orig.norm() > EPSILON_MATH) {
                    Vector3 dir_apex_to_J_curr_orig_normalized = vec_apex_to_J_curr_orig.normalized();

                    // Check if J_current_original is outside the cone:
                    // If angle(vec_apex_to_J_curr_orig, cone_axis_normalized) > cone_half_angle_rad
                    // then dot(dir_apex_to_J_curr_orig_normalized, cone_axis_normalized) < cos(cone_half_angle_rad)
                    double dot_product = dir_apex_to_J_curr_orig_normalized.dot(cone_axis_normalized);

                    if (dot_product < std::cos(cone_half_angle_rad) - EPSILON_MATH) { // J_i_orig is outside cone
                        // Project J_i_orig to the nearest point on the cone surface
                        guidance_point_for_direction = project_point_to_cone_surface_internal(
                            J_current_original, J_apex_cone, cone_axis_normalized, cone_half_angle_rad
                        );
                    }
                    // Else: J_i_orig is inside or on the cone, use J_i_orig as guidance.
                }
                // Else: J_i_orig is at the cone apex (J_i-1'), effectively on the cone. Use J_i_orig.
            }
            // Else: Cone axis is zero length (J_i-1' and J_i-2' are coincident).
            // Cone is ill-defined. Use J_i_orig as guidance point.
        }

        // Place J_i' (updated_chain.joints[i].position):
        // On the line from J_i-1' (J_prev_prime) towards guidance_point_for_direction,
        // at 'segment_length' distance from J_i-1'.

        Vector3 direction_for_placement_vec = guidance_point_for_direction - J_prev_prime;
        Vector3 final_placement_direction;

        if (direction_for_placement_vec.norm() < EPSILON_MATH) {
            // guidance_point_for_direction is coincident with J_prev_prime.
            // This can happen if J_i_orig (or its projection) landed exactly on J_i-1'.
            // The "dotted line" is a point. We need a fallback direction.
            // If a cone was active, use the cone's axis.
            if (can_apply_cone_constraint) { // Check again if cone logic was attempted
                const Vector3& J_apex_cone = J_prev_prime;
                const Vector3& J_cone_direction_ref_point = updated_chain.joints[i - 2].position;
                Vector3 cone_axis_vec = J_apex_cone - J_cone_direction_ref_point;
                if (cone_axis_vec.norm() > EPSILON_MATH) {
                    final_placement_direction = cone_axis_vec.normalized();
                } else {
                    // Fallback if cone axis also degenerate: point from J_i-1' towards where J_i+1_orig was
                     if (i + 1 < num_joints && (chain_state_before_pass.joints[i+1].position - J_prev_prime).norm() > EPSILON_MATH) {
                        final_placement_direction = (chain_state_before_pass.joints[i+1].position - J_prev_prime).normalized();
                    } else {
                        final_placement_direction = Vector3(0, 0, 1); // Ultimate fallback (e.g., along positive Z)
                    }
                }
            } else { // No cone constraint was applicable, and J_i_orig is at J_i-1'.
                     // Point from J_i-1' towards where J_i+1_orig was
                if (i + 1 < num_joints && (chain_state_before_pass.joints[i+1].position - J_prev_prime).norm() > EPSILON_MATH) {
                    final_placement_direction = (chain_state_before_pass.joints[i+1].position - J_prev_prime).normalized();
                } else {
                    final_placement_direction = Vector3(0, 0, 1); // Ultimate fallback
                }
            }
        } else {
            final_placement_direction = direction_for_placement_vec.normalized();
        }

        updated_chain.joints[i].position = J_prev_prime + final_placement_direction * segment_length;
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
        
        // Calculate prismatic length using existing modules
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
    // Use KinematicsModule instead of FermatModule directly
    // KinematicsModule properly handles half-angle transformation
    KinematicsResult kinematics_result = KinematicsModule::calculate(transformed_direction);
    
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