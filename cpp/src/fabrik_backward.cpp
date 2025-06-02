#include "fabrik_backward.hpp"
#include <cmath>
#include <algorithm>

namespace delta {

// Static helper function for cone surface projection
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

FabrikBackwardResult FabrikBackward::iterate_to_target(const FabrikChain& initial_chain,
                                                      const Vector3& target_position,
                                                      double tolerance,
                                                      int max_iterations) {
    
    FabrikChain current_chain = initial_chain;
    std::vector<Vector3> iteration_history;
    
    // Check if target is reachable
    bool reachable = is_target_reachable(initial_chain, target_position);
    
    int iteration = 0;
    Vector3 current_end_effector = get_end_effector_position(current_chain);
    iteration_history.push_back(current_end_effector);
    
    // Iterate until convergence or max iterations
    for (iteration = 0; iteration < max_iterations; iteration++) {
        // Perform single backward iteration
        current_chain = single_backward_iteration(current_chain, target_position);
        
        // Update end-effector position
        current_end_effector = get_end_effector_position(current_chain);
        iteration_history.push_back(current_end_effector);
        
        // Check convergence
        if (has_converged(current_end_effector, target_position, tolerance)) {
            break;
        }
    }
    
    double distance_to_base = calculate_distance_to_base(current_chain);
    
    FabrikBackwardResult result(current_chain, target_position, reachable, distance_to_base, iteration + 1);
    result.iteration_history = iteration_history;
    
    return result;
}

FabrikChain FabrikBackward::single_backward_iteration(const FabrikChain& chain_state_before_pass,
                                                    const Vector3& target_position) {
    
    FabrikChain updated_chain = chain_state_before_pass; // Make a working copy

    int num_joints = static_cast<int>(updated_chain.joints.size());
    if (num_joints == 0) {
        return updated_chain;
    }

    // --- Top-Down Approach ---

    // Step 1: Set J_N-1 (end-effector) to target_position
    updated_chain.joints[num_joints - 1].position = target_position;

    // Step 2: Iterate backwards from J_N-2 down to J_0
    for (int i = num_joints - 2; i >= 0; --i) {
        // J_i+1' (J_next_prime): The joint already positioned in this backward pass
        const Vector3& J_next_prime = updated_chain.joints[i + 1].position;

        // J_i_orig (J_current_original): The original position of joint J_i before this pass
        const Vector3& J_current_original = chain_state_before_pass.joints[i].position;

        // Segment J_i -- J_i+1 and its defined length
        const FabrikSegment& segment_to_enforce = updated_chain.segments[i];
        double segment_length = segment_to_enforce.length;

        if (segment_length < EPSILON_MATH) {
            // If segment length is zero, place J_i at J_i+1'
            updated_chain.joints[i].position = J_next_prime;
            continue;
        }

        Vector3 guidance_point_for_direction = J_current_original;

        // Cone Constraint Logic:
        // The constraint is at joint J_i+1 (which is J_next_prime).
        // The cone's apex is at J_i+1'.
        // The cone opens along the direction J_i+2' -> J_i+1'.
        // We check if J_i_orig is outside this cone.
        bool can_apply_cone_constraint = (i + 2 < num_joints) &&
                                         (updated_chain.joints[i + 1].type == JointType::SPHERICAL_120);

        if (can_apply_cone_constraint) {
            const Vector3& J_apex_cone = J_next_prime;
            const Vector3& J_cone_direction_ref_point = updated_chain.joints[i + 2].position;

            // Cone axis points from J_cone_direction_ref_point towards J_apex_cone
            Vector3 cone_axis_vec = J_apex_cone - J_cone_direction_ref_point;

            if (cone_axis_vec.norm() > EPSILON_MATH) {
                Vector3 cone_axis_normalized = cone_axis_vec.normalized();

                // SPHERICAL_JOINT_CONE_ANGLE_RAD is the full angle (120 degrees)
                double cone_half_angle_rad = SPHERICAL_JOINT_CONE_ANGLE_RAD / 2.0;

                // Vector from cone apex (J_i+1') to the point to check (J_i_orig)
                Vector3 vec_apex_to_J_curr_orig = J_current_original - J_apex_cone;

                if (vec_apex_to_J_curr_orig.norm() > EPSILON_MATH) {
                    Vector3 dir_apex_to_J_curr_orig_normalized = vec_apex_to_J_curr_orig.normalized();

                    // Check if J_current_original is outside the cone
                    double dot_product = dir_apex_to_J_curr_orig_normalized.dot(cone_axis_normalized);

                    if (dot_product < std::cos(cone_half_angle_rad) - EPSILON_MATH) {
                        // J_i_orig is outside cone - project to cone surface
                        guidance_point_for_direction = project_point_to_cone_surface_internal(
                            J_current_original, J_apex_cone, cone_axis_normalized, cone_half_angle_rad
                        );
                    }
                    // Else: J_i_orig is inside or on the cone, use J_i_orig as guidance
                }
                // Else: J_i_orig is at the cone apex, use J_i_orig
            }
            // Else: Cone axis is zero length, use J_i_orig as guidance
        }

        // Place J_i' at segment_length distance from J_i+1' towards guidance point
        Vector3 direction_for_placement_vec = guidance_point_for_direction - J_next_prime;
        Vector3 final_placement_direction;

        if (direction_for_placement_vec.norm() < EPSILON_MATH) {
            // Guidance point is coincident with J_next_prime - need fallback direction
            if (can_apply_cone_constraint) {
                const Vector3& J_apex_cone = J_next_prime;
                const Vector3& J_cone_direction_ref_point = updated_chain.joints[i + 2].position;
                Vector3 cone_axis_vec = J_apex_cone - J_cone_direction_ref_point;
                if (cone_axis_vec.norm() > EPSILON_MATH) {
                    final_placement_direction = cone_axis_vec.normalized();
                } else {
                    // Fallback: point towards previous joint
                    if (i > 0 && (chain_state_before_pass.joints[i-1].position - J_next_prime).norm() > EPSILON_MATH) {
                        final_placement_direction = (chain_state_before_pass.joints[i-1].position - J_next_prime).normalized();
                    } else if (i == 0 && (Vector3(0, 0, 0) - J_next_prime).norm() > EPSILON_MATH) {
                        final_placement_direction = (Vector3(0, 0, 0) - J_next_prime).normalized();
                    } else {
                        final_placement_direction = Vector3(0, 0, -1); // Ultimate fallback
                    }
                }
            } else {
                // No cone constraint - point towards previous joint
                if (i > 0 && (chain_state_before_pass.joints[i-1].position - J_next_prime).norm() > EPSILON_MATH) {
                    final_placement_direction = (chain_state_before_pass.joints[i-1].position - J_next_prime).normalized();
                } else if (i == 0 && (Vector3(0, 0, 0) - J_next_prime).norm() > EPSILON_MATH) {
                    final_placement_direction = (Vector3(0, 0, 0) - J_next_prime).normalized();
                } else {
                    final_placement_direction = Vector3(0, 0, -1); // Ultimate fallback
                }
            }
        } else {
            final_placement_direction = direction_for_placement_vec.normalized();
        }

        updated_chain.joints[i].position = J_next_prime + final_placement_direction * segment_length;
    }

    return updated_chain;
}

bool FabrikBackward::is_target_reachable(const FabrikChain& chain, const Vector3& target_position) {
    // Calculate total reach of the chain
    double total_reach = 0.0;
    for (const auto& segment : chain.segments) {
        total_reach += segment.length;
    }
    
    // Calculate distance from base to target
    Vector3 base_position = chain.joints[0].position;
    double distance_to_target = (target_position - base_position).norm();
    
    return distance_to_target <= total_reach;
}

double FabrikBackward::calculate_distance_to_base(const FabrikChain& chain) {
    if (chain.joints.empty()) return 0.0;
    
    Vector3 base_position = chain.joints[0].position;
    Vector3 end_position = chain.joints.back().position;
    
    return (end_position - base_position).norm();
}

Vector3 FabrikBackward::get_end_effector_position(const FabrikChain& chain) {
    if (chain.joints.empty()) return Vector3(0, 0, 0);
    return chain.joints.back().position;
}

bool FabrikBackward::has_converged(const Vector3& current_end_effector,
                                  const Vector3& target_position,
                                  double tolerance) {
    
    double distance = (current_end_effector - target_position).norm();
    return distance <= tolerance;
}

} // namespace delta