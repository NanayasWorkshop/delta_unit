#include "fabrik_backward.hpp"
#include <cmath>
#include <algorithm>

namespace delta {

// Helper function for projecting a direction onto cone constraints (NOT point projection!)
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

        // Calculate desired direction: from J_next_prime toward J_current_original
        Vector3 desired_direction = J_current_original - J_next_prime;
        
        // Apply cone constraint if applicable
        bool can_apply_cone_constraint = (i + 2 < num_joints) &&
                                         (updated_chain.joints[i + 1].type == JointType::SPHERICAL_120);

        Vector3 final_direction = desired_direction;

        if (can_apply_cone_constraint && desired_direction.norm() > EPSILON_MATH) {
            const Vector3& J_apex_cone = J_next_prime;
            const Vector3& J_cone_direction_ref_point = updated_chain.joints[i + 2].position;

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

        // Place J_i' at segment_length distance from J_i+1' in the final direction
        if (final_direction.norm() > EPSILON_MATH) {
            Vector3 final_placement_direction = final_direction.normalized();
            updated_chain.joints[i].position = J_next_prime + final_placement_direction * segment_length;
        } else {
            // Fallback when direction is zero or invalid
            Vector3 fallback_direction;
            
            if (i > 0) {
                // Point toward previous joint's original position
                fallback_direction = chain_state_before_pass.joints[i-1].position - J_next_prime;
                if (fallback_direction.norm() < EPSILON_MATH) {
                    fallback_direction = Vector3(0, 0, -1); // Ultimate fallback
                }
            } else {
                // Base joint - point toward origin or down
                fallback_direction = Vector3(0, 0, 0) - J_next_prime;
                if (fallback_direction.norm() < EPSILON_MATH) {
                    fallback_direction = Vector3(0, 0, -1); // Ultimate fallback
                }
            }
            
            updated_chain.joints[i].position = J_next_prime + fallback_direction.normalized() * segment_length;
        }
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