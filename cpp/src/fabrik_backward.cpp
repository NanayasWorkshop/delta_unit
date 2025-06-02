#include "fabrik_backward.hpp"
#include <cmath>
#include <algorithm>

namespace delta {

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

FabrikChain FabrikBackward::single_backward_iteration(const FabrikChain& chain,
                                                    const Vector3& target_position) {
    
    FabrikChain updated_chain = chain;
    
    // Start from end-effector and work backwards
    int num_joints = static_cast<int>(updated_chain.joints.size());
    
    // Set end-effector to target position
    updated_chain.joints[num_joints - 1].position = target_position;
    
    // Work backwards through the chain
    for (int i = num_joints - 2; i >= 0; i--) {
        // Get the segment connecting joint i to joint i+1
        const FabrikSegment& segment = updated_chain.segments[i];
        double segment_length = segment.length;
        
        Vector3 next_joint_pos = updated_chain.joints[i + 1].position;
        Vector3 current_joint_pos = updated_chain.joints[i].position;
        
        // Move current joint to maintain segment length from next joint
        // In backward iteration, ALL joints move, including the base
        Vector3 unconstrained_pos = move_joint_toward_target(current_joint_pos, next_joint_pos, segment_length);
        
        // Apply joint constraints (but NOT for base - base can move freely in backward iteration)
        if (updated_chain.joints[i].type == JointType::SPHERICAL_120) {
            Vector3 prev_joint_pos = (i > 0) ? updated_chain.joints[i - 1].position : Vector3(0, 0, 0);
            
            BackwardStepResult step_result = apply_joint_constraint(
                unconstrained_pos, prev_joint_pos, next_joint_pos, 
                updated_chain.joints[i], segment_length
            );
            
            updated_chain.joints[i].position = step_result.new_joint_position;
        } else {
            // For base joint and end-effector, just move to maintain segment length
            // No constraint applied in backward iteration
            updated_chain.joints[i].position = unconstrained_pos;
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

// Private methods

BackwardStepResult FabrikBackward::apply_joint_constraint(const Vector3& unconstrained_position,
                                                        const Vector3& previous_joint_position,
                                                        const Vector3& next_joint_position,
                                                        const FabrikJoint& joint,
                                                        double segment_length) {
    
    if (joint.type != JointType::SPHERICAL_120) {
        return BackwardStepResult(unconstrained_position, false);
    }
    
    // For spherical constraint in backward iteration:
    // - Cone apex is at the current joint position (unconstrained_position)
    // - Cone axis is the direction from next_joint to current_joint (incoming direction)
    Vector3 cone_apex = unconstrained_position;
    Vector3 incoming_direction = (unconstrained_position - next_joint_position).normalized();
    Vector3 cone_axis = incoming_direction;
    
    double cone_angle = joint.constraint_angle; // 120 degrees
    
    Vector3 constrained_pos = apply_spherical_constraint(
        previous_joint_position, cone_apex, cone_axis, cone_angle, segment_length
    );
    
    // The constrained position should be segment_length away from cone_apex
    Vector3 final_pos = cone_apex + (constrained_pos - cone_apex).normalized() * segment_length;
    
    // Check if constraint was applied
    double constraint_violation = (unconstrained_position - final_pos).norm();
    bool was_constrained = constraint_violation > 1e-6;
    
    return BackwardStepResult(final_pos, was_constrained, constraint_violation);
}

Vector3 FabrikBackward::apply_spherical_constraint(const Vector3& target_position,
                                                  const Vector3& cone_apex,
                                                  const Vector3& cone_axis,
                                                  double cone_angle,
                                                  double distance_from_apex) {
    
    Vector3 to_target = (target_position - cone_apex).normalized();
    
    // Calculate angle between cone axis and target direction
    double dot_product = std::max(-1.0, std::min(1.0, cone_axis.dot(to_target)));
    double angle_to_axis = std::acos(dot_product);
    
    // If within cone, no constraint needed
    double half_cone_angle = cone_angle / 2.0;
    if (angle_to_axis <= half_cone_angle) {
        return cone_apex + to_target * distance_from_apex;
    }
    
    // Project onto cone surface
    // Find the vector on the cone surface closest to the target direction
    
    // Calculate the axis perpendicular to both cone_axis and to_target
    Vector3 perpendicular(
        cone_axis.y * to_target.z - cone_axis.z * to_target.y,
        cone_axis.z * to_target.x - cone_axis.x * to_target.z,
        cone_axis.x * to_target.y - cone_axis.y * to_target.x
    );
    
    if (perpendicular.norm() < 1e-10) {
        // Vectors are parallel, return along cone axis
        return cone_apex + cone_axis * distance_from_apex;
    }
    
    perpendicular = perpendicular.normalized();
    
    // Calculate direction on cone surface
    Vector3 cone_direction = cone_axis * std::cos(half_cone_angle) + 
                           perpendicular * std::sin(half_cone_angle);
    
    return cone_apex + cone_direction * distance_from_apex;
}

Vector3 FabrikBackward::move_joint_toward_target(const Vector3& from_joint,
                                                const Vector3& to_joint,
                                                double required_distance) {
    
    // Calculate direction from 'from_joint' to 'to_joint' 
    Vector3 direction = (from_joint - to_joint);
    double current_distance = direction.norm();
    
    if (current_distance < 1e-10) {
        // Points are the same, return arbitrary direction
        return to_joint + Vector3(required_distance, 0, 0);
    }
    
    // Normalize and scale to required distance
    direction = direction.normalized();
    return to_joint + direction * required_distance;
}

bool FabrikBackward::has_converged(const Vector3& current_end_effector,
                                  const Vector3& target_position,
                                  double tolerance) {
    
    double distance = (current_end_effector - target_position).norm();
    return distance <= tolerance;
}

} // namespace delta