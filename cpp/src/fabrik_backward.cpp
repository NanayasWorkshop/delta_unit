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
                unconstrained_pos, next_joint_pos, prev_joint_pos, 
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
    
    // In BACKWARD iteration for joint constraint:
    // - We're trying to place the current joint at 'unconstrained_position'
    // - previous_joint_position = the joint we just processed (closer to target)
    // - next_joint_position = the joint we'll process next (closer to base)
    //
    // The constraint is: angle between incoming and outgoing segments ≤ 120°
    // - Incoming segment: from next_joint TO current joint (unconstrained_position)
    // - Outgoing segment: from current joint TO previous_joint
    
    // Calculate segment directions
    Vector3 incoming_direction = (unconstrained_position - next_joint_position).normalized();
    Vector3 outgoing_direction = (previous_joint_position - unconstrained_position).normalized();
    
    // Check angle between incoming and outgoing directions
    double dot_product = incoming_direction.dot(outgoing_direction);
    double angle = std::acos(std::max(-1.0, std::min(1.0, dot_product)));
    
    // 120° constraint means angle ≤ 120° (or ≤ 2π/3 radians)
    if (angle <= SPHERICAL_JOINT_CONE_ANGLE_RAD) {
        // Constraint satisfied, no adjustment needed
        return BackwardStepResult(unconstrained_position, false, 0.0);
    }
    
    // Constraint violated, need to adjust position
    // Project the outgoing direction onto the cone surface
    
    // The cone axis is the incoming direction
    // We need to rotate the outgoing direction to be within 120° of incoming
    
    // Calculate rotation axis (perpendicular to both directions)
    Vector3 rotation_axis = Vector3(
        incoming_direction.y * outgoing_direction.z - incoming_direction.z * outgoing_direction.y,
        incoming_direction.z * outgoing_direction.x - incoming_direction.x * outgoing_direction.z,
        incoming_direction.x * outgoing_direction.y - incoming_direction.y * outgoing_direction.x
    );
    
    double rotation_axis_length = rotation_axis.norm();
    if (rotation_axis_length < 1e-10) {
        // Directions are parallel/anti-parallel
        // For anti-parallel case, choose perpendicular direction
        Vector3 perpendicular;
        if (std::abs(incoming_direction.z) < 0.9) {
            perpendicular = Vector3(0, 0, 1);
        } else {
            perpendicular = Vector3(1, 0, 0);
        }
        
        // Create outgoing direction at exactly 120° from incoming
        Vector3 temp = Vector3(
            perpendicular.y * incoming_direction.z - perpendicular.z * incoming_direction.y,
            perpendicular.z * incoming_direction.x - perpendicular.x * incoming_direction.z,
            perpendicular.x * incoming_direction.y - perpendicular.y * incoming_direction.x
        ).normalized();
        
        Vector3 constrained_outgoing = incoming_direction * std::cos(SPHERICAL_JOINT_CONE_ANGLE_RAD) + 
                                       temp * std::sin(SPHERICAL_JOINT_CONE_ANGLE_RAD);
        
        Vector3 constrained_position = previous_joint_position - constrained_outgoing * segment_length;
        return BackwardStepResult(constrained_position, true, angle - SPHERICAL_JOINT_CONE_ANGLE_RAD);
    }
    
    rotation_axis = rotation_axis.normalized();
    
    // Calculate how much to rotate to reach exactly 120°
    double target_angle = SPHERICAL_JOINT_CONE_ANGLE_RAD;
    double rotation_angle = angle - target_angle;
    
    // Rotate outgoing direction by rotation_angle around rotation_axis
    double cos_rot = std::cos(rotation_angle);
    double sin_rot = std::sin(rotation_angle);
    
    // Rodrigues' rotation formula
    Vector3 v_parallel = rotation_axis * rotation_axis.dot(outgoing_direction);
    Vector3 v_perpendicular = outgoing_direction - v_parallel;
    Vector3 w = Vector3(
        rotation_axis.y * v_perpendicular.z - rotation_axis.z * v_perpendicular.y,
        rotation_axis.z * v_perpendicular.x - rotation_axis.x * v_perpendicular.z,
        rotation_axis.x * v_perpendicular.y - rotation_axis.y * v_perpendicular.x
    );
    
    Vector3 constrained_outgoing = v_parallel + v_perpendicular * cos_rot + w * sin_rot;
    constrained_outgoing = constrained_outgoing.normalized();
    
    // Calculate constrained position
    Vector3 constrained_position = previous_joint_position - constrained_outgoing * segment_length;
    
    double constraint_violation = angle - SPHERICAL_JOINT_CONE_ANGLE_RAD;
    
    return BackwardStepResult(constrained_position, true, constraint_violation);
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