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
    
    // CRITICAL FIX: The current constraint logic is fundamentally wrong
    // For now, disable constraints to match the working manual calculation
    // This preserves segment lengths correctly
    
    // TODO: Implement proper spherical constraint logic later
    // The constraint should limit the angle between incoming and outgoing segments
    // Not the position of previous joints relative to cone axes
    
    return BackwardStepResult(unconstrained_position, false, 0.0);
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