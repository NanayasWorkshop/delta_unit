#include "collision_manager.hpp"
#include "../core/constants.hpp"
#include <iostream>

namespace delta {

CollisionManager& CollisionManager::getInstance() {
    static CollisionManager instance;
    return instance;
}

Vector3 CollisionManager::validate_joint_placement(const Vector3& proposed_position,
                                                  int joint_index,
                                                  PassType pass_type,
                                                  const FabrikChain& current_chain_state) {
    
    // Handle automatic pill management based on pass type
    if (pass_type == PassType::BACKWARD) {
        handle_backward_pill_management(joint_index, proposed_position, current_chain_state);
    } else if (pass_type == PassType::FORWARD) {
        handle_forward_pill_updates(current_chain_state);
    }
    
    // Update last pass type
    last_pass_type = pass_type;
    
    // For now, just return the same position (passthrough)
    return proposed_position;
}

void CollisionManager::handle_backward_pill_management(int joint_index, const Vector3& proposed_position, const FabrikChain& chain) {
    int num_joints = static_cast<int>(chain.joints.size());
    
    // Skip J8 (highest joint) - no collision check
    if (joint_index == num_joints - 1) {
        return;
    }
    
    // Approve J7 (J_Nmax-1) - direct approval
    if (joint_index == num_joints - 2) {
        return;
    }
    
    // For J6 and below (J_Nmax-2 and lower), create or update pills
    if (joint_index <= num_joints - 3) {
        create_or_update_pill(joint_index, proposed_position, chain);
    }
}

void CollisionManager::handle_forward_pill_updates(const FabrikChain& chain) {
    // Update all existing pills with new joint positions
    update_existing_pills(chain);
}

void CollisionManager::create_or_update_pill(int joint_index, const Vector3& proposed_position, const FabrikChain& chain) {
    int num_joints = static_cast<int>(chain.joints.size());
    
    // Calculate S-point (50% between current joint and next joint)
    Vector3 current_joint = proposed_position;
    Vector3 next_joint = chain.joints[joint_index + 1].position;
    Vector3 s_point = calculate_s_point(current_joint, next_joint);
    
    // Determine pill endpoints
    Vector3 pill_start, pill_end;
    
    if (joint_index == num_joints - 3) {
        // First pill: J8 (highest) -> S1
        pill_start = chain.joints[num_joints - 1].position; // J8
        pill_end = s_point; // S1
        
        // Create new pill or update existing first pill
        if (active_pills.empty()) {
            active_pills.emplace_back(pill_start, pill_end, COLLISION_PILL_RADIUS, joint_index);
        } else {
            active_pills[0].update_points(pill_start, pill_end);
        }
    } else if (joint_index == 0) {
        // SPECIAL CASE: Last pill goes all the way to base (0,0,0)
        pill_start = active_pills.empty() ? chain.joints[joint_index + 1].position : active_pills.back().end_point;
        pill_end = Vector3(0, 0, 0); // Force to base, not S-point
        
        // Find the pill index for this joint
        int pill_index = (num_joints - 3) - joint_index;
        
        if (pill_index < static_cast<int>(active_pills.size())) {
            // Update existing pill
            active_pills[pill_index].update_points(pill_start, pill_end);
        } else {
            // Create new pill
            active_pills.emplace_back(pill_start, pill_end, COLLISION_PILL_RADIUS, joint_index);
        }
    } else {
        // Normal pills: previous S-point -> current S-point
        pill_end = s_point;
        
        // Find the pill index for this joint
        int pill_index = (num_joints - 3) - joint_index;
        
        if (pill_index < static_cast<int>(active_pills.size())) {
            // Update existing pill
            active_pills[pill_index].end_point = pill_end;
            // Start point comes from previous pill's end point
            if (pill_index > 0) {
                active_pills[pill_index].start_point = active_pills[pill_index - 1].end_point;
            }
        } else {
            // Create new pill
            pill_start = active_pills.empty() ? chain.joints[joint_index + 2].position : active_pills.back().end_point;
            active_pills.emplace_back(pill_start, pill_end, COLLISION_PILL_RADIUS, joint_index);
        }
    }
}

void CollisionManager::update_existing_pills(const FabrikChain& chain) {
    int num_joints = static_cast<int>(chain.joints.size());
    
    // Update all pills based on current joint positions
    for (int i = 0; i < static_cast<int>(active_pills.size()); ++i) {
        CollisionPill& pill = active_pills[i];
        int joint_idx = pill.associated_joint_index;
        
        if (joint_idx == 0) {
            // SPECIAL CASE: Last pill always ends at base (0,0,0)
            pill.end_point = Vector3(0, 0, 0);
            if (i > 0) {
                pill.start_point = active_pills[i - 1].end_point;
            }
        } else if (joint_idx < num_joints - 1) {
            Vector3 current_joint = chain.joints[joint_idx].position;
            Vector3 next_joint = chain.joints[joint_idx + 1].position;
            Vector3 new_s_point = calculate_s_point(current_joint, next_joint);
            
            // Update pill endpoint
            pill.end_point = new_s_point;
            
            // Update start point for first pill
            if (joint_idx == num_joints - 3) {
                pill.start_point = chain.joints[num_joints - 1].position;
            } else if (i > 0) {
                // Chain pills together
                pill.start_point = active_pills[i - 1].end_point;
            }
        }
    }
}

const std::vector<CollisionPill>& CollisionManager::get_active_pills() const {
    return active_pills;
}

Vector3 CollisionManager::calculate_s_point(const Vector3& joint1, const Vector3& joint2) const {
    return (joint1 + joint2) * 0.5; // 50% midpoint
}

} // namespace delta