#include "fabrik_initialization.hpp"
#include <cmath>
#include <iostream>

namespace delta {

FabrikInitResult FabrikInitialization::initialize_straight_up(int num_robot_segments) {
    // Create the chain structure
    FabrikChain chain = create_straight_chain(num_robot_segments);
    
    // Calculate final end-effector position
    Vector3 final_end_effector = calculate_segment_end_effector_position(num_robot_segments - 1);
    
    // Calculate total reach
    double total_reach = calculate_total_reach(chain);
    
    return FabrikInitResult(chain, final_end_effector, total_reach);
}

FabrikInitResult FabrikInitialization::initialize_from_joint_positions(int num_robot_segments,
                                                                      const std::vector<Vector3>& joint_positions) {
    // Validate input
    if (!validate_joint_positions(num_robot_segments, joint_positions)) {
        std::cerr << "Warning: Invalid joint positions provided. Falling back to straight-up initialization." << std::endl;
        return initialize_straight_up(num_robot_segments);
    }
    
    // Create chain from provided positions
    FabrikChain chain = create_chain_from_positions(num_robot_segments, joint_positions);
    
    // Final end-effector is the last joint position
    Vector3 final_end_effector = joint_positions.back();
    
    // Calculate total reach
    double total_reach = calculate_total_reach(chain);
    
    return FabrikInitResult(chain, final_end_effector, total_reach);
}

FabrikInitResult FabrikInitialization::initialize_with_direction(int num_robot_segments, 
                                                                const Vector3& direction) {
    // For now, this is a placeholder - we'll implement this later
    // when we handle non-straight configurations
    return initialize_straight_up(num_robot_segments);
}

double FabrikInitialization::calculate_segment_length(double hypotenuse_distance, double angle_rad) {
    // For straight up (angle = 0), this simplifies to hypotenuse / 2
    // General formula: segment_length = hypotenuse / (2 * cos(angle/2))
    if (std::abs(angle_rad) < 1e-10) {
        return hypotenuse_distance / 2.0;
    }
    
    double half_angle = angle_rad / 2.0;
    return hypotenuse_distance / (2.0 * std::cos(half_angle));
}

Vector3 FabrikInitialization::calculate_joint_position(int robot_segment_index, int joint_index_in_segment) {
    // Calculate position based on segment and joint index
    // robot_segment_index: 0, 1, 2, ... (which robot segment)
    // joint_index_in_segment: 0=base, 1=spherical_joint, 2=end_effector
    
    double base_z = 0.0;
    
    // Calculate the base Z position for this segment
    for (int i = 0; i < robot_segment_index; i++) {
        Vector3 prev_segment_end = calculate_segment_end_effector_position(i);
        base_z = prev_segment_end.z();
    }
    
    // Calculate positions within this segment
    double segment_length = calculate_segment_length(calculate_hypotenuse_distance());
    
    switch (joint_index_in_segment) {
        case 0: // Base of segment
            return Vector3(0, 0, base_z);
            
        case 1: // Spherical joint (H + segment_length)
            return Vector3(0, 0, base_z + WORKING_HEIGHT + segment_length);
            
        case 2: // End-effector of segment
            return calculate_segment_end_effector_position(robot_segment_index);
            
        default:
            return Vector3(0, 0, base_z);
    }
}

double FabrikInitialization::get_total_reach(int num_robot_segments) {
    // Calculate maximum possible reach of the robot
    double hypotenuse = calculate_hypotenuse_distance();
    return num_robot_segments * hypotenuse;
}

int FabrikInitialization::get_total_joints(int num_robot_segments) {
    // Each segment contributes: base + spherical_joint
    // Plus one final end-effector
    return num_robot_segments + 1;
}

// NEW: Validation functions

bool FabrikInitialization::validate_joint_positions(int num_robot_segments, const std::vector<Vector3>& joint_positions) {
    // Check joint count: should be num_robot_segments + 2 (base + segments + end-effector)
    int expected_joints = num_robot_segments + 2;
    if (static_cast<int>(joint_positions.size()) != expected_joints) {
        std::cerr << "Expected " << expected_joints << " joint positions, got " << joint_positions.size() << std::endl;
        return false;
    }
    
    // Check that base is at origin
    if (!is_base_at_origin(joint_positions[0])) {
        std::cerr << "Base joint must be at origin (0,0,0), got (" 
                  << joint_positions[0].x() << ", " << joint_positions[0].y() << ", " << joint_positions[0].z() << ")" << std::endl;
        return false;
    }
    
    // Check that joints are reasonably spaced (basic sanity check)
    for (int i = 1; i < static_cast<int>(joint_positions.size()); i++) {
        Vector3 prev_joint = joint_positions[i-1];
        Vector3 curr_joint = joint_positions[i];
        double distance = (curr_joint - prev_joint).norm();
        
        // Check minimum distance (avoid degenerate segments)
        if (distance < 1.0) {  // Minimum 1mm spacing
            std::cerr << "Joints " << i-1 << " and " << i << " are too close: " << distance << "mm" << std::endl;
            return false;
        }
        
        // Check maximum distance (avoid impossible segments)
        double max_reasonable_distance = 2000.0;  // 2m maximum segment length
        if (distance > max_reasonable_distance) {
            std::cerr << "Joints " << i-1 << " and " << i << " are too far apart: " << distance << "mm" << std::endl;
            return false;
        }
    }
    
    return true;
}

bool FabrikInitialization::is_base_at_origin(const Vector3& base_position, double tolerance) {
    return base_position.norm() <= tolerance;
}

// Private methods

double FabrikInitialization::calculate_hypotenuse_distance(double prismatic_length) {
    // From your kinematics: MIN_HEIGHT + 2*MOTOR_LIMIT + prismatic_length
    return MIN_HEIGHT + 2.0 * MOTOR_LIMIT + prismatic_length;
}

FabrikChain FabrikInitialization::create_straight_chain(int num_robot_segments) {
    FabrikChain chain(num_robot_segments);
    
    double segment_length = calculate_segment_length(calculate_hypotenuse_distance());
    
    // Create joints
    int joint_index = 0;
    
    // Base joint (fixed)
    chain.joints.push_back(FabrikJoint(Vector3(0, 0, 0), JointType::FIXED_BASE));
    
    // For each robot segment
    for (int seg = 0; seg < num_robot_segments; seg++) {
        double base_z = 0.0;
        
        // Calculate base Z for this segment
        if (seg > 0) {
            base_z = calculate_segment_end_effector_position(seg - 1).z();
        }
        
        // Spherical joint position
        double joint_z = base_z + WORKING_HEIGHT + segment_length;
        chain.joints.push_back(FabrikJoint(
            Vector3(0, 0, joint_z), 
            JointType::SPHERICAL_120, 
            SPHERICAL_JOINT_CONE_ANGLE_RAD
        ));
        joint_index++;
    }
    
    // Final end-effector
    Vector3 final_pos = calculate_segment_end_effector_position(num_robot_segments - 1);
    chain.joints.push_back(FabrikJoint(final_pos, JointType::END_EFFECTOR));
    
    // Create segments
    for (int i = 0; i < static_cast<int>(chain.joints.size()) - 1; i++) {
        Vector3 start_pos = chain.joints[i].position;
        Vector3 end_pos = chain.joints[i + 1].position;
        double length = (end_pos - start_pos).norm();
        
        chain.segments.push_back(FabrikSegment(i, i + 1, length));
    }
    
    return chain;
}

FabrikChain FabrikInitialization::create_chain_from_positions(int num_robot_segments, const std::vector<Vector3>& joint_positions) {
    FabrikChain chain(num_robot_segments);
    
    // Create joints from provided positions
    // Joint 0: Base (FIXED_BASE)
    chain.joints.push_back(FabrikJoint(joint_positions[0], JointType::FIXED_BASE));
    
    // Joints 1 to num_robot_segments: Spherical joints (SPHERICAL_120)
    for (int i = 1; i <= num_robot_segments; i++) {
        chain.joints.push_back(FabrikJoint(
            joint_positions[i], 
            JointType::SPHERICAL_120, 
            SPHERICAL_JOINT_CONE_ANGLE_RAD
        ));
    }
    
    // Final joint: End-effector (END_EFFECTOR)
    chain.joints.push_back(FabrikJoint(joint_positions.back(), JointType::END_EFFECTOR));
    
    // Create segments between consecutive joints
    for (int i = 0; i < static_cast<int>(chain.joints.size()) - 1; i++) {
        Vector3 start_pos = chain.joints[i].position;
        Vector3 end_pos = chain.joints[i + 1].position;
        double length = (end_pos - start_pos).norm();
        
        chain.segments.push_back(FabrikSegment(i, i + 1, length));
    }
    
    return chain;
}

double FabrikInitialization::calculate_total_reach(const FabrikChain& chain) {
    double total = 0.0;
    for (const auto& segment : chain.segments) {
        total += segment.length;
    }
    return total;
}

Vector3 FabrikInitialization::calculate_segment_end_effector_position(int segment_index) {
    // Calculate the end-effector position for a specific segment
    double segment_length = calculate_segment_length(calculate_hypotenuse_distance());
    
    double base_z = 0.0;
    
    // Stack segments
    for (int i = 0; i <= segment_index; i++) {
        if (i > 0) {
            base_z += WORKING_HEIGHT + 2 * segment_length + WORKING_HEIGHT;
        } else {
            base_z = WORKING_HEIGHT + 2 * segment_length + WORKING_HEIGHT;
        }
    }
    
    return Vector3(0, 0, base_z);
}

} // namespace delta