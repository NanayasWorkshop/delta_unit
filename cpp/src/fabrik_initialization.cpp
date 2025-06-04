#include "fabrik_initialization.hpp"
#include <cmath>

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
        base_z = prev_segment_end.z();  // Fixed: Use .z() instead of .z
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
            base_z = calculate_segment_end_effector_position(seg - 1).z();  // Fixed: Use .z() instead of .z
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