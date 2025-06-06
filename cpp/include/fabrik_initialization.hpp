#ifndef DELTA_FABRIK_INITIALIZATION_HPP
#define DELTA_FABRIK_INITIALIZATION_HPP

#include "math_utils.hpp"
#include "../core/constants.hpp"
#include <vector>

namespace delta {

// Joint types
enum class JointType {
    FIXED_BASE,      // Base connector (0,0,0)
    SPHERICAL_120,   // Spherical joint with 120° constraint
    END_EFFECTOR     // Final end-effector point
};

// Single joint in the chain
struct FabrikJoint {
    Vector3 position;           // Current joint position
    JointType type;            // Type of joint
    double constraint_angle;   // Constraint angle (120° for spherical)
    
    FabrikJoint(const Vector3& pos, JointType joint_type, double constraint = 0.0)
        : position(pos), type(joint_type), constraint_angle(constraint) {}
};

// Segment between two joints
struct FabrikSegment {
    int start_joint_index;     // Index of start joint
    int end_joint_index;       // Index of end joint
    double length;             // Fixed length between joints
    
    FabrikSegment(int start_idx, int end_idx, double seg_length)
        : start_joint_index(start_idx), end_joint_index(end_idx), length(seg_length) {}
};

// Complete robot chain
struct FabrikChain {
    std::vector<FabrikJoint> joints;       // All joints in order
    std::vector<FabrikSegment> segments;   // All segments
    int num_robot_segments;                // Number of robot segments (8 in your example)
    
    FabrikChain(int robot_segments) : num_robot_segments(robot_segments) {}
};

// Result of initialization
struct FabrikInitResult {
    FabrikChain chain;                     // Complete initialized chain
    Vector3 final_end_effector;           // Final end-effector position
    double total_reach;                    // Maximum reach of the chain
    
    FabrikInitResult(const FabrikChain& fabrik_chain, const Vector3& end_eff, double reach)
        : chain(fabrik_chain), final_end_effector(end_eff), total_reach(reach) {}
};

class FabrikInitialization {
public:
    // Main interface: Initialize chain with N robot segments
    static FabrikInitResult initialize_straight_up(int num_robot_segments = DEFAULT_ROBOT_SEGMENTS);
    
    // NEW: Initialize from provided joint positions
    static FabrikInitResult initialize_from_joint_positions(int num_robot_segments,
                                                           const std::vector<Vector3>& joint_positions);
    
    // Initialize with custom direction (for later)
    static FabrikInitResult initialize_with_direction(int num_robot_segments, 
                                                     const Vector3& direction);
    
    // Helper functions
    static double calculate_segment_length(double hypotenuse_distance, double angle_rad = 0.0);
    static Vector3 calculate_joint_position(int robot_segment_index, int joint_index_in_segment);
    
    // Utility functions
    static double get_total_reach(int num_robot_segments);
    static int get_total_joints(int num_robot_segments);
    
    // NEW: Validation functions
    static bool validate_joint_positions(int num_robot_segments, const std::vector<Vector3>& joint_positions);
    static bool is_base_at_origin(const Vector3& base_position, double tolerance = 1e-6);
    
private:
    // Internal calculation methods
    static double calculate_hypotenuse_distance(double prismatic_length = 0.0);
    static FabrikChain create_straight_chain(int num_robot_segments);
    static FabrikChain create_chain_from_positions(int num_robot_segments, const std::vector<Vector3>& joint_positions);
    static double calculate_total_reach(const FabrikChain& chain);
    static Vector3 calculate_segment_end_effector_position(int segment_index);
};

} // namespace delta

#endif // DELTA_FABRIK_INITIALIZATION_HPP