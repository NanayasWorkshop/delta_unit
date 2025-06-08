#ifndef DELTA_WAYPOINT_CONVERTER_HPP
#define DELTA_WAYPOINT_CONVERTER_HPP

#include "../core/math_utils.hpp"
#include "../fabrik/fabrik_initialization.hpp"
#include <vector>

namespace delta {

// Result from waypoint to joints conversion
struct WaypointConversionResult {
    std::vector<Vector3> joint_positions;       // Converted joint positions (J0, J1, J2, ...)
    std::vector<double> segment_lengths;        // Calculated segment lengths
    std::vector<double> joint_angles_deg;       // Joint angles in degrees
    double total_reach;                         // Total reach of the chain
    bool conversion_successful;                 // Whether conversion was successful
    
    WaypointConversionResult(bool success = false) 
        : total_reach(0.0), conversion_successful(success) {}
};

// Convert collision-free waypoints back to FABRIK joint positions
class WaypointConverter {
public:
    // Main interface: convert waypoints to FABRIK joint positions
    static WaypointConversionResult convert_waypoints_to_joints(
        const std::vector<Vector3>& waypoints
    );
    
    // Validate that waypoints can form a valid robot chain
    static bool validate_waypoints(const std::vector<Vector3>& waypoints);
    
    // Create FABRIK chain from converted joint positions
    static FabrikChain create_fabrik_chain_from_waypoints(
        const std::vector<Vector3>& waypoints,
        int num_robot_segments
    );
    
    // Debug: Print conversion results
    static void debug_print_conversion(const WaypointConversionResult& result, 
                                     const std::string& label = "Waypoint Conversion");

private:
    // NEW: Core optimization methods (from Python algorithm)
    static Vector3 optimize_joint_position(const std::vector<Vector3>& waypoints,
                                         const std::vector<Vector3>& current_joints,
                                         size_t joint_index);
    
    static Vector3 optimize_first_joint(const Vector3& p1, const Vector3& p2);
    
    static Vector3 optimize_subsequent_joint(const Vector3& j_prev, 
                                           const Vector3& p_prev, 
                                           const Vector3& p_curr);
    
    // NEW: Triangle optimization methods
    static double calculate_triangle_angle_difference(const Vector3& p1, 
                                                    const Vector3& j, 
                                                    const Vector3& p2);
    
    // Calculate segment lengths between consecutive joints
    static std::vector<double> calculate_segment_lengths(const std::vector<Vector3>& joint_positions);
    
    // Calculate joint angles (bend angles between segments)
    static std::vector<double> calculate_joint_angles(const std::vector<Vector3>& joint_positions);
    
    // Calculate angle between two vectors in degrees
    static double calculate_angle_between_vectors(const Vector3& vec1, const Vector3& vec2);
    
    // Normalize vector with safety check
    static Vector3 normalize_vector_safe(const Vector3& vector);
    
    // Validate individual joint position
    static bool validate_joint_position(const Vector3& joint_pos, int joint_index);
    
    // Calculate total reach of joint chain
    static double calculate_total_reach(const std::vector<double>& segment_lengths);
};

} // namespace delta

#endif // DELTA_WAYPOINT_CONVERTER_HPP