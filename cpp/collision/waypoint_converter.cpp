#include "waypoint_converter.hpp"
#include "../core/constants.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>

namespace delta {

WaypointConversionResult WaypointConverter::convert_waypoints_to_joints(
    const std::vector<Vector3>& waypoints) {
    
    if (waypoints.size() < 2) {
        return WaypointConversionResult(false);
    }
    
    // SIMPLIFIED WAYPOINT TO JOINT MAPPING:
    // Instead of complex triangle optimization, use simple 50% midpoint approach
    // Joint[i] = 50% between waypoint[i-1] and waypoint[i]
    
    std::vector<Vector3> joint_positions;
    std::vector<double> segment_lengths;
    std::vector<double> joint_angles_deg;
    
    try {
        // Step 1: Base joint (J0) = first waypoint (P1) - direct mapping
        joint_positions.push_back(waypoints[0]);
        
        // Step 2: SIMPLIFIED - Place each joint at 50% between consecutive waypoints
        for (size_t i = 1; i < waypoints.size(); ++i) {
            Vector3 prev_waypoint = waypoints[i-1];
            Vector3 curr_waypoint = waypoints[i];
            
            // Simple 50% midpoint calculation
            Vector3 joint_position = (prev_waypoint + curr_waypoint) / 2.0;
            joint_positions.push_back(joint_position);
        }
        
        // Step 3: End-effector joint (J8) = last waypoint (u8) - direct mapping
        joint_positions.push_back(waypoints.back());
        
        // Step 4: Calculate segment lengths between consecutive joints
        segment_lengths = calculate_segment_lengths(joint_positions);
        
        // Step 5: Calculate joint angles (bend angles between segments)
        joint_angles_deg = calculate_joint_angles(joint_positions);
        
        // Step 6: Calculate total reach
        double total_reach = calculate_total_reach(segment_lengths);
        
        // Step 7: Create successful result
        WaypointConversionResult result(true);
        result.joint_positions = joint_positions;
        result.segment_lengths = segment_lengths;
        result.joint_angles_deg = joint_angles_deg;
        result.total_reach = total_reach;
        
        return result;
        
    } catch (const std::exception& e) {
        std::cerr << "Waypoint conversion failed: " << e.what() << std::endl;
        return WaypointConversionResult(false);
    }
}

// COMMENTED OUT - Original complex triangle optimization methods
/*
Vector3 WaypointConverter::optimize_joint_position(const std::vector<Vector3>& waypoints,
                                                  const std::vector<Vector3>& current_joints,
                                                  size_t joint_index) {
    
    if (joint_index == 0) {
        // First joint optimization: J1 between P1 and P2
        return optimize_first_joint(waypoints[0], waypoints[1]);
    }
    
    // Subsequent joint optimization: Ji between Pi and Pi+1
    Vector3 p_prev = waypoints[joint_index];      // Pi (previous waypoint)
    Vector3 p_curr = waypoints[joint_index + 1];  // Pi+1 (current waypoint)
    Vector3 j_prev = current_joints.back();       // Ji-1 (previous joint)
    
    return optimize_subsequent_joint(j_prev, p_prev, p_curr);
}

Vector3 WaypointConverter::optimize_first_joint(const Vector3& p1, const Vector3& p2) {
    // Optimize J1 position to minimize angle difference in triangle P1-J1-P2
    // Constraint: J1 lies on Z-axis (robot-specific constraint)
    
    double best_z = 0.0;
    double min_angle_diff = std::numeric_limits<double>::infinity();
    
    // Search along Z-axis for optimal position
    double z_min = std::min(p1.z(), p2.z()) - 50.0;  // Search range
    double z_max = std::max(p1.z(), p2.z()) + 50.0;
    double z_step = 1.0;  // 1mm precision
    
    for (double z = z_min; z <= z_max; z += z_step) {
        Vector3 j1_candidate(0, 0, z);  // On Z-axis
        
        // Calculate angle difference for this candidate
        double angle_diff = calculate_triangle_angle_difference(p1, j1_candidate, p2);
        
        if (angle_diff < min_angle_diff) {
            min_angle_diff = angle_diff;
            best_z = z;
        }
    }
    
    return Vector3(0, 0, best_z);
}

Vector3 WaypointConverter::optimize_subsequent_joint(const Vector3& j_prev, 
                                                    const Vector3& p_prev, 
                                                    const Vector3& p_curr) {
    // Optimize Ji position along line from j_prev through p_prev
    // This maintains the kinematic chain constraint
    
    Vector3 direction = p_prev - j_prev;
    double direction_norm = direction.norm();
    
    if (direction_norm < EPSILON_MATH) {
        // Fallback: place joint at p_prev
        return p_prev;
    }
    
    Vector3 direction_unit = direction / direction_norm;
    
    double best_t = 0.0;
    double min_angle_diff = std::numeric_limits<double>::infinity();
    
    // Search along the line j_prev + t * direction_unit
    double t_min = -200.0;  // Search range in mm
    double t_max = 400.0;
    double t_step = 2.0;    // 2mm precision
    
    for (double t = t_min; t <= t_max; t += t_step) {
        Vector3 j_candidate = j_prev + t * direction_unit;
        
        // Calculate angle difference for triangle p_prev-j_candidate-p_curr
        double angle_diff = calculate_triangle_angle_difference(p_prev, j_candidate, p_curr);
        
        if (angle_diff < min_angle_diff) {
            min_angle_diff = angle_diff;
            best_t = t;
        }
    }
    
    return j_prev + best_t * direction_unit;
}

double WaypointConverter::calculate_triangle_angle_difference(const Vector3& p1, 
                                                            const Vector3& j, 
                                                            const Vector3& p2) {
    // Calculate angle difference between angles at p1 and p2 in triangle p1-j-p2
    
    // Vectors from p1
    Vector3 p1_j = j - p1;
    Vector3 p1_p2 = p2 - p1;
    
    // Vectors from p2
    Vector3 p2_j = j - p2;
    Vector3 p2_p1 = p1 - p2;
    
    // Calculate angles using dot product
    double angle1 = calculate_angle_between_vectors(p1_j, p1_p2);
    double angle2 = calculate_angle_between_vectors(p2_j, p2_p1);
    
    return std::abs(angle1 - angle2);
}
*/

double WaypointConverter::calculate_angle_between_vectors(const Vector3& v1, const Vector3& v2) {
    double v1_norm = v1.norm();
    double v2_norm = v2.norm();
    
    if (v1_norm < EPSILON_MATH || v2_norm < EPSILON_MATH) {
        return 0.0;
    }
    
    double dot_product = v1.dot(v2);
    double cos_angle = dot_product / (v1_norm * v2_norm);
    
    // Clamp to valid range for acos
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
    
    return std::acos(cos_angle) * 180.0 / M_PI;  // Convert to degrees
}

bool WaypointConverter::validate_waypoints(const std::vector<Vector3>& waypoints) {
    if (waypoints.size() < 2) {
        std::cerr << "Need at least 2 waypoints" << std::endl;
        return false;
    }
    
    // Check that waypoints are reasonably spaced
    for (size_t i = 1; i < waypoints.size(); ++i) {
        double distance = (waypoints[i] - waypoints[i-1]).norm();
        
        if (distance < 1.0) {  // Minimum 1mm spacing
            std::cerr << "Waypoints " << i-1 << " and " << i << " too close: " << distance << "mm" << std::endl;
            return false;
        }
        
        if (distance > 500.0) {  // Maximum 500mm spacing
            std::cerr << "Waypoints " << i-1 << " and " << i << " too far: " << distance << "mm" << std::endl;
            return false;
        }
    }
    
    return true;
}

FabrikChain WaypointConverter::create_fabrik_chain_from_waypoints(
    const std::vector<Vector3>& waypoints,
    int num_robot_segments) {
    
    // Convert waypoints to joint positions first
    WaypointConversionResult conversion = convert_waypoints_to_joints(waypoints);
    
    if (!conversion.conversion_successful) {
        // Return empty chain on failure
        return FabrikChain(num_robot_segments);
    }
    
    // Use FabrikInitialization PUBLIC method to create chain from converted positions
    FabrikInitResult init_result = FabrikInitialization::initialize_from_joint_positions(
        num_robot_segments, conversion.joint_positions);
    
    return init_result.chain;
}

void WaypointConverter::debug_print_conversion(const WaypointConversionResult& result, 
                                             const std::string& label) {
    std::cout << "\n=== " << label << " ===" << std::endl;
    std::cout << "Conversion successful: " << (result.conversion_successful ? "YES" : "NO") << std::endl;
    
    if (!result.conversion_successful) {
        return;
    }
    
    std::cout << "Joint positions (" << result.joint_positions.size() << "):" << std::endl;
    for (size_t i = 0; i < result.joint_positions.size(); ++i) {
        const Vector3& pos = result.joint_positions[i];
        std::cout << "  J" << i << ": (" << std::setprecision(3) << std::fixed
                  << pos.x() << ", " << pos.y() << ", " << pos.z() << ")" << std::endl;
    }
    
    std::cout << "Segment lengths (" << result.segment_lengths.size() << "):" << std::endl;
    for (size_t i = 0; i < result.segment_lengths.size(); ++i) {
        std::cout << "  Segment " << i << ": " << std::setprecision(2) << std::fixed
                  << result.segment_lengths[i] << "mm" << std::endl;
    }
    
    std::cout << "Joint angles (" << result.joint_angles_deg.size() << "):" << std::endl;
    for (size_t i = 0; i < result.joint_angles_deg.size(); ++i) {
        std::cout << "  Joint " << i+1 << ": " << std::setprecision(1) << std::fixed
                  << result.joint_angles_deg[i] << "°" << std::endl;
    }
    
    std::cout << "Total reach: " << std::setprecision(1) << std::fixed 
              << result.total_reach << "mm" << std::endl;
}

// Private utility methods

std::vector<double> WaypointConverter::calculate_segment_lengths(const std::vector<Vector3>& joint_positions) {
    std::vector<double> lengths;
    
    for (size_t i = 1; i < joint_positions.size(); ++i) {
        double length = (joint_positions[i] - joint_positions[i-1]).norm();
        lengths.push_back(length);
    }
    
    return lengths;
}

std::vector<double> WaypointConverter::calculate_joint_angles(const std::vector<Vector3>& joint_positions) {
    std::vector<double> angles;
    
    // Calculate bend angles between consecutive segments
    for (size_t i = 1; i < joint_positions.size() - 1; ++i) {
        Vector3 seg1 = joint_positions[i] - joint_positions[i-1];    // Previous segment
        Vector3 seg2 = joint_positions[i+1] - joint_positions[i];    // Next segment
        
        double angle = calculate_angle_between_vectors(-seg1, seg2);  // Bend angle
        angles.push_back(angle);
    }
    
    return angles;
}

Vector3 WaypointConverter::normalize_vector_safe(const Vector3& vector) {
    double norm = vector.norm();
    if (norm < EPSILON_MATH) {
        return Vector3(0, 0, 1);  // Default to Z+ direction
    }
    return vector / norm;
}

// REMOVED: validate_joint_position - was incorrect validation logic

double WaypointConverter::calculate_total_reach(const std::vector<double>& segment_lengths) {
    double total = 0.0;
    for (double length : segment_lengths) {
        total += length;
    }
    return total;
}

} // namespace delta