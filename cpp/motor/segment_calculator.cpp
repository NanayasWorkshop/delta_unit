#include "segment_calculator.hpp"
#include "../fabrik/fabrik_solver.hpp"  // For FabrikChain definition
#include <chrono>
#include <iostream>

namespace delta {

SegmentCalculationResult SegmentCalculator::calculate_segment_end_effectors(const FabrikChain& collision_free_chain) {
    return calculate_segment_end_effectors(collision_free_chain, collision_free_chain.num_robot_segments);
}

SegmentCalculationResult SegmentCalculator::calculate_segment_end_effectors(const FabrikChain& collision_free_chain,
                                                                           int num_robot_segments) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Validate inputs
    if (!validate_calculation_inputs(collision_free_chain, num_robot_segments)) {
        return SegmentCalculationResult(); // Failed result
    }
    
    // Perform the actual segment calculations
    std::vector<SegmentEndEffectorData> segment_data = extract_segment_data_from_chain(
        collision_free_chain, num_robot_segments);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double calculation_time_ms = duration.count() / 1000.0;
    
    return SegmentCalculationResult(segment_data, calculation_time_ms);
}

bool SegmentCalculator::is_chain_valid_for_segments(const FabrikChain& chain, int min_segments) {
    // Check basic chain validity
    if (chain.joints.size() < 2) {
        return false;
    }
    
    // Check that we have enough segments
    if (chain.num_robot_segments < min_segments) {
        return false;
    }
    
    // Check that joints and segments are consistent
    int expected_joints = chain.num_robot_segments + 2; // base + segments + end-effector
    if (static_cast<int>(chain.joints.size()) < expected_joints) {
        return false;
    }
    
    return true;
}

double SegmentCalculator::estimate_calculation_time(int num_robot_segments) {
    // Empirical estimates based on complexity of calculations
    // Base cost + per-segment cost
    double base_cost_ms = 0.05; // Base overhead
    double per_segment_cost_ms = 0.02; // Cost per segment calculation
    
    return base_cost_ms + (num_robot_segments * per_segment_cost_ms);
}

// Private methods

std::vector<SegmentEndEffectorData> SegmentCalculator::extract_segment_data_from_chain(const FabrikChain& solved_chain, 
                                                                                      int num_robot_segments) {
    std::vector<SegmentEndEffectorData> segment_data;
    
    if (num_robot_segments == 0 || solved_chain.joints.size() < 2) {
        return segment_data; // Empty result for invalid chains
    }
    
    // Extract direction pairs from solved chain (reuse existing FABRIK logic)
    std::vector<SegmentDirectionPair> direction_pairs = FabrikForward::extract_direction_pairs(solved_chain);
    
    // Calculate segment properties from direction pairs (reuse existing FABRIK logic)
    std::vector<SegmentProperties> segment_properties = FabrikForward::calculate_segment_properties(direction_pairs);
    
    // Calculate actual end-effector positions for each physical segment
    for (int seg = 0; seg < num_robot_segments; seg++) {
        // Calculate end-effector position for this segment
        Vector3 segment_end_effector = calculate_segment_end_effector_position(
            seg, solved_chain, segment_properties, num_robot_segments);
        
        // Get segment properties
        double prismatic_length = 0.0;
        double h_to_g_distance = 0.0;
        
        if (seg < static_cast<int>(segment_properties.size())) {
            prismatic_length = segment_properties[seg].prismatic_length;
            h_to_g_distance = segment_properties[seg].h_to_g_distance;
        }
        
        // Calculate additional derived values
        Vector3 direction_from_base = calculate_direction_from_base(segment_end_effector);
        double fabrik_distance = calculate_fabrik_distance_from_base(segment_end_effector, solved_chain);
        
        // Create segment data
        SegmentEndEffectorData data(
            seg + 1,                    // segment_number (1-based)
            segment_end_effector,       // end_effector_position
            direction_from_base,        // direction_from_base
            prismatic_length,           // prismatic_length
            h_to_g_distance,           // h_to_g_distance
            fabrik_distance            // fabrik_distance_from_base
        );
        
        segment_data.push_back(data);
        
        // Optional: Log segment calculation for debugging
        // log_segment_calculation(seg + 1, data, false);
    }
    
    return segment_data;
}

Vector3 SegmentCalculator::calculate_segment_end_effector_position(int segment_index,
                                                                  const FabrikChain& solved_chain,
                                                                  const std::vector<SegmentProperties>& segment_properties,
                                                                  int num_robot_segments) {
    
    if (segment_index == num_robot_segments - 1) {
        // Final segment (S3): Should be exactly at the target (end-effector)
        return solved_chain.joints.back().position;
    } else {
        // Segments S1 and S2: Calculate position between joints
        // S1: Between J1 and J2 → start from J1, go towards J2
        // S2: Between J2 and J3 → start from J2, go towards J3
        
        int joint_start = segment_index + 1;  // J[seg+1] - start from J1 for S1, J2 for S2
        int joint_end = segment_index + 2;    // J[seg+2] - go towards J2 for S1, J3 for S2
        
        if (joint_start >= static_cast<int>(solved_chain.joints.size()) || 
            joint_end >= static_cast<int>(solved_chain.joints.size())) {
            // Fallback to end-effector position
            return solved_chain.joints.back().position;
        }
        
        // Get joint positions
        Vector3 joint_start_pos = solved_chain.joints[joint_start].position;
        Vector3 joint_end_pos = solved_chain.joints[joint_end].position;
        
        // Calculate direction from joint_start to joint_end
        Vector3 segment_direction = (joint_end_pos - joint_start_pos).normalized();
        
        // Calculate the actual end-effector position for this segment
        double h_to_g_distance = 0.0;
        if (segment_index < static_cast<int>(segment_properties.size())) {
            h_to_g_distance = segment_properties[segment_index].h_to_g_distance;
        }
        
        double distance_to_end_effector = h_to_g_distance / 2.0 + WORKING_HEIGHT;
        
        return joint_start_pos + segment_direction * distance_to_end_effector;
    }
}

Vector3 SegmentCalculator::calculate_direction_from_base(const Vector3& end_effector_position) {
    // Calculate direction from world origin to this end-effector
    return end_effector_position.normalized();
}

double SegmentCalculator::calculate_fabrik_distance_from_base(const Vector3& end_effector_position, 
                                                             const FabrikChain& chain) {
    // Calculate distance along FABRIK chain from base to this point
    if (chain.joints.empty()) {
        return 0.0;
    }
    
    Vector3 base_position = chain.joints[0].position;
    return (end_effector_position - base_position).norm();
}

bool SegmentCalculator::validate_calculation_inputs(const FabrikChain& chain, int num_robot_segments) {
    // Check basic chain validity
    if (!is_chain_valid_for_segments(chain, num_robot_segments)) {
        std::cerr << "SegmentCalculator: Invalid FABRIK chain for segment calculations" << std::endl;
        return false;
    }
    
    // Check for reasonable number of robot segments
    if (num_robot_segments <= 0 || num_robot_segments > 20) {
        std::cerr << "SegmentCalculator: Invalid number of robot segments: " << num_robot_segments << std::endl;
        return false;
    }
    
    // Check that base joint is at origin (FABRIK requirement)
    Vector3 base_pos = chain.joints[0].position;
    if (base_pos.norm() > FABRIK_TOLERANCE) {
        std::cerr << "SegmentCalculator: Warning - Base joint not at origin (norm: " << base_pos.norm() << ")" << std::endl;
        // Don't fail, but warn - this might indicate FABRIK didn't converge properly
    }
    
    return true;
}

void SegmentCalculator::log_segment_calculation(int segment_number, const SegmentEndEffectorData& data, bool verbose) {
    if (verbose) {
        std::cout << "Segment " << segment_number << ":" << std::endl;
        std::cout << "  End-effector: (" 
                  << data.end_effector_position.x() << ", " 
                  << data.end_effector_position.y() << ", " 
                  << data.end_effector_position.z() << ")" << std::endl;
        std::cout << "  Direction: (" 
                  << data.direction_from_base.x() << ", " 
                  << data.direction_from_base.y() << ", " 
                  << data.direction_from_base.z() << ")" << std::endl;
        std::cout << "  Prismatic: " << data.prismatic_length 
                  << ", H-to-G: " << data.h_to_g_distance 
                  << ", FABRIK dist: " << data.fabrik_distance_from_base << std::endl;
    }
}

} // namespace delta