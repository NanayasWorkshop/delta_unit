#ifndef DELTA_SEGMENT_CALCULATOR_HPP
#define DELTA_SEGMENT_CALCULATOR_HPP

#include "../core/math_utils.hpp"
#include "../fabrik/fabrik_forward.hpp"
#include <vector>

// Forward declarations to avoid circular includes
namespace delta {
    struct FabrikChain;
}

namespace delta {

// Data for each physical segment's end-effector (moved from fabrik_solver.hpp)
struct SegmentEndEffectorData {
    int segment_number;                    // Which physical segment (1, 2, 3, etc.)
    Vector3 end_effector_position;         // 3D position of this segment's end-effector
    Vector3 direction_from_base;           // Direction vector from world origin to this end-effector
    double prismatic_length;               // This segment's prismatic length
    double h_to_g_distance;                // This segment's H→G distance
    double fabrik_distance_from_base;      // Distance along FABRIK chain from base to this end-effector
    
    SegmentEndEffectorData(int seg_num, const Vector3& pos, const Vector3& dir, 
                          double prismatic, double h_to_g, double fabrik_dist)
        : segment_number(seg_num), end_effector_position(pos), direction_from_base(dir)
        , prismatic_length(prismatic), h_to_g_distance(h_to_g), fabrik_distance_from_base(fabrik_dist) {}
};

// Result from segment calculations
struct SegmentCalculationResult {
    std::vector<SegmentEndEffectorData> segment_end_effectors;  // Calculated segment data
    double calculation_time_ms;                                 // Time taken for calculations
    bool calculation_successful;                                // Whether calculations completed successfully
    
    SegmentCalculationResult() 
        : calculation_time_ms(0.0), calculation_successful(false) {}
        
    SegmentCalculationResult(const std::vector<SegmentEndEffectorData>& segments, double time_ms)
        : segment_end_effectors(segments), calculation_time_ms(time_ms), calculation_successful(true) {}
};

/**
 * SegmentCalculator - Calculate complex S1, S2, S3 segment end-effector positions
 * 
 * This component performs the expensive transformations to calculate actual physical
 * segment end-effector positions from a solved FABRIK chain. It should only be called
 * AFTER collision-aware solving has found a collision-free solution.
 * 
 * Moved from FabrikSolver to maintain clean separation of concerns and optimize
 * performance during collision detection iterations.
 */
class SegmentCalculator {
public:
    /**
     * Calculate segment end-effector positions from a collision-free FABRIK chain
     * 
     * @param collision_free_chain Final FABRIK chain that is collision-free
     * @return SegmentCalculationResult with calculated segment data
     */
    static SegmentCalculationResult calculate_segment_end_effectors(const FabrikChain& collision_free_chain);
    
    /**
     * Calculate segment end-effectors with custom number of robot segments
     * 
     * @param collision_free_chain Final FABRIK chain that is collision-free
     * @param num_robot_segments Number of physical robot segments to calculate
     * @return SegmentCalculationResult with calculated segment data
     */
    static SegmentCalculationResult calculate_segment_end_effectors(const FabrikChain& collision_free_chain,
                                                                   int num_robot_segments);
    
    /**
     * Validate that a FABRIK chain is suitable for segment calculations
     * 
     * @param chain FABRIK chain to validate
     * @param min_segments Minimum number of segments required
     * @return true if chain is valid for segment calculations
     */
    static bool is_chain_valid_for_segments(const FabrikChain& chain, int min_segments = 1);
    
    /**
     * Get the total computational cost estimate for segment calculations
     * 
     * @param num_robot_segments Number of segments to calculate
     * @return Estimated computation time in milliseconds
     */
    static double estimate_calculation_time(int num_robot_segments);

private:
    /**
     * Core implementation of segment end-effector calculations
     * (Moved from FabrikSolver::extract_segment_end_effectors)
     */
    static std::vector<SegmentEndEffectorData> extract_segment_data_from_chain(const FabrikChain& solved_chain, 
                                                                              int num_robot_segments);
    
    /**
     * Calculate end-effector position for a specific segment
     * 
     * @param segment_index Which segment (0-based: 0=S1, 1=S2, 2=S3, etc.)
     * @param solved_chain FABRIK chain with joint positions
     * @param segment_properties Calculated segment properties
     * @param num_robot_segments Total number of robot segments
     * @return Calculated end-effector position for this segment
     */
    static Vector3 calculate_segment_end_effector_position(int segment_index,
                                                          const FabrikChain& solved_chain,
                                                          const std::vector<SegmentProperties>& segment_properties,
                                                          int num_robot_segments);
    
    /**
     * Calculate direction from world origin to segment end-effector
     */
    static Vector3 calculate_direction_from_base(const Vector3& end_effector_position);
    
    /**
     * Calculate distance along FABRIK chain from base to segment end-effector
     */
    static double calculate_fabrik_distance_from_base(const Vector3& end_effector_position, 
                                                     const FabrikChain& chain);
    
    /**
     * Validate segment calculation inputs
     */
    static bool validate_calculation_inputs(const FabrikChain& chain, int num_robot_segments);
    
    /**
     * Log segment calculation details (for debugging)
     */
    static void log_segment_calculation(int segment_number, const SegmentEndEffectorData& data, bool verbose = false);
};

} // namespace delta

#endif // DELTA_SEGMENT_CALCULATOR_HPP