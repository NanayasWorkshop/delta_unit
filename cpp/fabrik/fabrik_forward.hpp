// fabrik_forward.hpp - Forward iteration module

#ifndef DELTA_FABRIK_FORWARD_HPP
#define DELTA_FABRIK_FORWARD_HPP

#include "../core/math_utils.hpp"
#include "../core/constants.hpp"
#include "fabrik_initialization.hpp"
#include "kinematics_module.hpp"

namespace delta {

// Result of forward iteration
struct FabrikForwardResult {
    FabrikChain updated_chain;                  // Chain with updated joint positions
    Vector3 base_position;                      // Base position (should be 0,0,0)
    Vector3 final_end_effector;                 // Final end-effector position
    bool constraints_satisfied;                 // Whether all constraints were satisfied
    int iterations_used;                        // Number of iterations performed
    std::vector<Vector3> iteration_history;     // History of base positions per iteration
    std::vector<double> recalculated_lengths;   // New segment lengths after recalculation
    
    FabrikForwardResult(const FabrikChain& chain, const Vector3& base, const Vector3& end_eff,
                       bool constraints_ok, int iterations)
        : updated_chain(chain), base_position(base), final_end_effector(end_eff)
        , constraints_satisfied(constraints_ok), iterations_used(iterations) {}
};

// Direction vectors for one segment pair
struct SegmentDirectionPair {
    Vector3 reference_direction;    // Reference direction (e.g., [0]→[1])
    Vector3 target_direction;       // Target direction (e.g., [1]→[2])
    int segment_index;              // Which physical segment this represents
    
    SegmentDirectionPair(const Vector3& ref, const Vector3& target, int idx)
        : reference_direction(ref), target_direction(target), segment_index(idx) {}
};

// Calculated segment properties
struct SegmentProperties {
    double prismatic_length;        // Calculated prismatic length
    double h_to_g_distance;         // H→G distance for this segment
    double fabrik_segment_length;   // Corresponding FABRIK segment length
    Vector3 transformed_direction;  // Direction after coordinate transformation
    
    SegmentProperties(double prismatic, double h_to_g, double fabrik_len, const Vector3& dir)
        : prismatic_length(prismatic), h_to_g_distance(h_to_g)
        , fabrik_segment_length(fabrik_len), transformed_direction(dir) {}
};

class FabrikForward {
public:
    // Main interface: perform forward iteration from base to end-effector
    static FabrikForwardResult iterate_from_base(const FabrikChain& backward_result_chain,
                                                double tolerance = FABRIK_TOLERANCE,
                                                int max_iterations = FABRIK_MAX_ITERATIONS);
    
    // Single forward iteration step with cone constraint algorithm
    static FabrikChain single_forward_iteration(const FabrikChain& chain_state_before_pass,
                                               const std::vector<double>& target_segment_lengths);
    
    // Calculate new segment lengths from backward iteration result
    static std::vector<double> calculate_new_segment_lengths(const FabrikChain& backward_result);
    
    // Utility: Get base position from chain (should be 0,0,0 after forward iteration)
    static Vector3 get_base_position(const FabrikChain& chain);
    
    // Validation: Check if base is at origin
    static bool is_base_at_origin(const FabrikChain& chain, double tolerance = 0.01);
    
    // UPDATED! Expose these methods for use by FabrikSolver::extract_segment_end_effectors
    static std::vector<SegmentDirectionPair> extract_direction_pairs(const FabrikChain& chain);
    static std::vector<SegmentProperties> calculate_segment_properties(
        const std::vector<SegmentDirectionPair>& direction_pairs);
    
private:
    // Transform direction vectors to Z+ reference coordinate system
    static Vector3 transform_to_z_reference(const Vector3& reference_direction, 
                                           const Vector3& target_direction);
    
    // Calculate prismatic length using existing modules
    static double calculate_prismatic_from_direction(const Vector3& transformed_direction);
    
    // Convert segment properties to FABRIK segment lengths
    static std::vector<double> convert_to_fabrik_lengths(
        const std::vector<SegmentProperties>& segment_properties,
        int num_robot_segments);
    
    // Check convergence criteria
    static bool has_converged_forward(const Vector3& current_base,
                                    const Vector3& target_base,
                                    double tolerance);
    
    // Cross product helper (since Vector3 doesn't have cross method)
    static Vector3 cross_product(const Vector3& a, const Vector3& b);
    
    // Rodrigues rotation for coordinate transformation
    static Vector3 rodrigues_rotation(const Vector3& v, const Vector3& axis, double angle);
};

// Helper utilities for modular segment handling
namespace fabrik_forward_utils {
    
    // Get number of direction pairs needed for N robot segments
    inline int get_direction_pairs_count(int num_robot_segments) {
        return num_robot_segments; // [0→1,1→2], [1→2,2→3], [2→3,3→4], etc.
    }
    
    // Get FABRIK segment index mapping
    inline std::vector<int> get_fabrik_segment_indices(int num_robot_segments) {
        std::vector<int> indices;
        for (int i = 0; i < num_robot_segments + 1; i++) { // N+1 FABRIK segments
            indices.push_back(i);
        }
        return indices;
    }
    
    // Calculate H→G distance from prismatic length
    inline double calculate_h_to_g_distance(double prismatic_length) {
        return MIN_HEIGHT + 2.0 * MOTOR_LIMIT + prismatic_length;
    }
    
    // Convert physical segment to FABRIK segment lengths
    inline std::vector<double> physical_to_fabrik_lengths(const std::vector<double>& h_to_g_distances) {
        std::vector<double> fabrik_lengths;
        int num_segments = static_cast<int>(h_to_g_distances.size());
        
        if (num_segments == 0) return fabrik_lengths;
        
        // First FABRIK segment: [0] → [1]
        fabrik_lengths.push_back(WORKING_HEIGHT + h_to_g_distances[0] / 2.0);
        
        // Middle FABRIK segments: [1] → [2], [2] → [3], etc.
        for (int i = 1; i < num_segments; i++) {
            double length = h_to_g_distances[i-1] / 2.0 + 2.0 * WORKING_HEIGHT + h_to_g_distances[i] / 2.0;
            fabrik_lengths.push_back(length);
        }
        
        // Last FABRIK segment: [N] → [N+1]
        fabrik_lengths.push_back(h_to_g_distances[num_segments-1] / 2.0 + WORKING_HEIGHT);
        
        return fabrik_lengths;
    }
}

} // namespace delta

#endif // DELTA_FABRIK_FORWARD_HPP