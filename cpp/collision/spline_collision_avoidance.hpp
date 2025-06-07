#ifndef DELTA_SPLINE_COLLISION_AVOIDANCE_HPP
#define DELTA_SPLINE_COLLISION_AVOIDANCE_HPP

#include "../core/math_utils.hpp"
#include "../core/constants.hpp"
#include <vector>
#include <limits>

namespace delta {

// Forward declarations
struct Obstacle;
struct CollisionAvoidanceInput;
struct CollisionAvoidanceResult;
struct SplineSolution;

// =============================================================================
// CORE STRUCTURES
// =============================================================================

/**
 * Simple obstacle representation (spherical for now)
 */
struct Obstacle {
    Vector3 center;              // Obstacle position in 3D space
    double radius;               // Collision radius
    
    Obstacle(const Vector3& pos, double r) : center(pos), radius(r) {}
    Obstacle(double x, double y, double z, double r) : center(Vector3(x, y, z)), radius(r) {}
};

/**
 * Input parameters for collision avoidance
 */
struct CollisionAvoidanceInput {
    std::vector<Vector3> spline_control_points;   // Original spline from FABRIK
    std::vector<Obstacle> obstacles;              // Scene obstacles to avoid
    double spline_thickness;                      // Spline thickness for collision detection
    double safety_margin;                         // Extra clearance distance
    
    // Optional performance tuning
    int max_solution_candidates;                  // Max alternatives to generate
    double influence_radius;                      // Bending influence distance
    
    CollisionAvoidanceInput(const std::vector<Vector3>& spline_points,
                           const std::vector<Obstacle>& obs,
                           double thickness = SPLINE_THICKNESS,
                           double margin = COLLISION_SAFETY_MARGIN)
        : spline_control_points(spline_points)
        , obstacles(obs)
        , spline_thickness(thickness)
        , safety_margin(margin)
        , max_solution_candidates(COLLISION_MAX_SOLUTION_CANDIDATES)
        , influence_radius(COLLISION_INFLUENCE_RADIUS) {}
};

/**
 * Result of collision avoidance operation
 */
struct CollisionAvoidanceResult {
    std::vector<Vector3> safe_control_points;     // Collision-free spline points
    bool collision_free;                          // True if successful
    double total_adjustment;                      // Total displacement applied
    double execution_time_ms;                     // Performance measurement
    int solutions_evaluated;                      // Number of candidates tested
    
    // Debug information
    std::vector<int> collision_obstacle_indices;  // Which obstacles caused collisions
    std::vector<Vector3> collision_points;        // Where collisions occurred
    
    CollisionAvoidanceResult()
        : collision_free(false)
        , total_adjustment(0.0)
        , execution_time_ms(0.0)
        , solutions_evaluated(0) {}
};

/**
 * Internal structure for solution candidates
 */
struct SplineSolution {
    std::vector<Vector3> control_points;          // Modified spline points
    double total_movement;                        // Sum of control point displacements
    double spacing_deviation;                     // Change in relative spacing ratios
    double length_deviation;                      // Change in total spline length
    bool collision_free;                          // Must be true for valid solution
    double overall_score;                         // Weighted combination score
    
    SplineSolution(const std::vector<Vector3>& points)
        : control_points(points)
        , total_movement(0.0)
        , spacing_deviation(0.0)
        , length_deviation(0.0)
        , collision_free(false)
        , overall_score(std::numeric_limits<double>::max()) {}
};

// =============================================================================
// MAIN COLLISION AVOIDANCE CLASS
// =============================================================================

class SplineCollisionAvoidance {
public:
    /**
     * Main entry point: Avoid collisions for a spline
     */
    static CollisionAvoidanceResult avoid_collisions(const CollisionAvoidanceInput& input);
    
    /**
     * Convenience method using default parameters
     */
    static CollisionAvoidanceResult avoid_collisions(
        const std::vector<Vector3>& spline_points,
        const std::vector<Obstacle>& obstacles,
        double spline_thickness = SPLINE_THICKNESS,
        double safety_margin = COLLISION_SAFETY_MARGIN
    );
    
    /**
     * Check if spline has any collisions (fast check)
     */
    static bool has_collision(const std::vector<Vector3>& spline_points,
                             const std::vector<Obstacle>& obstacles,
                             double spline_thickness);
    
    /**
     * Find closest point on spline to given position
     */
    static Vector3 find_closest_point_on_spline(const Vector3& query_point,
                                               const std::vector<Vector3>& spline_points);

private:
    // =============================================================================
    // CORE ALGORITHM STEPS
    // =============================================================================
    
    /**
     * Step 1: Detect all collisions between spline and obstacles
     */
    static std::vector<int> detect_collisions(const std::vector<Vector3>& spline_points,
                                             const std::vector<Obstacle>& obstacles,
                                             double spline_thickness);
    
    /**
     * Step 2: Generate multiple bending solution candidates
     */
    static std::vector<SplineSolution> generate_solution_candidates(
        const std::vector<Vector3>& original_spline,
        const std::vector<Obstacle>& obstacles,
        const std::vector<int>& collision_indices,
        const CollisionAvoidanceInput& input
    );
    
    /**
     * Step 3: Evaluate and score each solution candidate
     */
    static void evaluate_solution_candidates(std::vector<SplineSolution>& candidates,
                                           const std::vector<Vector3>& original_spline,
                                           const std::vector<Obstacle>& obstacles,
                                           double spline_thickness);
    
    /**
     * Step 4: Select best solution based on scoring criteria
     */
    static SplineSolution select_best_solution(const std::vector<SplineSolution>& candidates);
    
    // =============================================================================
    // SOLUTION GENERATION HELPERS
    // =============================================================================
    
    /**
     * Generate bending directions for a single obstacle
     */
    static std::vector<Vector3> generate_bending_directions(const Vector3& impact_point,
                                                           const Vector3& obstacle_center);
    
    /**
     * Apply smooth bending to spline around impact point
     */
    static std::vector<Vector3> apply_smooth_bending(const std::vector<Vector3>& original_spline,
                                                   const Vector3& impact_point,
                                                   const Vector3& push_direction,
                                                   double displacement_amount,
                                                   double influence_radius);
    
    /**
     * Calculate required clearance displacement for obstacle
     */
    static double calculate_required_displacement(const Vector3& closest_spline_point,
                                                const Obstacle& obstacle,
                                                double spline_thickness,
                                                double safety_margin);
    
    // =============================================================================
    // CONSTRAINT PRESERVATION
    // =============================================================================
    
    /**
     * Preserve fixed endpoints (base and target positions)
     */
    static void preserve_endpoints(std::vector<Vector3>& spline_points,
                                 const std::vector<Vector3>& original_spline);
    
    /**
     * Calculate relative spacing ratios between control points
     */
    static std::vector<double> calculate_spacing_ratios(const std::vector<Vector3>& spline_points);
    
    /**
     * Calculate total spline length
     */
    static double calculate_total_length(const std::vector<Vector3>& spline_points);
    
    // =============================================================================
    // SCORING AND EVALUATION
    // =============================================================================
    
    /**
     * Calculate total movement of control points
     */
    static double calculate_total_movement(const std::vector<Vector3>& original_spline,
                                         const std::vector<Vector3>& modified_spline);
    
    /**
     * Calculate deviation in spacing ratios
     */
    static double calculate_spacing_deviation(const std::vector<Vector3>& original_spline,
                                            const std::vector<Vector3>& modified_spline);
    
    /**
     * Calculate deviation in total length
     */
    static double calculate_length_deviation(const std::vector<Vector3>& original_spline,
                                           const std::vector<Vector3>& modified_spline);
    
    /**
     * Compute weighted overall score for solution
     */
    static double compute_overall_score(double total_movement,
                                      double spacing_deviation,
                                      double length_deviation);
    
    // =============================================================================
    // MATHEMATICAL UTILITIES
    // =============================================================================
    
    /**
     * Smooth Gaussian falloff function for bending influence
     */
    static double smooth_falloff_function(double distance_to_impact,
                                        double influence_radius = COLLISION_INFLUENCE_RADIUS);
    
    /**
     * Calculate perpendicular directions to a primary direction
     */
    static std::pair<Vector3, Vector3> calculate_perpendicular_directions(const Vector3& primary_direction);
    
    /**
     * Distance from point to line segment
     */
    static double point_to_segment_distance(const Vector3& point,
                                          const Vector3& seg_start,
                                          const Vector3& seg_end);
};

} // namespace delta

#endif // DELTA_SPLINE_COLLISION_AVOIDANCE_HPP