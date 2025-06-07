#include "spline_collision_avoidance.hpp"
#include <chrono>
#include <algorithm>
#include <iostream>
#include <limits>

namespace delta {

// =============================================================================
// MAIN PUBLIC INTERFACE
// =============================================================================

CollisionAvoidanceResult SplineCollisionAvoidance::avoid_collisions(const CollisionAvoidanceInput& input) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    CollisionAvoidanceResult result;
    result.safe_control_points = input.spline_control_points;  // Start with original
    
    // Early exit if no obstacles
    if (input.obstacles.empty()) {
        result.collision_free = true;
        result.execution_time_ms = 0.0;
        return result;
    }
    
    // Step 1: Detect collisions
    std::vector<int> collision_indices = detect_collisions(
        input.spline_control_points, input.obstacles, input.spline_thickness);
    
    // Early exit if no collisions
    if (collision_indices.empty()) {
        result.collision_free = true;
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        result.execution_time_ms = duration.count() / 1000.0;
        return result;
    }
    
    // Store collision information
    result.collision_obstacle_indices = collision_indices;
    
    // Step 2: Generate solution candidates
    std::vector<SplineSolution> candidates = generate_solution_candidates(
        input.spline_control_points, input.obstacles, collision_indices, input);
    
    result.solutions_evaluated = static_cast<int>(candidates.size());
    
    // Step 3: Evaluate candidates
    evaluate_solution_candidates(candidates, input.spline_control_points, 
                               input.obstacles, input.spline_thickness);
    
    // Step 4: Select best solution
    if (!candidates.empty()) {
        SplineSolution best_solution = select_best_solution(candidates);
        
        if (best_solution.collision_free) {
            result.safe_control_points = best_solution.control_points;
            result.collision_free = true;
            result.total_adjustment = best_solution.total_movement;
        }
    }
    
    // Record timing
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    result.execution_time_ms = duration.count() / 1000.0;
    
    return result;
}

CollisionAvoidanceResult SplineCollisionAvoidance::avoid_collisions(
    const std::vector<Vector3>& spline_points,
    const std::vector<Obstacle>& obstacles,
    double spline_thickness,
    double safety_margin) {
    
    CollisionAvoidanceInput input(spline_points, obstacles, spline_thickness, safety_margin);
    return avoid_collisions(input);
}

bool SplineCollisionAvoidance::has_collision(const std::vector<Vector3>& spline_points,
                                           const std::vector<Obstacle>& obstacles,
                                           double spline_thickness) {
    
    std::vector<int> collisions = detect_collisions(spline_points, obstacles, spline_thickness);
    return !collisions.empty();
}

Vector3 SplineCollisionAvoidance::find_closest_point_on_spline(const Vector3& query_point,
                                                             const std::vector<Vector3>& spline_points) {
    
    if (spline_points.empty()) return Vector3(0, 0, 0);
    if (spline_points.size() == 1) return spline_points[0];
    
    double min_distance = std::numeric_limits<double>::max();
    Vector3 closest_point = spline_points[0];
    
    // Check distance to each spline segment
    for (size_t i = 0; i < spline_points.size() - 1; i++) {
        Vector3 seg_start = spline_points[i];
        Vector3 seg_end = spline_points[i + 1];
        
        // Find closest point on this segment
        Vector3 seg_vector = seg_end - seg_start;
        Vector3 query_vector = query_point - seg_start;
        
        double seg_length_sq = seg_vector.squaredNorm();
        if (seg_length_sq < EPSILON_MATH) {
            // Degenerate segment
            continue;
        }
        
        double t = query_vector.dot(seg_vector) / seg_length_sq;
        t = std::max(0.0, std::min(1.0, t));  // Clamp to [0,1]
        
        Vector3 point_on_segment = seg_start + t * seg_vector;
        double distance = (query_point - point_on_segment).norm();
        
        if (distance < min_distance) {
            min_distance = distance;
            closest_point = point_on_segment;
        }
    }
    
    return closest_point;
}

// =============================================================================
// COLLISION DETECTION
// =============================================================================

std::vector<int> SplineCollisionAvoidance::detect_collisions(const std::vector<Vector3>& spline_points,
                                                           const std::vector<Obstacle>& obstacles,
                                                           double spline_thickness) {
    
    std::vector<int> collision_indices;
    double spline_radius = spline_thickness / 2.0;
    
    for (size_t obs_idx = 0; obs_idx < obstacles.size(); obs_idx++) {
        const Obstacle& obstacle = obstacles[obs_idx];
        
        // Find closest point on spline to obstacle center
        Vector3 closest_point = find_closest_point_on_spline(obstacle.center, spline_points);
        double distance = (closest_point - obstacle.center).norm();
        
        // Check if collision occurs
        double required_clearance = obstacle.radius + spline_radius;
        if (distance < required_clearance) {
            collision_indices.push_back(static_cast<int>(obs_idx));
        }
    }
    
    return collision_indices;
}

// =============================================================================
// SOLUTION GENERATION
// =============================================================================

std::vector<SplineSolution> SplineCollisionAvoidance::generate_solution_candidates(
    const std::vector<Vector3>& original_spline,
    const std::vector<Obstacle>& obstacles,
    const std::vector<int>& collision_indices,
    const CollisionAvoidanceInput& input) {
    
    std::vector<SplineSolution> candidates;
    
    // For each colliding obstacle, generate bending alternatives
    for (int obs_idx : collision_indices) {
        const Obstacle& obstacle = obstacles[obs_idx];
        
        // Find impact point on spline
        Vector3 impact_point = find_closest_point_on_spline(obstacle.center, original_spline);
        
        // Calculate required displacement
        double required_displacement = calculate_required_displacement(
            impact_point, obstacle, input.spline_thickness, input.safety_margin);
        
        if (required_displacement <= 0) continue;  // No displacement needed
        
        // Generate bending directions
        std::vector<Vector3> bend_directions = generate_bending_directions(impact_point, obstacle.center);
        
        // Create solution candidate for each bending direction
        for (const Vector3& direction : bend_directions) {
            std::vector<Vector3> bent_spline = apply_smooth_bending(
                original_spline, impact_point, direction, 
                required_displacement, input.influence_radius);
            
            // Preserve endpoints (critical constraint)
            preserve_endpoints(bent_spline, original_spline);
            
            candidates.emplace_back(bent_spline);
        }
        
        // Limit total candidates for performance
        if (candidates.size() >= static_cast<size_t>(input.max_solution_candidates)) {
            break;
        }
    }
    
    return candidates;
}

std::vector<Vector3> SplineCollisionAvoidance::generate_bending_directions(const Vector3& impact_point,
                                                                         const Vector3& obstacle_center) {
    
    std::vector<Vector3> directions;
    
    // Primary direction: directly away from obstacle
    Vector3 primary_direction = (impact_point - obstacle_center).normalized();
    directions.push_back(primary_direction);
    
    // Calculate perpendicular directions
    auto [perp1, perp2] = calculate_perpendicular_directions(primary_direction);
    
    // Diagonal directions (70% primary + 30% perpendicular)
    Vector3 diagonal1 = (0.7 * primary_direction + 0.3 * perp1).normalized();
    Vector3 diagonal2 = (0.7 * primary_direction - 0.3 * perp1).normalized();
    directions.push_back(diagonal1);
    directions.push_back(diagonal2);
    
    // Pure perpendicular directions
    directions.push_back(perp1);
    directions.push_back(perp2);
    
    return directions;
}

std::vector<Vector3> SplineCollisionAvoidance::apply_smooth_bending(const std::vector<Vector3>& original_spline,
                                                                  const Vector3& impact_point,
                                                                  const Vector3& push_direction,
                                                                  double displacement_amount,
                                                                  double influence_radius) {
    
    std::vector<Vector3> bent_spline = original_spline;
    
    // Apply smooth displacement to nearby control points
    for (size_t i = 0; i < bent_spline.size(); i++) {
        double distance_to_impact = (bent_spline[i] - impact_point).norm();
        double weight = smooth_falloff_function(distance_to_impact, influence_radius);
        
        if (weight > EPSILON_MATH) {
            Vector3 displacement = weight * displacement_amount * push_direction;
            bent_spline[i] += displacement;
        }
    }
    
    return bent_spline;
}

double SplineCollisionAvoidance::calculate_required_displacement(const Vector3& closest_spline_point,
                                                               const Obstacle& obstacle,
                                                               double spline_thickness,
                                                               double safety_margin) {
    
    double current_distance = (closest_spline_point - obstacle.center).norm();
    double required_clearance = obstacle.radius + (spline_thickness / 2.0) + safety_margin;
    
    return std::max(0.0, required_clearance - current_distance);
}

// =============================================================================
// CONSTRAINT PRESERVATION
// =============================================================================

void SplineCollisionAvoidance::preserve_endpoints(std::vector<Vector3>& spline_points,
                                                 const std::vector<Vector3>& original_spline) {
    
    if (spline_points.size() != original_spline.size() || spline_points.empty()) {
        return;
    }
    
    // Fix start and end points (base and target)
    spline_points[0] = original_spline[0];  // Base position
    spline_points.back() = original_spline.back();  // Target position
}

std::vector<double> SplineCollisionAvoidance::calculate_spacing_ratios(const std::vector<Vector3>& spline_points) {
    std::vector<double> ratios;
    
    if (spline_points.size() < 2) return ratios;
    
    double total_length = calculate_total_length(spline_points);
    if (total_length < EPSILON_MATH) return ratios;
    
    for (size_t i = 0; i < spline_points.size() - 1; i++) {
        double segment_length = (spline_points[i + 1] - spline_points[i]).norm();
        ratios.push_back(segment_length / total_length);
    }
    
    return ratios;
}

double SplineCollisionAvoidance::calculate_total_length(const std::vector<Vector3>& spline_points) {
    double total = 0.0;
    
    for (size_t i = 0; i < spline_points.size() - 1; i++) {
        total += (spline_points[i + 1] - spline_points[i]).norm();
    }
    
    return total;
}

// =============================================================================
// SOLUTION EVALUATION
// =============================================================================

void SplineCollisionAvoidance::evaluate_solution_candidates(std::vector<SplineSolution>& candidates,
                                                           const std::vector<Vector3>& original_spline,
                                                           const std::vector<Obstacle>& obstacles,
                                                           double spline_thickness) {
    
    for (SplineSolution& candidate : candidates) {
        // Check collision-free status
        candidate.collision_free = !has_collision(candidate.control_points, obstacles, spline_thickness);
        
        // Only evaluate valid (collision-free) solutions
        if (candidate.collision_free) {
            candidate.total_movement = calculate_total_movement(original_spline, candidate.control_points);
            candidate.spacing_deviation = calculate_spacing_deviation(original_spline, candidate.control_points);
            candidate.length_deviation = calculate_length_deviation(original_spline, candidate.control_points);
            candidate.overall_score = compute_overall_score(candidate.total_movement, 
                                                          candidate.spacing_deviation,
                                                          candidate.length_deviation);
        }
    }
}

SplineSolution SplineCollisionAvoidance::select_best_solution(const std::vector<SplineSolution>& candidates) {
    // Create empty fallback solution
    std::vector<Vector3> empty_points;
    SplineSolution best_solution{empty_points};
    double best_score = std::numeric_limits<double>::max();
    
    for (const SplineSolution& candidate : candidates) {
        if (candidate.collision_free && candidate.overall_score < best_score) {
            best_score = candidate.overall_score;
            best_solution = candidate;
        }
    }
    
    return best_solution;
}

// =============================================================================
// SCORING CALCULATIONS
// =============================================================================

double SplineCollisionAvoidance::calculate_total_movement(const std::vector<Vector3>& original_spline,
                                                        const std::vector<Vector3>& modified_spline) {
    
    if (original_spline.size() != modified_spline.size()) return 0.0;
    
    double total = 0.0;
    for (size_t i = 0; i < original_spline.size(); i++) {
        total += (modified_spline[i] - original_spline[i]).norm();
    }
    
    return total;
}

double SplineCollisionAvoidance::calculate_spacing_deviation(const std::vector<Vector3>& original_spline,
                                                           const std::vector<Vector3>& modified_spline) {
    
    std::vector<double> original_ratios = calculate_spacing_ratios(original_spline);
    std::vector<double> modified_ratios = calculate_spacing_ratios(modified_spline);
    
    if (original_ratios.size() != modified_ratios.size()) return 0.0;
    
    double total_deviation = 0.0;
    for (size_t i = 0; i < original_ratios.size(); i++) {
        total_deviation += std::abs(modified_ratios[i] - original_ratios[i]);
    }
    
    return total_deviation;
}

double SplineCollisionAvoidance::calculate_length_deviation(const std::vector<Vector3>& original_spline,
                                                          const std::vector<Vector3>& modified_spline) {
    
    double original_length = calculate_total_length(original_spline);
    double modified_length = calculate_total_length(modified_spline);
    
    return std::abs(modified_length - original_length);
}

double SplineCollisionAvoidance::compute_overall_score(double total_movement,
                                                     double spacing_deviation,
                                                     double length_deviation) {
    
    return WEIGHT_TOTAL_MOVEMENT * total_movement + 
           WEIGHT_SPACING_DEVIATION * spacing_deviation + 
           WEIGHT_LENGTH_DEVIATION * length_deviation;
}

// =============================================================================
// MATHEMATICAL UTILITIES
// =============================================================================

double SplineCollisionAvoidance::smooth_falloff_function(double distance_to_impact, double influence_radius) {
    if (distance_to_impact > influence_radius) return 0.0;
    
    double normalized_dist = distance_to_impact / influence_radius;
    return std::exp(-normalized_dist * normalized_dist * COLLISION_GAUSSIAN_FALLOFF);
}

std::pair<Vector3, Vector3> SplineCollisionAvoidance::calculate_perpendicular_directions(const Vector3& primary_direction) {
    Vector3 up_vector(0, 0, 1);
    
    // Handle case where primary is parallel to up vector
    if (std::abs(primary_direction.dot(up_vector)) > 0.9) {
        up_vector = Vector3(1, 0, 0);
    }
    
    Vector3 perp1 = primary_direction.cross(up_vector).normalized();
    Vector3 perp2 = primary_direction.cross(perp1).normalized();
    
    return std::make_pair(perp1, perp2);
}

double SplineCollisionAvoidance::point_to_segment_distance(const Vector3& point,
                                                         const Vector3& seg_start,
                                                         const Vector3& seg_end) {
    
    Vector3 seg_vector = seg_end - seg_start;
    Vector3 point_vector = point - seg_start;
    
    double seg_length_sq = seg_vector.squaredNorm();
    if (seg_length_sq < EPSILON_MATH) {
        // Degenerate segment - distance to start point
        return (point - seg_start).norm();
    }
    
    double t = point_vector.dot(seg_vector) / seg_length_sq;
    t = std::max(0.0, std::min(1.0, t));  // Clamp to [0,1]
    
    Vector3 closest_point = seg_start + t * seg_vector;
    return (point - closest_point).norm();
}

} // namespace delta