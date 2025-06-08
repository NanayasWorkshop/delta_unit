#include "collision_detector.hpp"
#include "../core/constants.hpp"
#include <chrono>
#include <random>
#include <algorithm>
#include <limits>
#include <iostream>

namespace delta {

// Swarm avoider structures (from your original swarm_avoider.cpp)
struct SwarmObstacle {
    Eigen::Vector3d center;
    double radius;
};

struct SwarmResult {
    int generation;
    int swarm_id;
    std::vector<Eigen::Vector3d> control_points;
    std::vector<Eigen::Vector3d> spline_points;
    std::vector<Eigen::Vector3d> waypoints;
    double score;
    bool has_collision;
    double min_distance;
    std::vector<Eigen::Vector3d> collision_points;
    double computation_time;
};

// Full ConicalSwarmSplineAvoider implementation (from your swarm_avoider.cpp)
class ConicalSwarmSplineAvoider {
private:
    std::vector<SwarmObstacle> obstacles;
    std::vector<SwarmObstacle> original_obstacles;
    std::vector<Eigen::Vector3d> original_points;
    std::vector<double> original_cumulative_distances;
    std::vector<double> original_segments;
    double total_path_length;
    double spline_diameter;
    int num_points;
    
    // Swarm parameters (using constants)
    int num_swarms = SWARM_NUM_SWARMS;
    int num_generations = SWARM_NUM_GENERATIONS;
    int iterations_per_generation = SWARM_ITERATIONS_PER_GENERATION;
    double dt = SWARM_DT;
    
    // Force weights (using constants)
    double obstacle_avoidance_strength = SWARM_OBSTACLE_AVOIDANCE_STRENGTH;
    double path_following_strength = SWARM_PATH_FOLLOWING_STRENGTH;
    double neighbor_cohesion_strength = SWARM_NEIGHBOR_COHESION_STRENGTH;
    double velocity_damping = SWARM_VELOCITY_DAMPING;
    double smoothing_strength = SWARM_SMOOTHING_STRENGTH;
    
    // Pre-calculated exploration directions
    std::vector<Eigen::Vector3d> exploration_directions;
    
    // Random number generator
    mutable std::mt19937 rng;
    mutable std::uniform_real_distribution<double> uniform_dist;

public:
    ConicalSwarmSplineAvoider(const std::vector<SwarmObstacle>& obs, const std::vector<Eigen::Vector3d>& points, double diameter = DEFAULT_SPLINE_DIAMETER)
        : original_obstacles(obs), original_points(points), spline_diameter(diameter), 
          rng(std::chrono::steady_clock::now().time_since_epoch().count()), uniform_dist(-1.0, 1.0) {
        
        num_points = points.size();
        
        // Add safety buffer to obstacles
        obstacles.resize(obs.size());
        for (size_t i = 0; i < obs.size(); ++i) {
            obstacles[i].center = obs[i].center;
            obstacles[i].radius = obs[i].radius;
        }
        
        // Calculate original cumulative distances
        calculateCumulativeDistances(original_points, original_cumulative_distances);
        
        // Calculate original segment lengths
        original_segments.resize(original_points.size() - 1);
        total_path_length = 0.0;
        for (size_t i = 0; i < original_points.size() - 1; ++i) {
            double dist = (original_points[i + 1] - original_points[i]).norm();
            original_segments[i] = dist;
            total_path_length += dist;
        }
        
        // Pre-calculate exploration directions
        initializeExplorationDirections();
    }

    // Make these methods public for collision checking
    std::vector<Eigen::Vector3d> createSpline(const std::vector<Eigen::Vector3d>& points, int num_samples = 100) {
        if (points.size() < 2) return std::vector<Eigen::Vector3d>();
        
        // Simple cubic spline interpolation using Catmull-Rom
        std::vector<Eigen::Vector3d> spline_points;
        spline_points.reserve(num_samples);
        
        for (int i = 0; i < num_samples; ++i) {
            double t = static_cast<double>(i) / (num_samples - 1);
            double scaled_t = t * (points.size() - 1);
            int segment = static_cast<int>(scaled_t);
            double local_t = scaled_t - segment;
            
            if (segment >= static_cast<int>(points.size() - 1)) {
                spline_points.push_back(points.back());
                continue;
            }
            
            // Get control points for Catmull-Rom spline
            Eigen::Vector3d p0 = (segment > 0) ? points[segment - 1] : points[segment];
            Eigen::Vector3d p1 = points[segment];
            Eigen::Vector3d p2 = points[segment + 1];
            Eigen::Vector3d p3 = (segment < static_cast<int>(points.size() - 2)) ? points[segment + 2] : points[segment + 1];
            
            // Catmull-Rom interpolation
            Eigen::Vector3d interpolated = 0.5 * (
                2.0 * p1 +
                (-p0 + p2) * local_t +
                (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * local_t * local_t +
                (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * local_t * local_t * local_t
            );
            
            spline_points.push_back(interpolated);
        }
        
        return spline_points;
    }
    
    std::tuple<bool, std::vector<Eigen::Vector3d>, double> checkCollision(const std::vector<Eigen::Vector3d>& spline_points) {
        if (spline_points.empty()) {
            return std::make_tuple(true, std::vector<Eigen::Vector3d>(), -std::numeric_limits<double>::infinity());
        }
        
        double min_distance = std::numeric_limits<double>::infinity();
        bool has_collision = false;
        std::vector<Eigen::Vector3d> collision_points;
        
        for (const auto& obstacle : obstacles) {
            for (const auto& point : spline_points) {
                double distance = (point - obstacle.center).norm();
                
                if (distance < obstacle.radius) {
                    has_collision = true;
                    collision_points.push_back(point);
                }
                
                double min_dist_to_obstacle = distance - obstacle.radius;
                min_distance = std::min(min_distance, min_dist_to_obstacle);
            }
        }
        
        return std::make_tuple(has_collision, collision_points, min_distance);
    }

private:
    void initializeExplorationDirections() {
        exploration_directions = {
            Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(-1, 0, 0), Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, -1, 0),
            Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, -1), 
            Eigen::Vector3d(1, 1, 0).normalized(), Eigen::Vector3d(1, -1, 0).normalized(),
            Eigen::Vector3d(-1, 1, 0).normalized(), Eigen::Vector3d(-1, -1, 0).normalized(),
            Eigen::Vector3d(1, 0, 1).normalized(), Eigen::Vector3d(1, 0, -1).normalized(),
            Eigen::Vector3d(-1, 0, 1).normalized(), Eigen::Vector3d(-1, 0, -1).normalized(),
            Eigen::Vector3d(0, 1, 1).normalized(), Eigen::Vector3d(0, 1, -1).normalized(),
            Eigen::Vector3d(0, -1, 1).normalized(), Eigen::Vector3d(0, -1, -1).normalized()
        };
    }
    
    void calculateCumulativeDistances(const std::vector<Eigen::Vector3d>& points, std::vector<double>& cumulative_distances) {
        cumulative_distances.clear();
        cumulative_distances.push_back(0.0);
        
        for (size_t i = 0; i < points.size() - 1; ++i) {
            double distance = (points[i + 1] - points[i]).norm();
            cumulative_distances.push_back(cumulative_distances.back() + distance);
        }
    }
    
    double calculateConeRadius(int point_index) {
        if (point_index == 0 || point_index == num_points - 1) {
            return 0.0;
        }
        
        double distance_so_far = 0.0;
        for (int i = 0; i < point_index; ++i) {
            distance_so_far += original_segments[i];
        }
        
        double progress = distance_so_far / total_path_length;
        double cone_factor = 4.0 * progress * (1.0 - progress);
        
        double obstacle_proximity_factor = 1.0;
        Eigen::Vector3d point_pos = original_points[point_index];
        
        for (const auto& obstacle : obstacles) {
            double distance_to_obstacle = (point_pos - obstacle.center).norm();
            if (distance_to_obstacle < obstacle.radius * 3.0) {
                double proximity = std::max(0.0, 1.0 - (distance_to_obstacle / (obstacle.radius * 3.0)));
                obstacle_proximity_factor = std::max(obstacle_proximity_factor, 1.0 + proximity * 2.0);
            }
        }
        
        double base_radius = 80.0;
        return base_radius * cone_factor * obstacle_proximity_factor;
    }
    
    std::vector<Eigen::Vector3d> initializeSwarm(int swarm_id, int generation_num) {
        std::vector<Eigen::Vector3d> particles = original_points;
        Eigen::Vector3d primary_direction = exploration_directions[swarm_id % exploration_directions.size()];
        Eigen::Vector3d secondary_direction = exploration_directions[(swarm_id + 4) % exploration_directions.size()];
        
        for (int i = 1; i < num_points - 1; ++i) {
            double cone_radius = calculateConeRadius(i);
            
            if (cone_radius > 0) {
                Eigen::Vector3d directional_offset = primary_direction * cone_radius * 0.6;
                Eigen::Vector3d secondary_offset = secondary_direction * cone_radius * 0.3;
                Eigen::Vector3d random_offset(uniform_dist(rng), uniform_dist(rng), uniform_dist(rng));
                random_offset *= cone_radius * 0.3;
                
                Eigen::Vector3d total_offset = directional_offset + secondary_offset + random_offset;
                double generation_scale = 1.0 + generation_num * 0.4;
                total_offset *= generation_scale;
                
                // Push away from obstacles
                for (const auto& obstacle : obstacles) {
                    Eigen::Vector3d to_particle = (particles[i] + total_offset) - obstacle.center;
                    double distance = to_particle.norm();
                    
                    if (distance < obstacle.radius * 1.5) {
                        if (distance > 1e-6) {
                            Eigen::Vector3d push_direction = to_particle / distance;
                            double push_amount = obstacle.radius * 1.5 - distance + 20.0;
                            total_offset += push_direction * push_amount;
                        }
                    }
                }
                
                particles[i] += total_offset;
            }
        }
        
        return particles;
    }
    
    Eigen::Vector3d calculateObstacleForce(const Eigen::Vector3d& position) {
        Eigen::Vector3d total_force = Eigen::Vector3d::Zero();
        
        for (const auto& obstacle : obstacles) {
            Eigen::Vector3d to_particle = position - obstacle.center;
            double distance = to_particle.norm();
            
            double influence_radius = obstacle.radius * 3.5;
            
            if (distance < influence_radius) {
                Eigen::Vector3d direction;
                if (distance < 1e-6) {
                    direction = Eigen::Vector3d(1, 0, 0);
                } else {
                    direction = to_particle / distance;
                }
                
                double force_magnitude;
                if (distance < obstacle.radius) {
                    force_magnitude = obstacle_avoidance_strength * 20.0;
                } else if (distance < obstacle.radius * 1.3) {
                    force_magnitude = obstacle_avoidance_strength * 8.0;
                } else if (distance < obstacle.radius * 2.0) {
                    force_magnitude = obstacle_avoidance_strength * 3.0;
                } else {
                    force_magnitude = obstacle_avoidance_strength / (distance - obstacle.radius + 0.1);
                }
                
                total_force += direction * force_magnitude;
            }
        }
        
        return total_force;
    }
    
    Eigen::Vector3d calculatePathFollowingForce(int particle_idx, const Eigen::Vector3d& current_position, int generation_num) {
        if (particle_idx == 0 || particle_idx == num_points - 1) {
            return Eigen::Vector3d::Zero();
        }
        
        Eigen::Vector3d original_pos = original_points[particle_idx];
        Eigen::Vector3d to_original = original_pos - current_position;
        double distance_from_original = to_original.norm();
        
        if (distance_from_original > 100.0) {
            if (distance_from_original > 0) {
                Eigen::Vector3d direction = to_original / distance_from_original;
                double force_magnitude = path_following_strength * 0.3;
                return direction * force_magnitude;
            }
        }
        
        return Eigen::Vector3d::Zero();
    }
    
    Eigen::Vector3d calculateNeighborCohesionForce(int particle_idx, const std::vector<Eigen::Vector3d>& particles) {
        if (particle_idx == 0 || particle_idx == num_points - 1) {
            return Eigen::Vector3d::Zero();
        }
        
        Eigen::Vector3d total_force = Eigen::Vector3d::Zero();
        
        // Previous neighbor
        if (particle_idx > 0) {
            Eigen::Vector3d to_prev = particles[particle_idx - 1] - particles[particle_idx];
            double current_dist = to_prev.norm();
            double target_dist = original_segments[particle_idx - 1];
            
            if (current_dist > 0) {
                double dist_error = current_dist - target_dist;
                if (std::abs(dist_error) > target_dist * 0.15) {
                    Eigen::Vector3d direction = to_prev / current_dist;
                    double force_magnitude = neighbor_cohesion_strength * (dist_error / target_dist) * 0.5;
                    total_force += direction * force_magnitude;
                }
            }
        }
        
        // Next neighbor
        if (particle_idx < num_points - 1) {
            Eigen::Vector3d to_next = particles[particle_idx + 1] - particles[particle_idx];
            double current_dist = to_next.norm();
            double target_dist = original_segments[particle_idx];
            
            if (current_dist > 0) {
                double dist_error = current_dist - target_dist;
                if (std::abs(dist_error) > target_dist * 0.15) {
                    Eigen::Vector3d direction = to_next / current_dist;
                    double force_magnitude = neighbor_cohesion_strength * (dist_error / target_dist) * 0.5;
                    total_force += direction * force_magnitude;
                }
            }
        }
        
        return total_force;
    }
    
    Eigen::Vector3d calculateSmoothingForce(int particle_idx, const std::vector<Eigen::Vector3d>& particles) {
        if (particle_idx <= 0 || particle_idx >= num_points - 1) {
            return Eigen::Vector3d::Zero();
        }
        
        Eigen::Vector3d prev_point = particles[particle_idx - 1];
        Eigen::Vector3d curr_point = particles[particle_idx];
        Eigen::Vector3d next_point = particles[particle_idx + 1];
        
        Eigen::Vector3d expected_position = (prev_point + next_point) / 2.0;
        Eigen::Vector3d to_smooth = expected_position - curr_point;
        
        return to_smooth * smoothing_strength * 0.3;
    }
    
    double evaluateSolution(const std::vector<Eigen::Vector3d>& particles) {
        std::vector<Eigen::Vector3d> spline_points = createSpline(particles);
        if (spline_points.empty()) {
            return std::numeric_limits<double>::infinity();
        }
        
        auto [has_collision, collision_points, min_distance] = checkCollision(spline_points);
        
        if (has_collision) {
            return 10000.0 + std::abs(min_distance) * 200.0;
        }
        
        double score = 0.0;
        
        // Smoothness evaluation
        double smoothness_penalty = 0.0;
        for (int i = 1; i < num_points - 1; ++i) {
            Eigen::Vector3d prev_vec = particles[i] - particles[i - 1];
            Eigen::Vector3d next_vec = particles[i + 1] - particles[i];
            
            double prev_norm = prev_vec.norm();
            double next_norm = next_vec.norm();
            
            if (prev_norm > 0 && next_norm > 0) {
                Eigen::Vector3d prev_vec_norm = prev_vec / prev_norm;
                Eigen::Vector3d next_vec_norm = next_vec / next_norm;
                
                double dot_product = prev_vec_norm.dot(next_vec_norm);
                dot_product = std::max(-1.0, std::min(1.0, dot_product));
                double angle = std::acos(dot_product);
                smoothness_penalty += angle;
            }
        }
        
        // Path length penalty
        double length_penalty = 0.0;
        if (!spline_points.empty()) {
            double path_length = 0.0;
            for (size_t i = 0; i < spline_points.size() - 1; ++i) {
                path_length += (spline_points[i + 1] - spline_points[i]).norm();
            }
            
            std::vector<Eigen::Vector3d> original_spline = createSpline(original_points);
            if (!original_spline.empty()) {
                double original_length = 0.0;
                for (size_t i = 0; i < original_spline.size() - 1; ++i) {
                    original_length += (original_spline[i + 1] - original_spline[i]).norm();
                }
                
                double length_ratio = path_length / (original_length + 1e-6);
                length_penalty = std::abs(length_ratio - 1.0) * 10.0;
            }
        }
        
        // Efficiency bonus
        double efficiency_bonus = 0.0;
        if (!spline_points.empty()) {
            std::vector<double> min_distances;
            for (const auto& point : spline_points) {
                double min_dist = std::numeric_limits<double>::infinity();
                for (const auto& obstacle : obstacles) {
                    double dist = (point - obstacle.center).norm() - obstacle.radius;
                    min_dist = std::min(min_dist, dist);
                }
                min_distances.push_back(std::max(0.0, min_dist));
            }
            
            double avg_distance = 0.0;
            for (double d : min_distances) avg_distance += d;
            avg_distance /= min_distances.size();
            
            double target_distance = 25.0;
            if (avg_distance > target_distance) {
                efficiency_bonus = (avg_distance - target_distance) / 10.0;
            }
        }
        
        score = smoothness_penalty * 2.0 + length_penalty + efficiency_bonus;
        return score;
    }
    
    std::pair<std::vector<Eigen::Vector3d>, double> runSwarm(int swarm_id, int generation_num, const std::vector<Eigen::Vector3d>& initial_particles) {
        std::vector<Eigen::Vector3d> particles = initial_particles;
        std::vector<Eigen::Vector3d> velocities(num_points, Eigen::Vector3d::Zero());
        
        std::vector<Eigen::Vector3d> best_particles = particles;
        double best_score = evaluateSolution(particles);
        
        for (int iteration = 0; iteration < iterations_per_generation; ++iteration) {
            std::vector<Eigen::Vector3d> forces(num_points, Eigen::Vector3d::Zero());
            
            // Calculate forces
            for (int i = 0; i < num_points; ++i) {
                if (i == 0 || i == num_points - 1) continue;
                
                Eigen::Vector3d obstacle_force = calculateObstacleForce(particles[i]);
                Eigen::Vector3d path_force = calculatePathFollowingForce(i, particles[i], generation_num);
                Eigen::Vector3d cohesion_force = calculateNeighborCohesionForce(i, particles);
                Eigen::Vector3d smoothing_force = calculateSmoothingForce(i, particles);
                
                Eigen::Vector3d exploration_noise(uniform_dist(rng), uniform_dist(rng), uniform_dist(rng));
                exploration_noise *= 1.5 * (1.0 - static_cast<double>(iteration) / iterations_per_generation);
                
                Eigen::Vector3d total_force = obstacle_force + 
                                             path_force * 0.2 + 
                                             cohesion_force * 0.6 + 
                                             smoothing_force * 0.4 + 
                                             exploration_noise * 0.3;
                
                forces[i] = total_force;
            }
            
            // Update positions
            for (int i = 1; i < num_points - 1; ++i) {
                velocities[i] = velocities[i] * velocity_damping + forces[i] * dt;
                
                double max_velocity = 4.0;
                if (velocities[i].norm() > max_velocity) {
                    velocities[i] = velocities[i] / velocities[i].norm() * max_velocity;
                }
                
                particles[i] += velocities[i] * dt;
            }
            
            double current_score = evaluateSolution(particles);
            if (current_score < best_score) {
                best_score = current_score;
                best_particles = particles;
            }
        }
        
        return std::make_pair(best_particles, best_score);
    }
    
    std::vector<Eigen::Vector3d> sampleSplineAtDistances(const std::vector<Eigen::Vector3d>& spline_points, const std::vector<double>& target_distances) {
        if (spline_points.empty() || spline_points.size() < 2) {
            return std::vector<Eigen::Vector3d>();
        }
        
        // Calculate cumulative distances along the spline
        std::vector<double> spline_cumulative = {0.0};
        for (size_t i = 0; i < spline_points.size() - 1; ++i) {
            double distance = (spline_points[i + 1] - spline_points[i]).norm();
            spline_cumulative.push_back(spline_cumulative.back() + distance);
        }
        
        double total_spline_length = spline_cumulative.back();
        std::vector<Eigen::Vector3d> waypoints;
        
        for (double target_distance : target_distances) {
            if (target_distance <= 0) {
                waypoints.push_back(spline_points[0]);
            } else if (target_distance >= total_spline_length) {
                waypoints.push_back(spline_points.back());
            } else {
                for (size_t i = 0; i < spline_cumulative.size() - 1; ++i) {
                    if (spline_cumulative[i] <= target_distance && target_distance <= spline_cumulative[i + 1]) {
                        double segment_length = spline_cumulative[i + 1] - spline_cumulative[i];
                        if (segment_length > 0) {
                            double t = (target_distance - spline_cumulative[i]) / segment_length;
                            Eigen::Vector3d interpolated_point = (1.0 - t) * spline_points[i] + t * spline_points[i + 1];
                            waypoints.push_back(interpolated_point);
                        } else {
                            waypoints.push_back(spline_points[i]);
                        }
                        break;
                    }
                }
            }
        }
        
        return waypoints;
    }

public:
    SwarmResult solve(bool verbose = false) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        if (verbose) {
            std::cout << "Starting conical swarm optimization..." << std::endl;
        }
        
        SwarmResult overall_best_result;
        overall_best_result.score = std::numeric_limits<double>::infinity();
        
        for (int generation = 0; generation < num_generations; ++generation) {
            if (verbose) {
                std::cout << "Generation " << generation << std::endl;
            }
            
            for (int swarm_id = 0; swarm_id < num_swarms; ++swarm_id) {
                std::vector<Eigen::Vector3d> initial_particles = initializeSwarm(swarm_id, generation);
                auto [best_particles, best_score] = runSwarm(swarm_id, generation, initial_particles);
                
                std::vector<Eigen::Vector3d> spline_points = createSpline(best_particles);
                auto [has_collision, collision_points, min_distance] = checkCollision(spline_points);
                
                if (best_score < overall_best_result.score) {
                    overall_best_result.generation = generation;
                    overall_best_result.swarm_id = swarm_id;
                    overall_best_result.control_points = best_particles;
                    overall_best_result.spline_points = spline_points;
                    overall_best_result.score = best_score;
                    overall_best_result.has_collision = has_collision;
                    overall_best_result.min_distance = min_distance;
                    overall_best_result.collision_points = collision_points;
                }
            }
        }
        
        // Generate waypoints for best result
        if (!overall_best_result.spline_points.empty()) {
            overall_best_result.waypoints = sampleSplineAtDistances(
                overall_best_result.spline_points, 
                original_cumulative_distances
            );
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        overall_best_result.computation_time = duration.count() / 1000.0;
        
        if (verbose) {
            std::cout << "Total computation time: " << overall_best_result.computation_time << " seconds" << std::endl;
        }
        
        return overall_best_result;
    }
};

// Main collision detector implementation with early return optimization
CollisionResult CollisionDetector::check_and_avoid(const std::vector<Vector3>& u_points, 
                                                  const std::vector<Obstacle>& obstacles,
                                                  double spline_diameter) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Convert U points to swarm format
    std::vector<Eigen::Vector3d> swarm_points = convert_to_swarm_format(u_points);
    
    // Convert obstacles to swarm format
    std::vector<SwarmObstacle> swarm_obstacles;
    for (const auto& obs : obstacles) {
        SwarmObstacle swarm_obs;
        swarm_obs.center = Eigen::Vector3d(obs.center.x(), obs.center.y(), obs.center.z());
        swarm_obs.radius = obs.radius;
        swarm_obstacles.push_back(swarm_obs);
    }
    
    // Create avoider for collision checking
    ConicalSwarmSplineAvoider avoider(swarm_obstacles, swarm_points, spline_diameter);
    
    // OPTIMIZATION: Quick collision check first - don't run swarm if no collision
    std::vector<Eigen::Vector3d> test_spline = avoider.createSpline(swarm_points);
    auto [has_collision, collision_points_eigen, min_distance] = avoider.checkCollision(test_spline);
    
    std::cout << "Initial collision check: " << (has_collision ? "COLLISION DETECTED" : "NO COLLISION") 
              << ", min_distance: " << min_distance << std::endl;
    
    std::vector<Vector3> result_waypoints;
    std::vector<Vector3> result_collision_points;
    
    if (!has_collision) {
        // No collision detected - return original U points as waypoints (no swarm optimization needed)
        std::cout << "No collision - returning original U points (no swarm optimization)" << std::endl;
        
        for (const auto& point : swarm_points) {
            result_waypoints.push_back(Vector3(point.x(), point.y(), point.z()));
        }
        
        // No collision points
        result_collision_points.clear();
        
    } else {
        // Collision detected - run expensive swarm optimization
        std::cout << "Collision detected - running swarm optimization..." << std::endl;
        
        SwarmResult swarm_result = avoider.solve(false);  // Not verbose
        
        // Convert swarm result back to our format
        for (const auto& wp : swarm_result.waypoints) {
            result_waypoints.push_back(Vector3(wp.x(), wp.y(), wp.z()));
        }
        
        for (const auto& cp : swarm_result.collision_points) {
            result_collision_points.push_back(Vector3(cp.x(), cp.y(), cp.z()));
        }
        
        // Update collision status from swarm result
        has_collision = swarm_result.has_collision;
        min_distance = swarm_result.min_distance;
        
        std::cout << "Swarm optimization completed: " << (has_collision ? "STILL COLLISION" : "COLLISION AVOIDED") 
                  << ", final min_distance: " << min_distance << std::endl;
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    CollisionResult result(has_collision, result_waypoints, min_distance);
    result.collision_points = result_collision_points;
    result.computation_time = duration.count() / 1000.0;  // Convert to milliseconds
    
    return result;
}

std::vector<Obstacle> CollisionDetector::create_test_obstacles() {
    std::vector<Obstacle> obstacles;
    
    // Create test obstacles (same as in your original example)
    obstacles.push_back(Obstacle(Vector3(123, 28, 500), 60));   
    obstacles.push_back(Obstacle(Vector3(-30, -30, 370), 45));  
    obstacles.push_back(Obstacle(Vector3(50, 0, 200), 30));     
    
    return obstacles;
}

std::vector<Eigen::Vector3d> CollisionDetector::convert_to_swarm_format(const std::vector<Vector3>& u_points) {
    std::vector<Eigen::Vector3d> swarm_points;
    for (const auto& point : u_points) {
        swarm_points.push_back(Eigen::Vector3d(point.x(), point.y(), point.z()));
    }
    return swarm_points;
}

std::vector<Eigen::Vector3d> CollisionDetector::convert_waypoints_from_swarm(const std::vector<Eigen::Vector3d>& swarm_waypoints) {
    return swarm_waypoints;  // Already in correct format
}

} // namespace delta