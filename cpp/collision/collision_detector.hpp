#ifndef DELTA_COLLISION_DETECTOR_HPP
#define DELTA_COLLISION_DETECTOR_HPP

#include "../core/math_utils.hpp"
#include <vector>

namespace delta {

// Obstacle for collision detection (matches swarm avoider format)
struct Obstacle {
    Vector3 center;
    double radius;
    
    Obstacle(const Vector3& c, double r) : center(c), radius(r) {}
};

// Result from collision detection and avoidance
struct CollisionResult {
    bool has_collision;                     // Whether collision was detected
    std::vector<Vector3> waypoints;         // Collision-free waypoints
    double min_distance;                    // Minimum distance to obstacles
    std::vector<Vector3> collision_points;  // Points where collision occurred
    double computation_time;                // Time taken for avoidance
    
    CollisionResult(bool collision, const std::vector<Vector3>& wp, double min_dist)
        : has_collision(collision), waypoints(wp), min_distance(min_dist), computation_time(0.0) {}
};

// Bridge between U points and ConicalSwarmSplineAvoider
class CollisionDetector {
public:
    // Main interface: check collision and get avoidance waypoints
    static CollisionResult check_and_avoid(const std::vector<Vector3>& u_points, 
                                          const std::vector<Obstacle>& obstacles,
                                          double spline_diameter = 20.0);
    
    // Create dummy test obstacles for testing
    static std::vector<Obstacle> create_test_obstacles();

private:
    // Convert Eigen Vector3 to Vector3d for swarm avoider
    static std::vector<Eigen::Vector3d> convert_to_swarm_format(const std::vector<Vector3>& u_points);
    
    // Convert swarm obstacles to internal format
    static std::vector<Eigen::Vector3d> convert_waypoints_from_swarm(const std::vector<Eigen::Vector3d>& swarm_waypoints);
};

} // namespace delta

#endif // DELTA_COLLISION_DETECTOR_HPP