#ifndef DELTA_MESH_COLLISION_HPP
#define DELTA_MESH_COLLISION_HPP

#include "../core/math_utils.hpp"
#include "../core/constants.hpp"
#include <vector>

namespace delta {

// Forward declarations
struct CollisionPill;
struct CollisionMesh;

// Triangle representation for collision detection
struct Triangle {
    Vector3 v0, v1, v2;    // Triangle vertices
    Vector3 normal;        // Pre-computed normal
    double area;           // Pre-computed area
    
    Triangle(const Vector3& vertex0, const Vector3& vertex1, const Vector3& vertex2);
    void compute_normal_and_area();
};

// Collision detection result
struct CollisionResult {
    bool has_collision;
    Vector3 collision_point;
    Vector3 collision_normal;
    double penetration_depth;
    
    CollisionResult() : has_collision(false), penetration_depth(0.0) {}
    CollisionResult(const Vector3& point, const Vector3& normal, double depth)
        : has_collision(true), collision_point(point), collision_normal(normal), penetration_depth(depth) {}
};

// Mesh-specific collision detection utilities
class MeshCollisionDetector {
public:
    // Main collision detection methods
    static CollisionResult check_pill_mesh_collision(const CollisionPill& pill, const CollisionMesh& mesh);
    
    // Primitive collision tests
    static CollisionResult sphere_triangle_collision(const Vector3& sphere_center, double sphere_radius, const Triangle& triangle);
    static CollisionResult capsule_triangle_collision(const Vector3& capsule_start, const Vector3& capsule_end, 
                                                     double capsule_radius, const Triangle& triangle);
    
    // Collision resolution
    static Vector3 resolve_collision(const Vector3& original_position, const CollisionResult& collision);
    static Vector3 combine_collision_normals(const std::vector<CollisionResult>& collisions);
    
private:
    // Helper methods
    static Vector3 closest_point_on_triangle(const Vector3& point, const Triangle& triangle);
    static Vector3 closest_point_on_line_segment(const Vector3& point, const Vector3& line_start, const Vector3& line_end);
    static double distance_point_to_triangle(const Vector3& point, const Triangle& triangle);
    static bool point_in_triangle(const Vector3& point, const Triangle& triangle);
};

} // namespace delta

#endif // DELTA_MESH_COLLISION_HPP