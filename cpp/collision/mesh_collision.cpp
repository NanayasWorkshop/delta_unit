#include "mesh_collision.hpp"
#include "collision_manager.hpp"
#include <algorithm>
#include <cmath>

namespace delta {

Triangle::Triangle(const Vector3& vertex0, const Vector3& vertex1, const Vector3& vertex2)
    : v0(vertex0), v1(vertex1), v2(vertex2) {
    compute_normal_and_area();
}

void Triangle::compute_normal_and_area() {
    Vector3 edge1 = v1 - v0;
    Vector3 edge2 = v2 - v0;
    Vector3 cross = edge1.cross(edge2);
    area = cross.norm() * 0.5;
    
    if (area > COLLISION_EPSILON) {
        normal = cross.normalized();
    } else {
        normal = Vector3(0, 0, 1); // Fallback for degenerate triangles
    }
}

CollisionResult MeshCollisionDetector::check_pill_mesh_collision(const CollisionPill& pill, const CollisionMesh& mesh) {
    std::vector<CollisionResult> all_collisions;
    
    // Check pill (capsule) against all triangles in mesh
    for (const Triangle& triangle : mesh.triangles) {
        CollisionResult result = capsule_triangle_collision(
            pill.start_point, pill.end_point, pill.radius, triangle);
        
        if (result.has_collision) {
            all_collisions.push_back(result);
        }
    }
    
    // If no collisions, return empty result
    if (all_collisions.empty()) {
        return CollisionResult();
    }
    
    // Combine multiple collisions into single result
    Vector3 combined_normal = combine_collision_normals(all_collisions);
    
    // Find deepest penetration
    double max_depth = 0.0;
    Vector3 deepest_point = all_collisions[0].collision_point;
    
    for (const CollisionResult& collision : all_collisions) {
        if (collision.penetration_depth > max_depth) {
            max_depth = collision.penetration_depth;
            deepest_point = collision.collision_point;
        }
    }
    
    return CollisionResult(deepest_point, combined_normal, max_depth);
}

CollisionResult MeshCollisionDetector::sphere_triangle_collision(const Vector3& sphere_center, double sphere_radius, const Triangle& triangle) {
    // Find closest point on triangle to sphere center
    Vector3 closest_point = closest_point_on_triangle(sphere_center, triangle);
    
    // Calculate distance from sphere center to closest point
    Vector3 to_sphere = sphere_center - closest_point;
    double distance = to_sphere.norm();
    
    // Check if collision occurs
    if (distance > sphere_radius) {
        return CollisionResult(); // No collision
    }
    
    // Calculate collision normal
    Vector3 collision_normal;
    if (distance > COLLISION_EPSILON) {
        collision_normal = to_sphere.normalized();
    } else {
        // Sphere center is on triangle, use triangle normal
        collision_normal = triangle.normal;
    }
    
    double penetration_depth = sphere_radius - distance;
    
    return CollisionResult(closest_point, collision_normal, penetration_depth);
}

CollisionResult MeshCollisionDetector::capsule_triangle_collision(const Vector3& capsule_start, const Vector3& capsule_end, 
                                                                double capsule_radius, const Triangle& triangle) {
    // Find closest point on triangle to the capsule line segment
    Vector3 closest_on_triangle = closest_point_on_triangle(capsule_start, triangle);
    Vector3 closest_on_capsule = closest_point_on_line_segment(closest_on_triangle, capsule_start, capsule_end);
    
    // Check collision between the closest points
    return sphere_triangle_collision(closest_on_capsule, capsule_radius, triangle);
}

Vector3 MeshCollisionDetector::resolve_collision(const Vector3& original_position, const CollisionResult& collision) {
    if (!collision.has_collision) {
        return original_position;
    }
    
    // Move along collision normal by penetration depth
    return original_position + collision.collision_normal * collision.penetration_depth;
}

Vector3 MeshCollisionDetector::combine_collision_normals(const std::vector<CollisionResult>& collisions) {
    if (collisions.empty()) {
        return Vector3(0, 0, 1); // Default up direction
    }
    
    Vector3 combined_normal(0, 0, 0);
    double total_weight = 0.0;
    
    // Area-weighted average of collision normals
    for (const CollisionResult& collision : collisions) {
        double weight = collision.penetration_depth; // Weight by penetration depth
        combined_normal += collision.collision_normal * weight;
        total_weight += weight;
    }
    
    if (total_weight > COLLISION_EPSILON) {
        combined_normal = combined_normal / total_weight;
        return combined_normal.normalized();
    }
    
    return Vector3(0, 0, 1); // Fallback
}

Vector3 MeshCollisionDetector::closest_point_on_triangle(const Vector3& point, const Triangle& triangle) {
    // Project point onto triangle plane
    Vector3 to_point = point - triangle.v0;
    double dist_to_plane = to_point.dot(triangle.normal);
    Vector3 projected = point - triangle.normal * dist_to_plane;
    
    // Check if projected point is inside triangle
    if (point_in_triangle(projected, triangle)) {
        return projected;
    }
    
    // Find closest point on triangle edges
    Vector3 closest1 = closest_point_on_line_segment(point, triangle.v0, triangle.v1);
    Vector3 closest2 = closest_point_on_line_segment(point, triangle.v1, triangle.v2);
    Vector3 closest3 = closest_point_on_line_segment(point, triangle.v2, triangle.v0);
    
    // Return the closest of the three edge points
    double dist1 = (point - closest1).norm();
    double dist2 = (point - closest2).norm();
    double dist3 = (point - closest3).norm();
    
    if (dist1 <= dist2 && dist1 <= dist3) return closest1;
    if (dist2 <= dist3) return closest2;
    return closest3;
}

Vector3 MeshCollisionDetector::closest_point_on_line_segment(const Vector3& point, const Vector3& line_start, const Vector3& line_end) {
    Vector3 line_vec = line_end - line_start;
    double line_length_sq = line_vec.dot(line_vec);
    
    if (line_length_sq < COLLISION_EPSILON) {
        return line_start; // Degenerate line segment
    }
    
    Vector3 to_point = point - line_start;
    double t = to_point.dot(line_vec) / line_length_sq;
    
    // Clamp t to [0, 1] to stay on line segment
    t = std::max(0.0, std::min(1.0, t));
    
    return line_start + line_vec * t;
}

bool MeshCollisionDetector::point_in_triangle(const Vector3& point, const Triangle& triangle) {
    // Use barycentric coordinates
    Vector3 v0v1 = triangle.v1 - triangle.v0;
    Vector3 v0v2 = triangle.v2 - triangle.v0;
    Vector3 v0p = point - triangle.v0;
    
    double dot00 = v0v2.dot(v0v2);
    double dot01 = v0v2.dot(v0v1);
    double dot02 = v0v2.dot(v0p);
    double dot11 = v0v1.dot(v0v1);
    double dot12 = v0v1.dot(v0p);
    
    double inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    double u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
    double v = (dot00 * dot12 - dot01 * dot02) * inv_denom;
    
    return (u >= 0) && (v >= 0) && (u + v <= 1);
}

} // namespace delta