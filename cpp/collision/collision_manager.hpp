#ifndef DELTA_COLLISION_MANAGER_HPP
#define DELTA_COLLISION_MANAGER_HPP

#include "../core/math_utils.hpp"
#include "../fabrik/fabrik_initialization.hpp"
#include "mesh_collision.hpp"
#include <vector>
#include <unordered_map>

namespace delta {

enum class PassType {
    BACKWARD,
    FORWARD
};

struct CollisionPill {
    Vector3 start_point;
    Vector3 end_point;
    double radius;
    int associated_joint_index;
    
    CollisionPill(const Vector3& start, const Vector3& end, double r, int joint_idx)
        : start_point(start), end_point(end), radius(r), associated_joint_index(joint_idx) {}
    
    Vector3 get_center() const {
        return (start_point + end_point) * 0.5;
    }
    
    double get_length() const {
        return (end_point - start_point).norm();
    }
    
    void update_points(const Vector3& new_start, const Vector3& new_end) {
        start_point = new_start;
        end_point = new_end;
    }
};

struct CollisionMesh {
    int mesh_id;
    std::vector<Triangle> triangles;
    Matrix4 transform_matrix;  // For mesh positioning/rotation
    
    CollisionMesh() : mesh_id(-1), transform_matrix(Matrix4::Identity()) {}  // Default constructor
    CollisionMesh(int id) : mesh_id(id), transform_matrix(Matrix4::Identity()) {}
    
    void update_vertices(const float* vertices, const int* faces, int vertex_count, int face_count);
    void set_transform(const Matrix4& transform) { transform_matrix = transform; }
    void clear() { triangles.clear(); }
    bool is_empty() const { return triangles.empty(); }
};

// FABRIK constraint information for collision-aware positioning
struct FabrikConstraints {
    Vector3 connected_joint_pos;    // Position of connected joint (for segment length)
    double segment_length;          // Required segment length (HARD constraint)
    Vector3 cone_axis;             // Cone axis direction (for spherical joints)
    double cone_half_angle;        // Maximum cone half-angle (HARD constraint)
    bool has_cone_constraint;      // Whether cone constraint applies
    
    FabrikConstraints() : segment_length(0.0), cone_half_angle(0.0), has_cone_constraint(false) {}
    
    FabrikConstraints(const Vector3& connected_pos, double seg_length)
        : connected_joint_pos(connected_pos), segment_length(seg_length), 
          cone_half_angle(0.0), has_cone_constraint(false) {}
    
    FabrikConstraints(const Vector3& connected_pos, double seg_length,
                     const Vector3& axis, double half_angle)
        : connected_joint_pos(connected_pos), segment_length(seg_length),
          cone_axis(axis), cone_half_angle(half_angle), has_cone_constraint(true) {}
};

class CollisionManager {
public:
    // Singleton access
    static CollisionManager& getInstance();
    
    // Main validation function - called for each joint placement
    Vector3 validate_joint_placement(const Vector3& proposed_position,
                                   int joint_index,
                                   PassType pass_type,
                                   const FabrikChain& current_chain_state);
    
    // Mesh management
    void update_mesh(int mesh_id, const float* vertices, const int* faces, 
                     int vertex_count, int face_count);
    void remove_mesh(int mesh_id);
    void clear_all_meshes();
    void set_mesh_transform(int mesh_id, const Matrix4& transform);
    
    // FABRIK-aware collision resolution
    Vector3 find_collision_free_position(const Vector3& proposed_position,
                                        const FabrikConstraints& constraints,
                                        int joint_index);
    
    // Constraint validation
    bool satisfies_fabrik_constraints(const Vector3& candidate_position,
                                    const FabrikConstraints& constraints) const;
    
    // Collision detection
    bool check_collision_with_meshes(const CollisionPill& pill, Vector3& collision_normal, double& penetration_depth) const;
    bool check_self_collision(const std::vector<CollisionPill>& pills, int current_pill_index) const;
    
    // Visualization access
    const std::vector<CollisionPill>& get_active_pills() const;
    const std::unordered_map<int, CollisionMesh>& get_active_meshes() const;

private:
    CollisionManager() = default;
    ~CollisionManager() = default;
    
    // Prevent copying
    CollisionManager(const CollisionManager&) = delete;
    CollisionManager& operator=(const CollisionManager&) = delete;
    
    // Storage
    std::vector<CollisionPill> active_pills;
    std::unordered_map<int, CollisionMesh> active_meshes;
    PassType last_pass_type = PassType::BACKWARD;
    
    // Internal pill management
    void handle_backward_pill_management(int joint_index, const Vector3& proposed_position, const FabrikChain& chain);
    void handle_forward_pill_updates(const FabrikChain& chain);
    
    // FABRIK constraint extraction
    FabrikConstraints extract_fabrik_constraints(int joint_index, const FabrikChain& chain) const;
    
    // Multi-direction sampling for collision resolution
    std::vector<Vector3> sample_search_directions(const Vector3& collision_normal, int num_samples = 16) const;
    Vector3 find_best_collision_free_candidate(const Vector3& proposed_position,
                                              const FabrikConstraints& constraints,
                                              const std::vector<Vector3>& search_directions,
                                              int joint_index) const;
    
    // Collision depth calculation
    double calculate_collision_depth(const Vector3& position, int joint_index) const;
    
    // Helper methods
    Vector3 calculate_s_point(const Vector3& joint1, const Vector3& joint2) const;
    void update_existing_pills(const FabrikChain& chain);
    void create_or_update_pill(int joint_index, const Vector3& proposed_position, const FabrikChain& chain);
};

} // namespace delta

#endif // DELTA_COLLISION_MANAGER_HPP