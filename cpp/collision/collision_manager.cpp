#include "collision_manager.hpp"
#include "../core/constants.hpp"
#include <iostream>
#include <random>

namespace delta {

void CollisionMesh::update_vertices(const float* vertices, const int* faces, int vertex_count, int face_count) {
    triangles.clear();
    triangles.reserve(face_count);
    
    // Convert vertex/face data to triangles
    for (int i = 0; i < face_count; i++) {
        int v0_idx = faces[i * 3 + 0];
        int v1_idx = faces[i * 3 + 1];
        int v2_idx = faces[i * 3 + 2];
        
        // Bounds check
        if (v0_idx >= vertex_count || v1_idx >= vertex_count || v2_idx >= vertex_count) {
            continue; // Skip invalid triangle
        }
        
        Vector3 v0(vertices[v0_idx * 3 + 0], vertices[v0_idx * 3 + 1], vertices[v0_idx * 3 + 2]);
        Vector3 v1(vertices[v1_idx * 3 + 0], vertices[v1_idx * 3 + 1], vertices[v1_idx * 3 + 2]);
        Vector3 v2(vertices[v2_idx * 3 + 0], vertices[v2_idx * 3 + 1], vertices[v2_idx * 3 + 2]);
        
        // Apply transform if needed
        if (!transform_matrix.isIdentity()) {
            Eigen::Vector4d v0_h(v0.x(), v0.y(), v0.z(), 1.0);
            Eigen::Vector4d v1_h(v1.x(), v1.y(), v1.z(), 1.0);
            Eigen::Vector4d v2_h(v2.x(), v2.y(), v2.z(), 1.0);
            
            v0 = (transform_matrix * v0_h).head<3>();
            v1 = (transform_matrix * v1_h).head<3>();
            v2 = (transform_matrix * v2_h).head<3>();
        }
        
        triangles.emplace_back(v0, v1, v2);
    }
}

CollisionManager& CollisionManager::getInstance() {
    static CollisionManager instance;
    return instance;
}

Vector3 CollisionManager::validate_joint_placement(const Vector3& proposed_position,
                                                  int joint_index,
                                                  PassType pass_type,
                                                  const FabrikChain& current_chain_state) {
    
    // Handle automatic pill management based on pass type
    if (pass_type == PassType::BACKWARD) {
        handle_backward_pill_management(joint_index, proposed_position, current_chain_state);
    } else if (pass_type == PassType::FORWARD) {
        handle_forward_pill_updates(current_chain_state);
    }
    
    // Update last pass type
    last_pass_type = pass_type;
    
    // Extract FABRIK constraints for this joint
    FabrikConstraints constraints = extract_fabrik_constraints(joint_index, current_chain_state);
    
    // Find collision-free position that respects FABRIK constraints
    Vector3 safe_position = find_collision_free_position(proposed_position, constraints, joint_index);
    
    return safe_position;
}

Vector3 CollisionManager::find_collision_free_position(const Vector3& proposed_position,
                                                      const FabrikConstraints& constraints,
                                                      int joint_index) {
    
    // If no collision meshes, return original position
    if (active_meshes.empty()) {
        return proposed_position;
    }
    
    // Check if proposed position has collision
    double collision_depth = calculate_collision_depth(proposed_position, joint_index);
    
    if (collision_depth <= 0.0) {
        // No collision, return original position
        return proposed_position;
    }
    
    std::cout << "DEBUG: Joint " << joint_index << " has collision depth " << collision_depth 
              << ", searching for collision-free position..." << std::endl;
    
    // Get collision normal for primary search direction
    Vector3 collision_normal;
    double penetration_depth;
    Vector3 next_joint_pos = (joint_index + 1 < 9) ? Vector3(0, 0, 0) : Vector3(0, 0, 0); // Simplified
    Vector3 s_point = calculate_s_point(proposed_position, next_joint_pos);
    CollisionPill temp_pill(proposed_position, s_point, COLLISION_PILL_RADIUS, joint_index);
    
    if (!check_collision_with_meshes(temp_pill, collision_normal, penetration_depth)) {
        collision_normal = Vector3(0, 0, 1); // Fallback direction
    }
    
    // Sample multiple search directions
    std::vector<Vector3> search_directions = sample_search_directions(collision_normal, 16);
    
    // Find best collision-free candidate
    Vector3 best_candidate = find_best_collision_free_candidate(
        proposed_position, constraints, search_directions, joint_index);
    
    if ((best_candidate - proposed_position).norm() > COLLISION_EPSILON) {
        std::cout << "DEBUG: Found collision-free position for joint " << joint_index 
                  << " at (" << best_candidate.x() << ", " << best_candidate.y() << ", " << best_candidate.z() << ")" << std::endl;
    } else {
        std::cout << "DEBUG: Could not find collision-free position for joint " << joint_index 
                  << ", using position with minimum collision depth" << std::endl;
    }
    
    return best_candidate;
}

bool CollisionManager::satisfies_fabrik_constraints(const Vector3& candidate_position,
                                                   const FabrikConstraints& constraints) const {
    
    // Check segment length constraint (HARD constraint)
    if (constraints.segment_length > COLLISION_EPSILON) {
        double actual_distance = (candidate_position - constraints.connected_joint_pos).norm();
        if (std::abs(actual_distance - constraints.segment_length) > COLLISION_EPSILON) {
            return false; // Segment length violated
        }
    }
    
    // Check cone constraint (HARD constraint)
    if (constraints.has_cone_constraint && constraints.cone_half_angle > COLLISION_EPSILON) {
        Vector3 direction_to_candidate = (candidate_position - constraints.connected_joint_pos).normalized();
        Vector3 cone_axis_normalized = constraints.cone_axis.normalized();
        
        double dot_product = direction_to_candidate.dot(cone_axis_normalized);
        double angle_to_axis = std::acos(std::max(-1.0, std::min(1.0, dot_product)));
        
        if (angle_to_axis > constraints.cone_half_angle) {
            return false; // Cone constraint violated
        }
    }
    
    return true; // All constraints satisfied
}

std::vector<Vector3> CollisionManager::sample_search_directions(const Vector3& collision_normal, int num_samples) const {
    std::vector<Vector3> directions;
    directions.reserve(num_samples + 1);
    
    // Primary direction: collision normal
    directions.push_back(collision_normal.normalized());
    
    // Secondary directions: around collision normal
    Vector3 normal = collision_normal.normalized();
    
    // Find two perpendicular vectors to the normal
    Vector3 perp1, perp2;
    if (std::abs(normal.z()) < 0.9) {
        perp1 = normal.cross(Vector3(0, 0, 1)).normalized();
    } else {
        perp1 = normal.cross(Vector3(1, 0, 0)).normalized();
    }
    perp2 = normal.cross(perp1).normalized();
    
    // Sample directions in a cone around the collision normal
    for (int i = 0; i < num_samples; i++) {
        double theta = 2.0 * M_PI * i / num_samples;
        double cone_angle = M_PI / 4.0; // 45 degree cone
        
        // Create direction in cone
        Vector3 direction = normal * std::cos(cone_angle) + 
                           (perp1 * std::cos(theta) + perp2 * std::sin(theta)) * std::sin(cone_angle);
        
        directions.push_back(direction.normalized());
    }
    
    return directions;
}

Vector3 CollisionManager::find_best_collision_free_candidate(const Vector3& proposed_position,
                                                           const FabrikConstraints& constraints,
                                                           const std::vector<Vector3>& search_directions,
                                                           int joint_index) const {
    
    Vector3 best_candidate = proposed_position;
    double min_collision_depth = calculate_collision_depth(proposed_position, joint_index);
    
    // Search along each direction
    for (const Vector3& direction : search_directions) {
        // Try different distances along this direction
        for (int step = 1; step <= 10; step++) {
            double distance = step * 5.0; // 5mm steps
            
            // Calculate candidate position that maintains segment length
            Vector3 candidate;
            if (constraints.segment_length > COLLISION_EPSILON) {
                // Move along direction while maintaining segment length
                Vector3 displaced_center = proposed_position + direction * distance;
                Vector3 to_connected = constraints.connected_joint_pos - displaced_center;
                candidate = constraints.connected_joint_pos - to_connected.normalized() * constraints.segment_length;
            } else {
                candidate = proposed_position + direction * distance;
            }
            
            // Check if candidate satisfies FABRIK constraints
            if (!satisfies_fabrik_constraints(candidate, constraints)) {
                continue; // Skip invalid candidates
            }
            
            // Check collision depth at candidate position
            double candidate_collision_depth = calculate_collision_depth(candidate, joint_index);
            
            if (candidate_collision_depth < min_collision_depth) {
                min_collision_depth = candidate_collision_depth;
                best_candidate = candidate;
                
                // If collision-free, return immediately
                if (min_collision_depth <= 0.0) {
                    return best_candidate;
                }
            }
        }
    }
    
    return best_candidate;
}

double CollisionManager::calculate_collision_depth(const Vector3& position, int joint_index) const {
    // Create temporary pill for collision checking
    Vector3 next_joint_pos = Vector3(0, 0, 0); // Simplified for now
    Vector3 s_point = calculate_s_point(position, next_joint_pos);
    CollisionPill temp_pill(position, s_point, COLLISION_PILL_RADIUS, joint_index);
    
    Vector3 collision_normal;
    double penetration_depth;
    
    if (check_collision_with_meshes(temp_pill, collision_normal, penetration_depth)) {
        return penetration_depth;
    }
    
    return 0.0; // No collision
}

FabrikConstraints CollisionManager::extract_fabrik_constraints(int joint_index, const FabrikChain& chain) const {
    int num_joints = static_cast<int>(chain.joints.size());
    
    if (joint_index >= num_joints) {
        return FabrikConstraints(); // Invalid index
    }
    
    // Determine connected joint and segment length
    Vector3 connected_joint_pos;
    double segment_length = 0.0;
    
    if (joint_index + 1 < num_joints) {
        // Use next joint as connected joint
        connected_joint_pos = chain.joints[joint_index + 1].position;
        if (joint_index < static_cast<int>(chain.segments.size())) {
            segment_length = chain.segments[joint_index].length;
        }
    } else if (joint_index > 0) {
        // Use previous joint as connected joint
        connected_joint_pos = chain.joints[joint_index - 1].position;
        if (joint_index - 1 < static_cast<int>(chain.segments.size())) {
            segment_length = chain.segments[joint_index - 1].length;
        }
    }
    
    // Check for cone constraints
    if (joint_index > 0 && joint_index < num_joints - 1 && 
        chain.joints[joint_index].type == JointType::SPHERICAL_120) {
        
        // Cone axis points from previous joint to next joint
        Vector3 prev_joint = chain.joints[joint_index - 1].position;
        Vector3 next_joint = chain.joints[joint_index + 1].position;
        Vector3 cone_axis = (next_joint - prev_joint).normalized();
        double cone_half_angle = SPHERICAL_JOINT_CONE_ANGLE_RAD / 2.0;
        
        return FabrikConstraints(connected_joint_pos, segment_length, cone_axis, cone_half_angle);
    }
    
    return FabrikConstraints(connected_joint_pos, segment_length);
}

void CollisionManager::update_mesh(int mesh_id, const float* vertices, const int* faces, 
                                  int vertex_count, int face_count) {
    // Create or update mesh
    if (active_meshes.find(mesh_id) == active_meshes.end()) {
        active_meshes[mesh_id] = CollisionMesh(mesh_id);
    }
    
    active_meshes[mesh_id].update_vertices(vertices, faces, vertex_count, face_count);
}

void CollisionManager::remove_mesh(int mesh_id) {
    active_meshes.erase(mesh_id);
}

void CollisionManager::clear_all_meshes() {
    active_meshes.clear();
}

void CollisionManager::set_mesh_transform(int mesh_id, const Matrix4& transform) {
    auto it = active_meshes.find(mesh_id);
    if (it != active_meshes.end()) {
        it->second.set_transform(transform);
    }
}

bool CollisionManager::check_collision_with_meshes(const CollisionPill& pill, Vector3& collision_normal, double& penetration_depth) const {
    bool has_collision = false;
    std::vector<CollisionResult> all_collisions;
    
    // Check pill against all active meshes
    for (const auto& mesh_pair : active_meshes) {
        const CollisionMesh& mesh = mesh_pair.second;
        if (mesh.is_empty()) continue;
        
        CollisionResult result = MeshCollisionDetector::check_pill_mesh_collision(pill, mesh);
        if (result.has_collision) {
            all_collisions.push_back(result);
            has_collision = true;
        }
    }
    
    if (has_collision && !all_collisions.empty()) {
        // Combine collision results
        collision_normal = MeshCollisionDetector::combine_collision_normals(all_collisions);
        
        // Find maximum penetration depth
        penetration_depth = 0.0;
        for (const CollisionResult& result : all_collisions) {
            penetration_depth = std::max(penetration_depth, result.penetration_depth);
        }
    }
    
    return has_collision;
}

bool CollisionManager::check_self_collision(const std::vector<CollisionPill>& pills, int current_pill_index) const {
    // Check current pill against non-neighboring pills
    for (int i = 0; i < static_cast<int>(pills.size()); i++) {
        // Skip self and neighboring pills (they can't physically collide)
        if (std::abs(i - current_pill_index) <= 1) continue;
        
        // TODO: Implement pill-pill collision detection
        // For now, assume no self-collision
    }
    
    return false;
}

void CollisionManager::handle_backward_pill_management(int joint_index, const Vector3& proposed_position, const FabrikChain& chain) {
    int num_joints = static_cast<int>(chain.joints.size());
    
    // Skip J8 (highest joint) - no collision check
    if (joint_index == num_joints - 1) {
        return;
    }
    
    // Approve J7 (J_Nmax-1) - direct approval
    if (joint_index == num_joints - 2) {
        return;
    }
    
    // For J6 and below (J_Nmax-2 and lower), create or update pills
    if (joint_index <= num_joints - 3) {
        create_or_update_pill(joint_index, proposed_position, chain);
    }
}

void CollisionManager::handle_forward_pill_updates(const FabrikChain& chain) {
    // Update all existing pills with new joint positions
    update_existing_pills(chain);
}

void CollisionManager::create_or_update_pill(int joint_index, const Vector3& proposed_position, const FabrikChain& chain) {
    int num_joints = static_cast<int>(chain.joints.size());
    
    // Calculate S-point (50% between current joint and next joint)
    Vector3 current_joint = proposed_position;
    Vector3 next_joint = chain.joints[joint_index + 1].position;
    Vector3 s_point = calculate_s_point(current_joint, next_joint);
    
    // Determine pill endpoints
    Vector3 pill_start, pill_end;
    
    if (joint_index == num_joints - 3) {
        // First pill: J8 (highest) -> S1
        pill_start = chain.joints[num_joints - 1].position; // J8
        pill_end = s_point; // S1
        
        // Create new pill or update existing first pill
        if (active_pills.empty()) {
            active_pills.emplace_back(pill_start, pill_end, COLLISION_PILL_RADIUS, joint_index);
        } else {
            active_pills[0].update_points(pill_start, pill_end);
        }
    } else if (joint_index == 0) {
        // SPECIAL CASE: Last pill goes all the way to base (0,0,0)
        pill_start = active_pills.empty() ? chain.joints[joint_index + 1].position : active_pills.back().end_point;
        pill_end = Vector3(0, 0, 0); // Force to base, not S-point
        
        // Find the pill index for this joint
        int pill_index = (num_joints - 3) - joint_index;
        
        if (pill_index < static_cast<int>(active_pills.size())) {
            // Update existing pill
            active_pills[pill_index].update_points(pill_start, pill_end);
        } else {
            // Create new pill
            active_pills.emplace_back(pill_start, pill_end, COLLISION_PILL_RADIUS, joint_index);
        }
    } else {
        // Normal pills: previous S-point -> current S-point
        pill_end = s_point;
        
        // Find the pill index for this joint
        int pill_index = (num_joints - 3) - joint_index;
        
        if (pill_index < static_cast<int>(active_pills.size())) {
            // Update existing pill
            active_pills[pill_index].end_point = pill_end;
            // Start point comes from previous pill's end point
            if (pill_index > 0) {
                active_pills[pill_index].start_point = active_pills[pill_index - 1].end_point;
            }
        } else {
            // Create new pill
            pill_start = active_pills.empty() ? chain.joints[joint_index + 2].position : active_pills.back().end_point;
            active_pills.emplace_back(pill_start, pill_end, COLLISION_PILL_RADIUS, joint_index);
        }
    }
}

void CollisionManager::update_existing_pills(const FabrikChain& chain) {
    int num_joints = static_cast<int>(chain.joints.size());
    
    // Update all pills based on current joint positions
    for (int i = 0; i < static_cast<int>(active_pills.size()); ++i) {
        CollisionPill& pill = active_pills[i];
        int joint_idx = pill.associated_joint_index;
        
        if (joint_idx == 0) {
            // SPECIAL CASE: Last pill always ends at base (0,0,0)
            pill.end_point = Vector3(0, 0, 0);
            if (i > 0) {
                pill.start_point = active_pills[i - 1].end_point;
            }
        } else if (joint_idx < num_joints - 1) {
            Vector3 current_joint = chain.joints[joint_idx].position;
            Vector3 next_joint = chain.joints[joint_idx + 1].position;
            Vector3 new_s_point = calculate_s_point(current_joint, next_joint);
            
            // Update pill endpoint
            pill.end_point = new_s_point;
            
            // Update start point for first pill
            if (joint_idx == num_joints - 3) {
                pill.start_point = chain.joints[num_joints - 1].position;
            } else if (i > 0) {
                // Chain pills together
                pill.start_point = active_pills[i - 1].end_point;
            }
        }
    }
}

const std::vector<CollisionPill>& CollisionManager::get_active_pills() const {
    return active_pills;
}

const std::unordered_map<int, CollisionMesh>& CollisionManager::get_active_meshes() const {
    return active_meshes;
}

Vector3 CollisionManager::calculate_s_point(const Vector3& joint1, const Vector3& joint2) const {
    return (joint1 + joint2) * 0.5; // 50% midpoint
}

} // namespace delta