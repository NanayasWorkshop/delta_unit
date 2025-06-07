#ifndef DELTA_COLLISION_MANAGER_HPP
#define DELTA_COLLISION_MANAGER_HPP

#include "../core/math_utils.hpp"
#include "../fabrik/fabrik_initialization.hpp"
#include <vector>

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

class CollisionManager {
public:
    // Singleton access
    static CollisionManager& getInstance();
    
    // Main validation function - called for each joint placement
    Vector3 validate_joint_placement(const Vector3& proposed_position,
                                   int joint_index,
                                   PassType pass_type,
                                   const FabrikChain& current_chain_state);
    
    // Visualization access
    const std::vector<CollisionPill>& get_active_pills() const;

private:
    CollisionManager() = default;
    ~CollisionManager() = default;
    
    // Prevent copying
    CollisionManager(const CollisionManager&) = delete;
    CollisionManager& operator=(const CollisionManager&) = delete;
    
    // Storage
    std::vector<CollisionPill> active_pills;
    PassType last_pass_type = PassType::BACKWARD;
    
    // Internal pill management
    void handle_backward_pill_management(int joint_index, const Vector3& proposed_position, const FabrikChain& chain);
    void handle_forward_pill_updates(const FabrikChain& chain);
    
    // Helper methods
    Vector3 calculate_s_point(const Vector3& joint1, const Vector3& joint2) const;
    void update_existing_pills(const FabrikChain& chain);
    void create_or_update_pill(int joint_index, const Vector3& proposed_position, const FabrikChain& chain);
};

} // namespace delta

#endif // DELTA_COLLISION_MANAGER_HPP