#include "fabrik_forward.hpp"
#include <cmath>
#include <algorithm>

namespace delta {

FabrikForwardResult FabrikForward::iterate_from_base(const FabrikChain& backward_result_chain,
                                                    double tolerance,
                                                    int max_iterations) {
    
    // Step 1: Calculate new segment lengths from backward result
    std::vector<double> new_segment_lengths = calculate_new_segment_lengths(backward_result_chain);
    
    FabrikChain current_chain = backward_result_chain;
    std::vector<Vector3> iteration_history;
    
    int iteration = 0;
    Vector3 current_base = get_base_position(current_chain);
    iteration_history.push_back(current_base);
    
    // Iterate until base is at origin or max iterations
    for (iteration = 0; iteration < max_iterations; iteration++) {
        // Perform single forward iteration
        current_chain = single_forward_iteration(current_chain, new_segment_lengths);
        
        // Update base position
        current_base = get_base_position(current_chain);
        iteration_history.push_back(current_base);
        
        // Check convergence (base should be at origin)
        if (has_converged_forward(current_base, Vector3(0, 0, 0), tolerance)) {
            break;
        }
    }
    
    Vector3 final_end_effector = current_chain.joints.back().position;
    bool constraints_ok = is_base_at_origin(current_chain, tolerance);
    
    FabrikForwardResult result(current_chain, current_base, final_end_effector, constraints_ok, iteration + 1);
    result.iteration_history = iteration_history;
    result.recalculated_lengths = new_segment_lengths;
    
    return result;
}

FabrikChain FabrikForward::single_forward_iteration(const FabrikChain& chain,
                                                  const std::vector<double>& target_segment_lengths) {
    
    FabrikChain updated_chain = chain;
    
    // Fix base at origin
    updated_chain.joints[0].position = Vector3(0, 0, 0);
    
    // Work forward through the chain
    int num_joints = static_cast<int>(updated_chain.joints.size());
    
    for (int i = 1; i < num_joints; i++) {
        // Get the segment connecting joint i-1 to joint i
        int segment_index = i - 1;
        double segment_length = target_segment_lengths[segment_index];
        
        Vector3 prev_joint_pos = updated_chain.joints[i - 1].position;
        Vector3 current_joint_pos = updated_chain.joints[i].position;
        
        // Move current joint to maintain segment length from previous joint
        Vector3 unconstrained_pos = move_joint_from_base(prev_joint_pos, current_joint_pos, segment_length);
        
        // Apply joint constraints for spherical joints
        if (updated_chain.joints[i].type == JointType::SPHERICAL_120) {
            Vector3 next_joint_pos = (i < num_joints - 1) ? updated_chain.joints[i + 1].position : Vector3(0, 0, 0);
            
            Vector3 constrained_pos = apply_forward_constraint(
                unconstrained_pos, prev_joint_pos, next_joint_pos,
                updated_chain.joints[i], segment_length
            );
            
            updated_chain.joints[i].position = constrained_pos;
        } else {
            // For end-effector, just maintain segment length
            updated_chain.joints[i].position = unconstrained_pos;
        }
    }
    
    return updated_chain;
}

std::vector<double> FabrikForward::calculate_new_segment_lengths(const FabrikChain& backward_result) {
    
    // Step 1: Extract direction pairs
    std::vector<SegmentDirectionPair> direction_pairs = extract_direction_pairs(backward_result);
    
    // Step 2: Calculate segment properties
    std::vector<SegmentProperties> segment_properties = calculate_segment_properties(direction_pairs);
    
    // Step 3: Convert to FABRIK segment lengths
    std::vector<double> fabrik_lengths = convert_to_fabrik_lengths(segment_properties, backward_result.num_robot_segments);
    
    return fabrik_lengths;
}

Vector3 FabrikForward::get_base_position(const FabrikChain& chain) {
    if (chain.joints.empty()) return Vector3(0, 0, 0);
    return chain.joints[0].position;
}

bool FabrikForward::is_base_at_origin(const FabrikChain& chain, double tolerance) {
    Vector3 base_pos = get_base_position(chain);
    return base_pos.norm() <= tolerance;
}

// Private methods

std::vector<SegmentDirectionPair> FabrikForward::extract_direction_pairs(const FabrikChain& chain) {
    std::vector<SegmentDirectionPair> pairs;
    
    int num_joints = static_cast<int>(chain.joints.size());
    int num_robot_segments = chain.num_robot_segments;
    
    // Extract direction pairs for each robot segment
    for (int seg = 0; seg < num_robot_segments; seg++) {
        // Direction pair indices
        int ref_start = seg;
        int ref_end = seg + 1;
        int target_start = seg + 1;
        int target_end = seg + 2;
        
        // Ensure indices are valid
        if (target_end >= num_joints) {
            // For last segment, use same direction as reference
            target_start = ref_start;
            target_end = ref_end;
        }
        
        Vector3 ref_dir = (chain.joints[ref_end].position - chain.joints[ref_start].position).normalized();
        Vector3 target_dir = (chain.joints[target_end].position - chain.joints[target_start].position).normalized();
        
        pairs.push_back(SegmentDirectionPair(ref_dir, target_dir, seg));
    }
    
    return pairs;
}

std::vector<SegmentProperties> FabrikForward::calculate_segment_properties(
    const std::vector<SegmentDirectionPair>& direction_pairs) {
    
    std::vector<SegmentProperties> properties;
    
    for (const auto& pair : direction_pairs) {
        // Transform target direction to Z+ reference coordinate system
        Vector3 transformed_dir = transform_to_z_reference(pair.reference_direction, pair.target_direction);
        
        // Calculate prismatic length using existing modules
        double prismatic_length = calculate_prismatic_from_direction(transformed_dir);
        
        // Calculate H→G distance
        double h_to_g_distance = fabrik_forward_utils::calculate_h_to_g_distance(prismatic_length);
        
        // For now, set fabrik_segment_length to 0 (will be calculated later)
        properties.push_back(SegmentProperties(prismatic_length, h_to_g_distance, 0.0, transformed_dir));
    }
    
    return properties;
}

Vector3 FabrikForward::transform_to_z_reference(const Vector3& reference_direction, 
                                               const Vector3& target_direction) {
    
    Vector3 ref_norm = reference_direction.normalized();
    Vector3 target_norm = target_direction.normalized();
    Vector3 z_axis(0, 0, 1);
    
    // If reference is already Z+, no transformation needed
    if ((ref_norm - z_axis).norm() < 1e-6) {
        return target_norm;
    }
    
    // If reference is -Z, simple flip
    if ((ref_norm + z_axis).norm() < 1e-6) {
        return Vector3(-target_norm.x, -target_norm.y, -target_norm.z);
    }
    
    // Calculate rotation axis (cross product of reference and Z+)
    Vector3 rotation_axis = cross_product(ref_norm, z_axis);
    double rotation_axis_length = rotation_axis.norm();
    
    if (rotation_axis_length < 1e-6) {
        // Vectors are parallel, handled above
        return target_norm;
    }
    
    rotation_axis = rotation_axis / rotation_axis_length;
    
    // Calculate rotation angle
    double cos_angle = ref_norm.dot(z_axis);
    double angle = std::acos(std::max(-1.0, std::min(1.0, cos_angle)));
    
    // Apply rotation to target direction using Rodrigues' rotation formula
    Vector3 rotated = rodrigues_rotation(target_norm, rotation_axis, angle);
    
    return rotated;
}

double FabrikForward::calculate_prismatic_from_direction(const Vector3& transformed_direction) {
    // Step 1: Calculate Fermat point from direction vector
    FermatResult fermat_result = FermatModule::calculate(transformed_direction);
    
    // Step 2: Extract prismatic length from Fermat point
    double prismatic_length = 2.0 * fermat_result.fermat_point.z;
    
    return prismatic_length;
}

std::vector<double> FabrikForward::convert_to_fabrik_lengths(
    const std::vector<SegmentProperties>& segment_properties,
    int num_robot_segments) {
    
    // Extract H→G distances
    std::vector<double> h_to_g_distances;
    for (const auto& prop : segment_properties) {
        h_to_g_distances.push_back(prop.h_to_g_distance);
    }
    
    // Convert to FABRIK segment lengths
    return fabrik_forward_utils::physical_to_fabrik_lengths(h_to_g_distances);
}

Vector3 FabrikForward::apply_forward_constraint(const Vector3& unconstrained_position,
                                              const Vector3& previous_joint_position,
                                              const Vector3& next_joint_position,
                                              const FabrikJoint& joint,
                                              double segment_length) {
    
    if (joint.type != JointType::SPHERICAL_120) {
        return unconstrained_position;
    }
    
    // For now, disable constraints similar to backward iteration
    // TODO: Implement proper forward constraints later
    return unconstrained_position;
}

Vector3 FabrikForward::move_joint_from_base(const Vector3& from_joint,
                                          const Vector3& to_joint,
                                          double required_distance) {
    
    // Calculate direction from 'from_joint' to 'to_joint'
    Vector3 direction = (to_joint - from_joint);
    double current_distance = direction.norm();
    
    if (current_distance < 1e-10) {
        // Points are the same, return arbitrary direction
        return from_joint + Vector3(required_distance, 0, 0);
    }
    
    // Normalize and scale to required distance
    direction = direction.normalized();
    return from_joint + direction * required_distance;
}

bool FabrikForward::has_converged_forward(const Vector3& current_base,
                                        const Vector3& target_base,
                                        double tolerance) {
    
    double distance = (current_base - target_base).norm();
    return distance <= tolerance;
}

Vector3 FabrikForward::cross_product(const Vector3& a, const Vector3& b) {
    return Vector3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

Vector3 FabrikForward::rodrigues_rotation(const Vector3& v, const Vector3& axis, double angle) {
    double cos_angle = std::cos(angle);
    double sin_angle = std::sin(angle);
    
    Vector3 v_parallel = axis * axis.dot(v);
    Vector3 v_perpendicular = v - v_parallel;
    Vector3 w = cross_product(axis, v_perpendicular);
    
    return v_parallel + v_perpendicular * cos_angle + w * sin_angle;
}

} // namespace delta