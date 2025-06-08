#include "u_points_extractor.hpp"

namespace delta {

std::vector<Vector3> UPointsExtractor::extract_u_points(const FabrikChain& solved_chain) {
    std::vector<Vector3> joint_positions;
    for (const auto& joint : solved_chain.joints) {
        joint_positions.push_back(joint.position);
    }
    return extract_u_points_from_positions(joint_positions);
}

std::vector<Vector3> UPointsExtractor::extract_u_points_from_positions(const std::vector<Vector3>& joint_positions) {
    std::vector<Vector3> u_points;
    
    if (joint_positions.size() < 2) {
        return u_points;
    }
    
    // u1 = base joint (J0)
    u_points.push_back(joint_positions[0]);
    
    // u2 to u7 = midpoints between consecutive joints (skip first and last pairs)
    // u2 = midpoint(J1,J2), u3 = midpoint(J2,J3), ..., u7 = midpoint(J6,J7)
    for (size_t i = 1; i < joint_positions.size() - 2; ++i) {
        Vector3 midpoint = calculate_midpoint(joint_positions[i], joint_positions[i + 1]);
        u_points.push_back(midpoint);
    }
    
    // u8 = end-effector (J8)
    u_points.push_back(joint_positions.back());
    
    return u_points;
}

Vector3 UPointsExtractor::calculate_midpoint(const Vector3& point1, const Vector3& point2) {
    return (point1 + point2) / 2.0;
}

} // namespace delta