#ifndef DELTA_U_POINTS_EXTRACTOR_HPP
#define DELTA_U_POINTS_EXTRACTOR_HPP

#include "../core/math_utils.hpp"
#include "../fabrik/fabrik_initialization.hpp"
#include <vector>

namespace delta {

// Extract simple collision detection points from FABRIK chains
// U Points: u1=base, u2=midpoint(J1,J2), ..., u_last=end-effector
class UPointsExtractor {
public:
    // Extract U points from solved FABRIK chain
    static std::vector<Vector3> extract_u_points(const FabrikChain& solved_chain);
    
    // Extract U points from joint positions
    static std::vector<Vector3> extract_u_points_from_positions(const std::vector<Vector3>& joint_positions);

private:
    static Vector3 calculate_midpoint(const Vector3& point1, const Vector3& point2);
};

} // namespace delta

#endif // DELTA_U_POINTS_EXTRACTOR_HPP