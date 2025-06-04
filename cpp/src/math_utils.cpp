#include "math_utils.hpp"
#include <algorithm>

namespace delta {

double calculate_z_intersection(double base_x, double base_y, const Vector3& normal) {
    // Plane equation: normal.x * x + normal.y * y + normal.z * z = 0 (plane through origin)
    // Point on vertical line: (base_x, base_y, z)
    // Solve for z: normal.x * base_x + normal.y * base_y + normal.z * z = 0
    // z = -(normal.x * base_x + normal.y * base_y) / normal.z
    
    if (std::abs(normal.z()) < 1e-10) {
        // Normal is horizontal, no intersection or infinite intersections
        return 0.0;
    }
    
    return -(normal.x() * base_x + normal.y() * base_y) / normal.z();
}

FermatCalculation::FermatCalculation(const Vector3& direction) {
    // Normalize direction vector
    Vector3 normal = direction.normalized();
    
    // Get base positions
    Vector3 base_A = get_base_position_A();
    Vector3 base_B = get_base_position_B(); 
    Vector3 base_C = get_base_position_C();
    
    // Calculate Z intersections with plane
    double z_A = calculate_z_intersection(base_A.x(), base_A.y(), normal);
    double z_B = calculate_z_intersection(base_B.x(), base_B.y(), normal);
    double z_C = calculate_z_intersection(base_C.x(), base_C.y(), normal);
    
    // Create 3D points
    A_point = Vector3(base_A.x(), base_A.y(), z_A);
    B_point = Vector3(base_B.x(), base_B.y(), z_B);
    C_point = Vector3(base_C.x(), base_C.y(), z_C);
    
    // Calculate vectors
    AB = B_point - A_point;
    BC = C_point - B_point;
    CA = A_point - C_point;
    
    // Calculate side lengths
    side_a = BC.norm();
    side_b = CA.norm();
    side_c = AB.norm();
    
    // Calculate angles using dot product
    Vector3 neg_CA = -CA;
    Vector3 neg_AB = -AB;
    Vector3 neg_BC = -BC;
    
    alpha = std::acos(std::max(-1.0, std::min(1.0, neg_CA.dot(AB) / (CA.norm() * AB.norm()))));
    beta = std::acos(std::max(-1.0, std::min(1.0, neg_AB.dot(BC) / (AB.norm() * BC.norm()))));
    gamma = std::acos(std::max(-1.0, std::min(1.0, neg_BC.dot(CA) / (BC.norm() * CA.norm()))));
    
    // Calculate Lambda values with safety checks
    double sin_alpha = std::sin(alpha + M_PI/3);
    double sin_beta = std::sin(beta + M_PI/3);
    double sin_gamma = std::sin(gamma + M_PI/3);
    
    const double epsilon = 1e-10;
    lambda_A = side_a / std::max(sin_alpha, epsilon);
    lambda_B = side_b / std::max(sin_beta, epsilon);
    lambda_C = side_c / std::max(sin_gamma, epsilon);
    
    // Calculate Fermat point
    double total_lambda = lambda_A + lambda_B + lambda_C;
    double fermat_x = (lambda_A * A_point.x() + lambda_B * B_point.x() + lambda_C * C_point.x()) / total_lambda;
    double fermat_y = (lambda_A * A_point.y() + lambda_B * B_point.y() + lambda_C * C_point.y()) / total_lambda;
    double fermat_z = (lambda_A * A_point.z() + lambda_B * B_point.z() + lambda_C * C_point.z()) / total_lambda;
    
    fermat_point = Vector3(fermat_x, fermat_y, fermat_z);
}

} // namespace delta