#include "fermat_module.hpp"
#include <chrono>

namespace delta {

FermatResult FermatSolver::calculate(double x, double y, double z) {
    return calculate(Vector3(x, y, z));
}

FermatResult FermatSolver::calculate(const Vector3& direction) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Validate input
    if (!is_direction_valid(direction)) {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        return FermatResult::failed(duration.count() / 1000.0);
    }
    
    // Perform calculation
    FermatResult result = calculate_internal(direction);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    result.computation_time_ms = duration.count() / 1000.0;
    
    return result;
}

bool FermatSolver::is_direction_valid(const Vector3& direction) {
    // Check for zero vector
    if (direction.norm() < EPSILON_MATH) {
        return false;
    }
    
    // Check for reasonable magnitude
    double norm = direction.norm();
    if (norm > 1000.0 || norm < 0.001) {
        return false;
    }
    
    return true;
}

FermatResult FermatSolver::calculate_internal(const Vector3& direction) {
    // Use existing FermatCalculation for core math
    FermatCalculation calc(direction);
    
    return FermatResult(
        calc.A_point.z(),
        calc.B_point.z(), 
        calc.C_point.z(),
        calc.fermat_point
    );
}

} // namespace delta