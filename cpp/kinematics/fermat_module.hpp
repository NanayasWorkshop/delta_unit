#ifndef DELTA_FERMAT_MODULE_HPP
#define DELTA_FERMAT_MODULE_HPP

#include "../core/math_utils.hpp"
#include <chrono>

namespace delta {

struct FermatResult {
    double z_A, z_B, z_C;           // Z positions of points A, B, C
    Vector3 fermat_point;           // Final Fermat point
    double computation_time_ms;     // Time taken for calculation
    bool calculation_successful;    // Whether calculation completed successfully
    
    FermatResult(double zA, double zB, double zC, const Vector3& fermat, double time_ms = 0.0, bool success = true)
        : z_A(zA), z_B(zB), z_C(zC), fermat_point(fermat), computation_time_ms(time_ms), calculation_successful(success) {}
    
    // Failed result constructor
    static FermatResult failed(double time_ms = 0.0) {
        return FermatResult(0.0, 0.0, 0.0, Vector3(0, 0, 0), time_ms, false);
    }
};

class FermatSolver {
public:
    // Main interface: input direction vector, get essential outputs with timing
    static FermatResult calculate(double x, double y, double z);
    static FermatResult calculate(const Vector3& direction);
    
    // Get base positions (for reference)
    static Vector3 get_base_A() { return get_base_position_A(); }
    static Vector3 get_base_B() { return get_base_position_B(); }
    static Vector3 get_base_C() { return get_base_position_C(); }
    
    // Validation
    static bool is_direction_valid(const Vector3& direction);

private:
    // Core calculation with timing
    static FermatResult calculate_internal(const Vector3& direction);
};

// Backward compatibility alias
using FermatModule = FermatSolver;

} // namespace delta

#endif // DELTA_FERMAT_MODULE_HPP