#ifndef DELTA_FERMAT_MODULE_HPP
#define DELTA_FERMAT_MODULE_HPP

#include "math_utils.hpp"

namespace delta {

struct FermatResult {
    double z_A, z_B, z_C;           // Z positions of points A, B, C
    Vector3 fermat_point;           // Final Fermat point
    
    FermatResult(double zA, double zB, double zC, const Vector3& fermat)
        : z_A(zA), z_B(zB), z_C(zC), fermat_point(fermat) {}
};

class FermatModule {
public:
    // Main interface: input direction vector, get essential outputs
    static FermatResult calculate(double x, double y, double z);
    static FermatResult calculate(const Vector3& direction);
    
    // Get base positions (for reference)
    static Vector3 get_base_A() { return get_base_position_A(); }
    static Vector3 get_base_B() { return get_base_position_B(); }
    static Vector3 get_base_C() { return get_base_position_C(); }
    
private:
    // Internal detailed calculation (if needed for debugging)
    static FermatCalculation detailed_calculation(const Vector3& direction);
};

} // namespace delta

#endif // DELTA_FERMAT_MODULE_HPP
