#include "fermat_module.hpp"

namespace delta {

FermatResult FermatModule::calculate(double x, double y, double z) {
    return calculate(Vector3(x, y, z));
}

FermatResult FermatModule::calculate(const Vector3& direction) {
    FermatCalculation calc(direction);
    
    return FermatResult(
        calc.A_point.z(),
        calc.B_point.z(), 
        calc.C_point.z(),
        calc.fermat_point
    );
}

FermatCalculation FermatModule::detailed_calculation(const Vector3& direction) {
    return FermatCalculation(direction);
}

} // namespace delta