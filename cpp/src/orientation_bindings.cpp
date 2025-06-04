#include <pybind11/pybind11.h>
#include "orientation_module.hpp"

using namespace pybind11::literals;

PYBIND11_MODULE(orientation_module, m) {
    m.doc() = "Delta robot orientation calculation module";
    
    // NO Vector3/Matrix4x4/CoordinateFrame registration - assumes delta_types is imported
    // NO KinematicsResult registration - assumes kinematics_module is imported
    
    // OrientationResult - main output structure with ALL coordinate frames
    pybind11::class_<delta::OrientationResult>(m, "OrientationResult")
        .def_readonly("transformation_matrix", &delta::OrientationResult::transformation_matrix)
        .def_readonly("fermat_point", &delta::OrientationResult::fermat_point)
        .def_readonly("end_effector_position", &delta::OrientationResult::end_effector_position)
        .def_readonly("UVW_at_fermat", &delta::OrientationResult::UVW_at_fermat)         // Step 1
        .def_readonly("IJK_mirrored", &delta::OrientationResult::IJK_mirrored)           // Step 2
        .def_readonly("UVW_prime_aligned", &delta::OrientationResult::UVW_prime_aligned) // Step 3
        .def_readonly("final_frame", &delta::OrientationResult::final_frame)             // Step 4
        .def("__repr__", [](const delta::OrientationResult& r) {
            return "OrientationResult(end_effector=(" + 
                   std::to_string(r.end_effector_position.x()) + "," +
                   std::to_string(r.end_effector_position.y()) + "," + 
                   std::to_string(r.end_effector_position.z()) + 
                   "), transformation_matrix=4x4, all_frames_available=True)";
        });
    
    // OrientationModule - main interface
    pybind11::class_<delta::OrientationModule>(m, "OrientationModule")
        .def_static("calculate", 
                   pybind11::overload_cast<double, double, double>(&delta::OrientationModule::calculate),
                   "Calculate from x,y,z coordinates")
        .def_static("calculate", 
                   pybind11::overload_cast<const delta::Vector3&>(&delta::OrientationModule::calculate),
                   "Calculate from Vector3")
        .def_static("calculate_from_kinematics",
                   &delta::OrientationModule::calculate_from_kinematics,
                   "Calculate from existing KinematicsResult");
}