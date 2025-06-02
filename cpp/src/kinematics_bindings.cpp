#include <pybind11/pybind11.h>
#include "kinematics_module.hpp"

using namespace pybind11::literals;

PYBIND11_MODULE(kinematics_module, m) {
    m.doc() = "Delta robot kinematics calculation module";
    
    // NO Vector3 registration - assumes delta_types is imported
    // NO FermatResult/JointStateResult registration - assumes those modules are imported
    
    // KinematicsResult - main output structure
    pybind11::class_<delta::KinematicsResult>(m, "KinematicsResult")
        .def_readonly("end_effector_position", &delta::KinematicsResult::end_effector_position)
        .def_readonly("prismatic_joint_length", &delta::KinematicsResult::prismatic_joint_length)
        .def_readonly("transformed_vector", &delta::KinematicsResult::transformed_vector)
        .def_readonly("original_input", &delta::KinematicsResult::original_input)
        .def_readonly("input_angle_from_z", &delta::KinematicsResult::input_angle_from_z)
        .def_readonly("fermat_data", &delta::KinematicsResult::fermat_data)
        .def_readonly("joint_state_data", &delta::KinematicsResult::joint_state_data)
        .def("__repr__", [](const delta::KinematicsResult& r) {
            return "KinematicsResult(end_effector=(" + 
                   std::to_string(r.end_effector_position.x) + "," +
                   std::to_string(r.end_effector_position.y) + "," + 
                   std::to_string(r.end_effector_position.z) + 
                   "), prismatic_length=" + std::to_string(r.prismatic_joint_length) + 
                   ", angle_from_z=" + std::to_string(r.input_angle_from_z) + ")";
        });
    
    // KinematicsModule - main interface
    pybind11::class_<delta::KinematicsModule>(m, "KinematicsModule")
        .def_static("calculate", 
                   pybind11::overload_cast<double, double, double>(&delta::KinematicsModule::calculate),
                   "Calculate from x,y,z coordinates")
        .def_static("calculate", 
                   pybind11::overload_cast<const delta::Vector3&>(&delta::KinematicsModule::calculate),
                   "Calculate from Vector3");
}