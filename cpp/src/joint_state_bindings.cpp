#include <pybind11/pybind11.h>
#include "joint_state.hpp"
#include "fermat_module.hpp"

using namespace pybind11::literals;

PYBIND11_MODULE(joint_state_module, m) {
    m.doc() = "Delta robot joint state calculation module";
    
    // NO Vector3 registration - assumes delta_types is imported
    
    // JointStateResult - output structure
    pybind11::class_<delta::JointStateResult>(m, "JointStateResult")
        .def_readonly("prismatic_joint", &delta::JointStateResult::prismatic_joint)
        .def_readonly("roll_joint", &delta::JointStateResult::roll_joint)
        .def_readonly("pitch_joint", &delta::JointStateResult::pitch_joint)
        .def_readonly("direction_vector", &delta::JointStateResult::direction_vector)
        .def_readonly("fermat_point", &delta::JointStateResult::fermat_point)
        .def_readonly("z_A", &delta::JointStateResult::z_A)
        .def_readonly("z_B", &delta::JointStateResult::z_B)
        .def_readonly("z_C", &delta::JointStateResult::z_C)
        .def("__repr__", [](const delta::JointStateResult& r) {
            return "JointStateResult(prismatic=" + std::to_string(r.prismatic_joint) + 
                   ", roll=" + std::to_string(r.roll_joint) + 
                   ", pitch=" + std::to_string(r.pitch_joint) + 
                   ", direction=(" + std::to_string(r.direction_vector.x()) + "," +
                   std::to_string(r.direction_vector.y()) + "," + 
                   std::to_string(r.direction_vector.z()) + "))";
        });
    
    // JointStateModule - main interface
    pybind11::class_<delta::JointStateModule>(m, "JointStateModule")
        .def_static("calculate", 
                   &delta::JointStateModule::calculate,
                   "Calculate joint states from direction vector, fermat point, and Z values")
        .def_static("calculate_from_fermat", 
                   &delta::JointStateModule::calculate_from_fermat,
                   "Calculate joint states from direction vector and FermatResult");
}