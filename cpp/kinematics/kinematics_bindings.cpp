#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "fermat_module.hpp"
#include "joint_state.hpp"
#include "kinematics_module.hpp"
#include "orientation_module.hpp"

using namespace pybind11::literals;

PYBIND11_MODULE(kinematics_complete, m) {
    m.doc() = "Delta robot complete kinematics module - Consolidated fermat, joint_state, kinematics, and orientation";
    
    // =============================================================================
    // FERMAT SUBMODULE
    // =============================================================================
    
    // FermatResult - clean output structure
    pybind11::class_<delta::FermatResult>(m, "FermatResult")
        .def_readonly("z_A", &delta::FermatResult::z_A)
        .def_readonly("z_B", &delta::FermatResult::z_B)
        .def_readonly("z_C", &delta::FermatResult::z_C)
        .def_readonly("fermat_point", &delta::FermatResult::fermat_point)
        .def("__repr__", [](const delta::FermatResult& r) {
            return "FermatResult(z_A=" + std::to_string(r.z_A) + 
                   ", z_B=" + std::to_string(r.z_B) + 
                   ", z_C=" + std::to_string(r.z_C) + 
                   ", fermat=(" + std::to_string(r.fermat_point.x()) + "," +
                   std::to_string(r.fermat_point.y()) + "," + 
                   std::to_string(r.fermat_point.z()) + "))";
        });
    
    // FermatModule - main interface
    pybind11::class_<delta::FermatModule>(m, "FermatModule")
        .def_static("calculate", 
                   pybind11::overload_cast<double, double, double>(&delta::FermatModule::calculate),
                   "Calculate from x,y,z coordinates")
        .def_static("calculate", 
                   pybind11::overload_cast<const delta::Vector3&>(&delta::FermatModule::calculate),
                   "Calculate from Vector3")
        .def_static("get_base_A", &delta::FermatModule::get_base_A)
        .def_static("get_base_B", &delta::FermatModule::get_base_B)
        .def_static("get_base_C", &delta::FermatModule::get_base_C);

    // =============================================================================
    // JOINT STATE SUBMODULE
    // =============================================================================
    
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

    // =============================================================================
    // KINEMATICS SUBMODULE
    // =============================================================================
    
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
                   std::to_string(r.end_effector_position.x()) + "," +
                   std::to_string(r.end_effector_position.y()) + "," + 
                   std::to_string(r.end_effector_position.z()) + 
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

    // =============================================================================
    // ORIENTATION SUBMODULE
    // =============================================================================
    
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