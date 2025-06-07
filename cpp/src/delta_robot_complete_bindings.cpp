// consolidated_bindings.cpp - ALL modules in one file with collision manager
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

// Include all headers
#include "fabrik_initialization.hpp"
#include "fabrik_backward.hpp"
#include "fabrik_forward.hpp"
#include "fabrik_solver.hpp"
#include "fermat_module.hpp"
#include "joint_state.hpp"
#include "kinematics_module.hpp"
#include "orientation_module.hpp"
#include "motor_module.hpp"
#include "collision_manager.hpp"

using namespace pybind11::literals;

PYBIND11_MODULE(delta_robot_complete, m) {
    m.doc() = "Complete Delta Robot Module - All functionality in one place";
    
    // =============================================================================
    // TYPES & ENUMS
    // =============================================================================
    
    pybind11::enum_<delta::JointType>(m, "JointType")
        .value("FIXED_BASE", delta::JointType::FIXED_BASE)
        .value("SPHERICAL_120", delta::JointType::SPHERICAL_120)
        .value("END_EFFECTOR", delta::JointType::END_EFFECTOR);
    
    pybind11::enum_<delta::PassType>(m, "PassType")
        .value("BACKWARD", delta::PassType::BACKWARD)
        .value("FORWARD", delta::PassType::FORWARD);
    
    // =============================================================================
    // COLLISION SYSTEM
    // =============================================================================
    
    pybind11::class_<delta::CollisionPill>(m, "CollisionPill")
        .def_readonly("start_point", &delta::CollisionPill::start_point)
        .def_readonly("end_point", &delta::CollisionPill::end_point)
        .def_readonly("radius", &delta::CollisionPill::radius)
        .def_readonly("associated_joint_index", &delta::CollisionPill::associated_joint_index)
        .def("get_center", &delta::CollisionPill::get_center)
        .def("get_length", &delta::CollisionPill::get_length);
    
    // CollisionManager - Only expose functionality, not the class itself
    m.def("get_collision_pills", []() {
        return delta::CollisionManager::getInstance().get_active_pills();
    }, pybind11::return_value_policy::reference_internal,
       "Get the current active collision pills");
    
    // =============================================================================
    // CORE STRUCTURES
    // =============================================================================
    
    pybind11::class_<delta::FabrikJoint>(m, "FabrikJoint")
        .def_readwrite("position", &delta::FabrikJoint::position)
        .def_readwrite("type", &delta::FabrikJoint::type);
    
    pybind11::class_<delta::FabrikChain>(m, "FabrikChain")
        .def_readwrite("joints", &delta::FabrikChain::joints)
        .def_readwrite("segments", &delta::FabrikChain::segments);
    
    // =============================================================================
    // RESULT TYPES (Simple, no helpers)
    // =============================================================================
    
    pybind11::class_<delta::FermatResult>(m, "FermatResult")
        .def_readonly("fermat_point", &delta::FermatResult::fermat_point);
    
    pybind11::class_<delta::KinematicsResult>(m, "KinematicsResult")
        .def_readonly("end_effector_position", &delta::KinematicsResult::end_effector_position)
        .def_readonly("prismatic_joint_length", &delta::KinematicsResult::prismatic_joint_length);
    
    pybind11::class_<delta::JointStateResult>(m, "JointStateResult")
        .def_readonly("prismatic_joint", &delta::JointStateResult::prismatic_joint)
        .def_readonly("roll_joint", &delta::JointStateResult::roll_joint)
        .def_readonly("pitch_joint", &delta::JointStateResult::pitch_joint);
    
    pybind11::class_<delta::OrientationResult>(m, "OrientationResult")
        .def_readonly("transformation_matrix", &delta::OrientationResult::transformation_matrix)
        .def_readonly("end_effector_position", &delta::OrientationResult::end_effector_position);
    
    pybind11::class_<delta::FabrikSolutionResult>(m, "FabrikSolutionResult")
        .def_readonly("final_chain", &delta::FabrikSolutionResult::final_chain)
        .def_readonly("converged", &delta::FabrikSolutionResult::converged)
        .def_readonly("final_error", &delta::FabrikSolutionResult::final_error);
    
    pybind11::class_<delta::MotorResult>(m, "MotorResult")
        .def_readonly("target_position", &delta::MotorResult::target_position)
        .def_readonly("fabrik_converged", &delta::MotorResult::fabrik_converged)
        .def_readonly("fabrik_error", &delta::MotorResult::fabrik_error)
        .def_readonly("solve_time_ms", &delta::MotorResult::solve_time_ms)
        .def_readonly("original_segment_numbers", &delta::MotorResult::original_segment_numbers)
        .def_readonly("original_segment_positions", &delta::MotorResult::original_segment_positions)
        .def_readonly("fabrik_joint_positions", &delta::MotorResult::fabrik_joint_positions)
        .def_readonly("levels", &delta::MotorResult::levels);
    
    pybind11::class_<delta::LevelData>(m, "LevelData")
        .def_readonly("z_A", &delta::LevelData::z_A)
        .def_readonly("z_B", &delta::LevelData::z_B)
        .def_readonly("z_C", &delta::LevelData::z_C)
        .def_readonly("base_segment_position", &delta::LevelData::base_segment_position)
        .def_readonly("prismatic_joint", &delta::LevelData::prismatic_joint)
        .def_readonly("roll_joint", &delta::LevelData::roll_joint)
        .def_readonly("pitch_joint", &delta::LevelData::pitch_joint)
        .def_readonly("transformed_segment_original_numbers", &delta::LevelData::transformed_segment_original_numbers)
        .def_readonly("transformed_segment_positions", &delta::LevelData::transformed_segment_positions);
    
    // =============================================================================
    // KINEMATICS MODULES
    // =============================================================================
    
    pybind11::class_<delta::FermatModule>(m, "FermatModule")
        .def_static("calculate", 
                   pybind11::overload_cast<double, double, double>(&delta::FermatModule::calculate))
        .def_static("calculate", 
                   pybind11::overload_cast<const delta::Vector3&>(&delta::FermatModule::calculate));
    
    pybind11::class_<delta::JointStateModule>(m, "JointStateModule")
        .def_static("calculate_from_fermat", &delta::JointStateModule::calculate_from_fermat);
    
    pybind11::class_<delta::KinematicsModule>(m, "KinematicsModule")
        .def_static("calculate", 
                   pybind11::overload_cast<double, double, double>(&delta::KinematicsModule::calculate))
        .def_static("calculate", 
                   pybind11::overload_cast<const delta::Vector3&>(&delta::KinematicsModule::calculate));
    
    pybind11::class_<delta::OrientationModule>(m, "OrientationModule")
        .def_static("calculate", 
                   pybind11::overload_cast<double, double, double>(&delta::OrientationModule::calculate))
        .def_static("calculate", 
                   pybind11::overload_cast<const delta::Vector3&>(&delta::OrientationModule::calculate));
    
    // =============================================================================
    // FABRIK MODULES
    // =============================================================================
    
    pybind11::class_<delta::FabrikInitialization>(m, "FabrikInitialization")
        .def_static("initialize_straight_up", &delta::FabrikInitialization::initialize_straight_up);
    
    pybind11::class_<delta::FabrikSolver>(m, "FabrikSolver")
        .def_static("solve", &delta::FabrikSolver::solve);
    
    // =============================================================================
    // MOTOR MODULE
    // =============================================================================
    
    pybind11::class_<delta::MotorModule>(m, "MotorModule")
        .def_static("calculate_motors", 
                   pybind11::overload_cast<double, double, double>(&delta::MotorModule::calculate_motors))
        .def_static("calculate_motors", 
                   pybind11::overload_cast<const delta::Vector3&>(&delta::MotorModule::calculate_motors));
    
    // =============================================================================
    // CONVENIENCE FUNCTIONS
    // =============================================================================
    
    m.def("solve_delta_robot", &delta::fabrik_utils::solve_delta_robot);
    m.def("calculate_motors", [](double x, double y, double z) {
        return delta::MotorModule::calculate_motors(x, y, z);
    });
    
    // =============================================================================
    // CONSTANTS
    // =============================================================================
    
    m.attr("FABRIK_TOLERANCE") = delta::FABRIK_TOLERANCE;
    m.attr("DEFAULT_ROBOT_SEGMENTS") = delta::DEFAULT_ROBOT_SEGMENTS;
    m.attr("COLLISION_PILL_RADIUS") = delta::COLLISION_PILL_RADIUS;
}