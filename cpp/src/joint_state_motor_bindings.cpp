#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "joint_state_motor.hpp"

using namespace pybind11::literals;

PYBIND11_MODULE(joint_state_motor, m) {
    m.doc() = "Delta robot joint state motor module - Converts targets to motor positions";
    
    // SegmentMotorData structure - Motor data for individual segments
    pybind11::class_<delta::SegmentMotorData>(m, "SegmentMotorData")
        .def_readonly("segment_number", &delta::SegmentMotorData::segment_number)
        .def_readonly("fabrik_position", &delta::SegmentMotorData::fabrik_position)
        .def_readonly("local_position", &delta::SegmentMotorData::local_position)
        .def_readonly("z_A", &delta::SegmentMotorData::z_A)
        .def_readonly("z_B", &delta::SegmentMotorData::z_B)
        .def_readonly("z_C", &delta::SegmentMotorData::z_C)
        .def_readonly("prismatic_joint", &delta::SegmentMotorData::prismatic_joint)
        .def_readonly("roll_joint", &delta::SegmentMotorData::roll_joint)
        .def_readonly("pitch_joint", &delta::SegmentMotorData::pitch_joint)
        .def_readonly("uvw_frame", &delta::SegmentMotorData::uvw_frame)
        .def_readonly("local_to_world", &delta::SegmentMotorData::local_to_world)
        .def("__repr__", [](const delta::SegmentMotorData& s) {
            return "SegmentMotorData(segment=" + std::to_string(s.segment_number) + 
                   ", fabrik=(" + std::to_string(s.fabrik_position.x) + "," + 
                   std::to_string(s.fabrik_position.y) + "," + 
                   std::to_string(s.fabrik_position.z) + 
                   "), local=(" + std::to_string(s.local_position.x) + "," + 
                   std::to_string(s.local_position.y) + "," + 
                   std::to_string(s.local_position.z) + 
                   "), motors=[zA=" + std::to_string(s.z_A) + 
                   ", zB=" + std::to_string(s.z_B) + 
                   ", zC=" + std::to_string(s.z_C) + 
                   ", prismatic=" + std::to_string(s.prismatic_joint) + 
                   ", roll=" + std::to_string(s.roll_joint * 180 / 3.14159) + "°" +
                   ", pitch=" + std::to_string(s.pitch_joint * 180 / 3.14159) + "°])";
        });
    
    // JointStateMotorResult structure - Main output with all segment motor data
    pybind11::class_<delta::JointStateMotorResult>(m, "JointStateMotorResult")
        .def_readonly("target_position", &delta::JointStateMotorResult::target_position)
        .def_readonly("achieved_end_effector", &delta::JointStateMotorResult::achieved_end_effector)
        .def_readonly("fabrik_converged", &delta::JointStateMotorResult::fabrik_converged)
        .def_readonly("fabrik_error", &delta::JointStateMotorResult::fabrik_error)
        .def_readonly("fabrik_iterations", &delta::JointStateMotorResult::fabrik_iterations)
        .def_readonly("solve_time_ms", &delta::JointStateMotorResult::solve_time_ms)
        .def_readonly("fabrik_result", &delta::JointStateMotorResult::fabrik_result)
        .def_readonly("all_segment_motors", &delta::JointStateMotorResult::all_segment_motors)
        .def("__repr__", [](const delta::JointStateMotorResult& r) {
            return "JointStateMotorResult(target=(" + 
                   std::to_string(r.target_position.x) + "," +
                   std::to_string(r.target_position.y) + "," + 
                   std::to_string(r.target_position.z) + 
                   "), converged=" + (r.fabrik_converged ? "True" : "False") +
                   ", error=" + std::to_string(r.fabrik_error) +
                   ", iterations=" + std::to_string(r.fabrik_iterations) +
                   ", time=" + std::to_string(r.solve_time_ms) + "ms" +
                   ", segments=" + std::to_string(r.all_segment_motors.size()) + ")";
        });
    
    // JointStateMotorModule - Main interface
    pybind11::class_<delta::JointStateMotorModule>(m, "JointStateMotorModule")
        .def_static("calculate_motors",
                   pybind11::overload_cast<double, double, double>(&delta::JointStateMotorModule::calculate_motors),
                   "target_x"_a, "target_y"_a, "target_z"_a,
                   "Calculate motor positions from target coordinates")
        .def_static("calculate_motors",
                   pybind11::overload_cast<const delta::Vector3&>(&delta::JointStateMotorModule::calculate_motors),
                   "target_position"_a,
                   "Calculate motor positions from target Vector3")
        .def_static("calculate_motors_advanced",
                   &delta::JointStateMotorModule::calculate_motors_advanced,
                   "target_position"_a, "num_segments"_a, "tolerance"_a, "max_iterations"_a,
                   "Calculate motor positions with advanced FABRIK configuration")
        .def_static("is_target_reachable",
                   &delta::JointStateMotorModule::is_target_reachable,
                   "target_position"_a, "num_segments"_a = delta::DEFAULT_ROBOT_SEGMENTS,
                   "Check if target is within robot workspace");
    
    // Utility functions
    m.def("solve_target", &delta::joint_state_utils::solve_target,
          "x"_a, "y"_a, "z"_a,
          "Quick solve for target with default parameters");
    
    m.def("solve_multiple_targets", &delta::joint_state_utils::solve_multiple_targets,
          "targets"_a,
          "Solve multiple targets in batch");
    
    m.def("test_workspace_point", &delta::joint_state_utils::test_workspace_point,
          "x"_a, "y"_a, "z"_a,
          "Test if point is reachable and print result");
    
    // Constants
    m.attr("DEFAULT_ROBOT_SEGMENTS") = delta::DEFAULT_ROBOT_SEGMENTS;
    m.attr("FABRIK_TOLERANCE") = delta::FABRIK_TOLERANCE;
    m.attr("FABRIK_MAX_ITERATIONS") = delta::FABRIK_MAX_ITERATIONS;
}