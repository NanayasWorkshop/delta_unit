#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "motor_module.hpp"

using namespace pybind11::literals;

PYBIND11_MODULE(motor_module, m) {
    m.doc() = "Delta robot motor orchestration module (dynamic levels with joint position support)";
    
    // LevelData structure with proper Eigen handling
    pybind11::class_<delta::LevelData>(m, "LevelData")
        .def_property_readonly("base_segment_position", [](const delta::LevelData& ld) {
            // Return as numpy array for easy Python access
            return ld.base_segment_position;
        })
        .def_readonly("z_A", &delta::LevelData::z_A)
        .def_readonly("z_B", &delta::LevelData::z_B)
        .def_readonly("z_C", &delta::LevelData::z_C)
        .def_readonly("prismatic_joint", &delta::LevelData::prismatic_joint)
        .def_readonly("roll_joint", &delta::LevelData::roll_joint)
        .def_readonly("pitch_joint", &delta::LevelData::pitch_joint)
        .def_property_readonly("uvw_origin", [](const delta::LevelData& ld) {
            return ld.uvw_origin;
        })
        .def_property_readonly("uvw_u_axis", [](const delta::LevelData& ld) {
            return ld.uvw_u_axis;
        })
        .def_property_readonly("uvw_v_axis", [](const delta::LevelData& ld) {
            return ld.uvw_v_axis;
        })
        .def_property_readonly("uvw_w_axis", [](const delta::LevelData& ld) {
            return ld.uvw_w_axis;
        })
        .def_readonly("transformed_segment_original_numbers", &delta::LevelData::transformed_segment_original_numbers)
        .def_property_readonly("transformed_segment_positions", [](const delta::LevelData& ld) {
            return ld.transformed_segment_positions;
        })
        .def("__repr__", [](const delta::LevelData& ld) {
            return "LevelData(base_pos=(" + 
                   std::to_string(ld.base_segment_position.x()) + "," +
                   std::to_string(ld.base_segment_position.y()) + "," +
                   std::to_string(ld.base_segment_position.z()) + 
                   "), transformed_count=" + std::to_string(ld.transformed_segment_positions.size()) + ")";
        });

    // MotorResult structure with proper Eigen handling
    pybind11::class_<delta::MotorResult>(m, "MotorResult")
        .def_property_readonly("target_position", [](const delta::MotorResult& r) {
            return r.target_position;
        })
        .def_readonly("fabrik_converged", &delta::MotorResult::fabrik_converged)
        .def_readonly("fabrik_error", &delta::MotorResult::fabrik_error)
        .def_readonly("solve_time_ms", &delta::MotorResult::solve_time_ms)
        .def_readonly("original_segment_numbers", &delta::MotorResult::original_segment_numbers)
        .def_property_readonly("original_segment_positions", [](const delta::MotorResult& r) {
            return r.original_segment_positions;
        })
        .def_property_readonly("fabrik_joint_positions", [](const delta::MotorResult& r) {
            // NEW: Expose FABRIK joint positions
            return r.fabrik_joint_positions;
        })
        .def_readonly("levels", &delta::MotorResult::levels)
        .def("__repr__", [](const delta::MotorResult& r) {
            return "MotorResult(target=(" + 
                   std::to_string(r.target_position.x()) + "," +
                   std::to_string(r.target_position.y()) + "," + 
                   std::to_string(r.target_position.z()) + 
                   "), converged=" + (r.fabrik_converged ? "True" : "False") +
                   ", original_segments=" + std::to_string(r.original_segment_numbers.size()) +
                   ", joints=" + std::to_string(r.fabrik_joint_positions.size()) +
                   ", levels=" + std::to_string(r.levels.size()) + 
                   ", time=" + std::to_string(r.solve_time_ms) + "ms)";
        });
    
    // MotorModule class with updated overloads
    pybind11::class_<delta::MotorModule>(m, "MotorModule")
        .def_static("calculate_motors", 
                   [](double x, double y, double z) { 
                       return delta::MotorModule::calculate_motors(x, y, z); 
                   },
                   "target_x"_a, "target_y"_a, "target_z"_a,
                   "Calculate motor positions for target coordinates")
        .def_static("calculate_motors", 
                   [](const delta::Vector3& target) { 
                       return delta::MotorModule::calculate_motors(target); 
                   },
                   "target_position"_a,
                   "Calculate motor positions for target Vector3")
        .def_static("calculate_motors", 
                   [](double x, double y, double z, const std::vector<delta::Vector3>& current_joints) { 
                       return delta::MotorModule::calculate_motors(x, y, z, current_joints); 
                   },
                   "target_x"_a, "target_y"_a, "target_z"_a, "current_joint_positions"_a,
                   "Calculate motor positions with current joint positions")
        .def_static("calculate_motors", 
                   [](const delta::Vector3& target, const std::vector<delta::Vector3>& current_joints) { 
                       return delta::MotorModule::calculate_motors(target, current_joints); 
                   },
                   "target_position"_a, "current_joint_positions"_a,
                   "Calculate motor positions for target Vector3 with current joint positions");
}