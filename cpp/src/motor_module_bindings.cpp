#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "motor_module.hpp"

using namespace pybind11::literals;

PYBIND11_MODULE(motor_module, m) {
    m.doc() = "Delta robot motor orchestration module (dynamic levels)";
    
    // NO Vector3 registration - assumes delta_types is imported
    
    // LevelData structure
    pybind11::class_<delta::LevelData>(m, "LevelData")
        .def_readonly("base_segment_position", &delta::LevelData::base_segment_position)
        .def_readonly("z_A", &delta::LevelData::z_A)
        .def_readonly("z_B", &delta::LevelData::z_B)
        .def_readonly("z_C", &delta::LevelData::z_C)
        .def_readonly("prismatic_joint", &delta::LevelData::prismatic_joint)
        .def_readonly("roll_joint", &delta::LevelData::roll_joint)
        .def_readonly("pitch_joint", &delta::LevelData::pitch_joint)
        .def_readonly("uvw_origin", &delta::LevelData::uvw_origin)
        .def_readonly("uvw_u_axis", &delta::LevelData::uvw_u_axis)
        .def_readonly("uvw_v_axis", &delta::LevelData::uvw_v_axis)
        .def_readonly("uvw_w_axis", &delta::LevelData::uvw_w_axis)
        .def_readonly("transformed_segment_original_numbers", &delta::LevelData::transformed_segment_original_numbers)
        .def_readonly("transformed_segment_positions", &delta::LevelData::transformed_segment_positions)
        .def("__repr__", [](const delta::LevelData& ld) {
            return "LevelData(base_pos=(" + 
                   std::to_string(ld.base_segment_position.x) + "," +
                   std::to_string(ld.base_segment_position.y) + "," +
                   std::to_string(ld.base_segment_position.z) + 
                   "), transformed_count=" + std::to_string(ld.transformed_segment_positions.size()) + ")";
        });

    // MotorResult structure
    pybind11::class_<delta::MotorResult>(m, "MotorResult")
        .def_readonly("target_position", &delta::MotorResult::target_position)
        .def_readonly("fabrik_converged", &delta::MotorResult::fabrik_converged)
        .def_readonly("fabrik_error", &delta::MotorResult::fabrik_error)
        .def_readonly("solve_time_ms", &delta::MotorResult::solve_time_ms)
        .def_readonly("original_segment_numbers", &delta::MotorResult::original_segment_numbers)
        .def_readonly("original_segment_positions", &delta::MotorResult::original_segment_positions)
        .def_readonly("levels", &delta::MotorResult::levels)
        .def("__repr__", [](const delta::MotorResult& r) {
            return "MotorResult(target=(" + 
                   std::to_string(r.target_position.x) + "," +
                   std::to_string(r.target_position.y) + "," + 
                   std::to_string(r.target_position.z) + 
                   "), converged=" + (r.fabrik_converged ? "True" : "False") +
                   ", original_segments=" + std::to_string(r.original_segment_numbers.size()) +
                   ", levels=" + std::to_string(r.levels.size()) + 
                   ", time=" + std::to_string(r.solve_time_ms) + "ms)";
        });
    
    // MotorModule class
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
                   "Calculate motor positions for target Vector3");
}