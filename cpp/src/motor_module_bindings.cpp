#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "motor_module.hpp"

using namespace pybind11::literals;

PYBIND11_MODULE(motor_module, m) {
    m.doc() = "Delta robot motor orchestration module";
    
    // NO Vector3 registration - assumes delta_types is imported
    
    // MotorResult structure
    pybind11::class_<delta::MotorResult>(m, "MotorResult")
        .def_readonly("target_position", &delta::MotorResult::target_position)
        .def_readonly("fabrik_converged", &delta::MotorResult::fabrik_converged)
        .def_readonly("fabrik_error", &delta::MotorResult::fabrik_error)
        .def_readonly("segment_numbers", &delta::MotorResult::segment_numbers)
        .def_readonly("segment_positions", &delta::MotorResult::segment_positions)
        .def_readonly("first_segment_position", &delta::MotorResult::first_segment_position)
        .def_readonly("z_A", &delta::MotorResult::z_A)
        .def_readonly("z_B", &delta::MotorResult::z_B)
        .def_readonly("z_C", &delta::MotorResult::z_C)
        .def_readonly("prismatic_joint", &delta::MotorResult::prismatic_joint)
        .def_readonly("roll_joint", &delta::MotorResult::roll_joint)
        .def_readonly("pitch_joint", &delta::MotorResult::pitch_joint)
        .def_readonly("uvw_origin", &delta::MotorResult::uvw_origin)
        .def_readonly("uvw_u_axis", &delta::MotorResult::uvw_u_axis)
        .def_readonly("uvw_v_axis", &delta::MotorResult::uvw_v_axis)
        .def_readonly("uvw_w_axis", &delta::MotorResult::uvw_w_axis)
        .def_readonly("transformed_segment_numbers", &delta::MotorResult::transformed_segment_numbers)
        .def_readonly("transformed_segment_positions", &delta::MotorResult::transformed_segment_positions)
        .def_readonly("second_segment_position", &delta::MotorResult::second_segment_position)
        .def_readonly("second_z_A", &delta::MotorResult::second_z_A)
        .def_readonly("second_z_B", &delta::MotorResult::second_z_B)
        .def_readonly("second_z_C", &delta::MotorResult::second_z_C)
        .def_readonly("second_prismatic_joint", &delta::MotorResult::second_prismatic_joint)
        .def_readonly("second_roll_joint", &delta::MotorResult::second_roll_joint)
        .def_readonly("second_pitch_joint", &delta::MotorResult::second_pitch_joint)
        .def_readonly("second_uvw_origin", &delta::MotorResult::second_uvw_origin)
        .def_readonly("second_uvw_u_axis", &delta::MotorResult::second_uvw_u_axis)
        .def_readonly("second_uvw_v_axis", &delta::MotorResult::second_uvw_v_axis)
        .def_readonly("second_uvw_w_axis", &delta::MotorResult::second_uvw_w_axis)
        .def_readonly("second_level_transformed_segment_numbers", &delta::MotorResult::second_level_transformed_segment_numbers)
        .def_readonly("second_level_transformed_segment_positions", &delta::MotorResult::second_level_transformed_segment_positions)
        .def_readonly("third_segment_position", &delta::MotorResult::third_segment_position)
        .def_readonly("third_z_A", &delta::MotorResult::third_z_A)
        .def_readonly("third_z_B", &delta::MotorResult::third_z_B)
        .def_readonly("third_z_C", &delta::MotorResult::third_z_C)
        .def_readonly("third_prismatic_joint", &delta::MotorResult::third_prismatic_joint)
        .def_readonly("third_roll_joint", &delta::MotorResult::third_roll_joint)
        .def_readonly("third_pitch_joint", &delta::MotorResult::third_pitch_joint)
        .def_readonly("third_uvw_origin", &delta::MotorResult::third_uvw_origin)
        .def_readonly("third_uvw_u_axis", &delta::MotorResult::third_uvw_u_axis)
        .def_readonly("third_uvw_v_axis", &delta::MotorResult::third_uvw_v_axis)
        .def_readonly("third_uvw_w_axis", &delta::MotorResult::third_uvw_w_axis)
        .def("__repr__", [](const delta::MotorResult& r) {
            return "MotorResult(target=(" + 
                   std::to_string(r.target_position.x) + "," +
                   std::to_string(r.target_position.y) + "," + 
                   std::to_string(r.target_position.z) + 
                   "), converged=" + (r.fabrik_converged ? "True" : "False") +
                   ", segments=" + std::to_string(r.segment_numbers.size()) +
                   ", transformed=" + std::to_string(r.transformed_segment_numbers.size()) +
                   ", second_level=" + std::to_string(r.second_level_transformed_segment_numbers.size()) + ")";
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