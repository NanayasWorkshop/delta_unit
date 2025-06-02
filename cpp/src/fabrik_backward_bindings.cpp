#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "fabrik_backward.hpp"

using namespace pybind11::literals;

PYBIND11_MODULE(fabrik_backward, m) {
    m.doc() = "Delta robot FABRIK backward iteration module";
    
    // NO Vector3/FabrikChain registration - assumes delta_types and fabrik_initialization are imported
    
    // BackwardStepResult structure
    pybind11::class_<delta::BackwardStepResult>(m, "BackwardStepResult")
        .def_readonly("new_joint_position", &delta::BackwardStepResult::new_joint_position)
        .def_readonly("constraint_applied", &delta::BackwardStepResult::constraint_applied)
        .def_readonly("constraint_violation", &delta::BackwardStepResult::constraint_violation)
        .def("__repr__", [](const delta::BackwardStepResult& r) {
            return "BackwardStepResult(pos=(" + 
                   std::to_string(r.new_joint_position.x) + "," +
                   std::to_string(r.new_joint_position.y) + "," + 
                   std::to_string(r.new_joint_position.z) + 
                   "), constrained=" + (r.constraint_applied ? "True" : "False") +
                   ", violation=" + std::to_string(r.constraint_violation) + ")";
        });
    
    // FabrikBackwardResult structure
    pybind11::class_<delta::FabrikBackwardResult>(m, "FabrikBackwardResult")
        .def_readonly("updated_chain", &delta::FabrikBackwardResult::updated_chain)
        .def_readonly("target_position", &delta::FabrikBackwardResult::target_position)
        .def_readonly("target_reachable", &delta::FabrikBackwardResult::target_reachable)
        .def_readonly("distance_to_base", &delta::FabrikBackwardResult::distance_to_base)
        .def_readonly("iterations_used", &delta::FabrikBackwardResult::iterations_used)
        .def_readonly("iteration_history", &delta::FabrikBackwardResult::iteration_history)
        .def("__repr__", [](const delta::FabrikBackwardResult& r) {
            return "FabrikBackwardResult(target=(" + 
                   std::to_string(r.target_position.x) + "," +
                   std::to_string(r.target_position.y) + "," + 
                   std::to_string(r.target_position.z) + 
                   "), reachable=" + (r.target_reachable ? "True" : "False") +
                   ", iterations=" + std::to_string(r.iterations_used) +
                   ", final_dist=" + std::to_string(r.distance_to_base) + ")";
        });
    
    // FabrikBackward class
    pybind11::class_<delta::FabrikBackward>(m, "FabrikBackward")
        .def_static("iterate_to_target",
                   &delta::FabrikBackward::iterate_to_target,
                   "initial_chain"_a, "target_position"_a, 
                   "tolerance"_a = delta::FABRIK_TOLERANCE,
                   "max_iterations"_a = delta::FABRIK_MAX_ITERATIONS,
                   "Perform complete backward iteration to target")
        .def_static("single_backward_iteration",
                   &delta::FabrikBackward::single_backward_iteration,
                   "chain"_a, "target_position"_a,
                   "Perform single backward iteration step")
        .def_static("is_target_reachable",
                   &delta::FabrikBackward::is_target_reachable,
                   "chain"_a, "target_position"_a,
                   "Check if target position is reachable")
        .def_static("calculate_distance_to_base",
                   &delta::FabrikBackward::calculate_distance_to_base,
                   "chain"_a,
                   "Calculate distance from end-effector to base")
        .def_static("get_end_effector_position",
                   &delta::FabrikBackward::get_end_effector_position,
                   "chain"_a,
                   "Get current end-effector position from chain");
    
    // Expose solver constants
    m.attr("FABRIK_TOLERANCE") = delta::FABRIK_TOLERANCE;
    m.attr("FABRIK_MAX_ITERATIONS") = delta::FABRIK_MAX_ITERATIONS;
}