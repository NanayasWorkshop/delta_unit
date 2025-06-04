#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "fabrik_forward.hpp"

using namespace pybind11::literals;

PYBIND11_MODULE(fabrik_forward, m) {
    m.doc() = "Delta robot FABRIK forward iteration module";
    
    // NO Vector3/FabrikChain registration - assumes delta_types and fabrik_initialization are imported
    
    // SegmentDirectionPair structure
    pybind11::class_<delta::SegmentDirectionPair>(m, "SegmentDirectionPair")
        .def_readonly("reference_direction", &delta::SegmentDirectionPair::reference_direction)
        .def_readonly("target_direction", &delta::SegmentDirectionPair::target_direction)
        .def_readonly("segment_index", &delta::SegmentDirectionPair::segment_index)
        .def("__repr__", [](const delta::SegmentDirectionPair& p) {
            return "SegmentDirectionPair(segment=" + std::to_string(p.segment_index) + 
                   ", ref=(" + std::to_string(p.reference_direction.x()) + "," + 
                   std::to_string(p.reference_direction.y()) + "," + 
                   std::to_string(p.reference_direction.z()) + 
                   "), target=(" + std::to_string(p.target_direction.x()) + "," + 
                   std::to_string(p.target_direction.y()) + "," + 
                   std::to_string(p.target_direction.z()) + "))";
        });
    
    // SegmentProperties structure
    pybind11::class_<delta::SegmentProperties>(m, "SegmentProperties")
        .def_readonly("prismatic_length", &delta::SegmentProperties::prismatic_length)
        .def_readonly("h_to_g_distance", &delta::SegmentProperties::h_to_g_distance)
        .def_readonly("fabrik_segment_length", &delta::SegmentProperties::fabrik_segment_length)
        .def_readonly("transformed_direction", &delta::SegmentProperties::transformed_direction)
        .def("__repr__", [](const delta::SegmentProperties& p) {
            return "SegmentProperties(prismatic=" + std::to_string(p.prismatic_length) + 
                   ", h_to_g=" + std::to_string(p.h_to_g_distance) + 
                   ", fabrik_len=" + std::to_string(p.fabrik_segment_length) + ")";
        });
    
    // FabrikForwardResult structure
    pybind11::class_<delta::FabrikForwardResult>(m, "FabrikForwardResult")
        .def_readonly("updated_chain", &delta::FabrikForwardResult::updated_chain)
        .def_readonly("base_position", &delta::FabrikForwardResult::base_position)
        .def_readonly("final_end_effector", &delta::FabrikForwardResult::final_end_effector)
        .def_readonly("constraints_satisfied", &delta::FabrikForwardResult::constraints_satisfied)
        .def_readonly("iterations_used", &delta::FabrikForwardResult::iterations_used)
        .def_readonly("iteration_history", &delta::FabrikForwardResult::iteration_history)
        .def_readonly("recalculated_lengths", &delta::FabrikForwardResult::recalculated_lengths)
        .def("__repr__", [](const delta::FabrikForwardResult& r) {
            return "FabrikForwardResult(base=(" + 
                   std::to_string(r.base_position.x()) + "," +
                   std::to_string(r.base_position.y()) + "," + 
                   std::to_string(r.base_position.z()) + 
                   "), constraints_ok=" + (r.constraints_satisfied ? "True" : "False") +
                   ", iterations=" + std::to_string(r.iterations_used) + ")";
        });
    
    // FabrikForward class
    pybind11::class_<delta::FabrikForward>(m, "FabrikForward")
        .def_static("iterate_from_base",
                   &delta::FabrikForward::iterate_from_base,
                   "backward_result_chain"_a, 
                   "tolerance"_a = delta::FABRIK_TOLERANCE,
                   "max_iterations"_a = delta::FABRIK_MAX_ITERATIONS,
                   "Perform complete forward iteration from base")
        .def_static("single_forward_iteration",
                   &delta::FabrikForward::single_forward_iteration,
                   "chain"_a, "target_segment_lengths"_a,
                   "Perform single forward iteration step")
        .def_static("calculate_new_segment_lengths",
                   &delta::FabrikForward::calculate_new_segment_lengths,
                   "backward_result"_a,
                   "Calculate new segment lengths from backward result")
        .def_static("get_base_position",
                   &delta::FabrikForward::get_base_position,
                   "chain"_a,
                   "Get base position from chain")
        .def_static("is_base_at_origin",
                   &delta::FabrikForward::is_base_at_origin,
                   "chain"_a, "tolerance"_a = 0.01,
                   "Check if base is at origin");
    
    // Utility namespace functions
    m.def("get_direction_pairs_count", &delta::fabrik_forward_utils::get_direction_pairs_count,
          "num_robot_segments"_a, "Get number of direction pairs for N segments");
    
    m.def("get_fabrik_segment_indices", &delta::fabrik_forward_utils::get_fabrik_segment_indices,
          "num_robot_segments"_a, "Get FABRIK segment indices for N segments");
    
    m.def("calculate_h_to_g_distance", &delta::fabrik_forward_utils::calculate_h_to_g_distance,
          "prismatic_length"_a, "Calculate H→G distance from prismatic length");
    
    m.def("physical_to_fabrik_lengths", &delta::fabrik_forward_utils::physical_to_fabrik_lengths,
          "h_to_g_distances"_a, "Convert physical segments to FABRIK lengths");
    
    // REMOVED: Redundant constant exposure - rely on delta_types module instead
    // Constants are now centralized in delta_types_bindings.cpp
}