#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "fabrik_solver.hpp"

using namespace pybind11::literals;

PYBIND11_MODULE(fabrik_solver, m) {
    m.doc() = "Delta robot FABRIK complete solver module";
    
    // NO Vector3/FabrikChain registration - assumes delta_types and fabrik_initialization are imported
    
    // NEW! SegmentEndEffectorData structure
    pybind11::class_<delta::SegmentEndEffectorData>(m, "SegmentEndEffectorData")
        .def_readonly("segment_number", &delta::SegmentEndEffectorData::segment_number)
        .def_readonly("end_effector_position", &delta::SegmentEndEffectorData::end_effector_position)
        .def_readonly("direction_from_base", &delta::SegmentEndEffectorData::direction_from_base)
        .def_readonly("prismatic_length", &delta::SegmentEndEffectorData::prismatic_length)
        .def_readonly("h_to_g_distance", &delta::SegmentEndEffectorData::h_to_g_distance)
        .def_readonly("fabrik_distance_from_base", &delta::SegmentEndEffectorData::fabrik_distance_from_base)
        .def("__repr__", [](const delta::SegmentEndEffectorData& s) {
            return "SegmentEndEffectorData(segment=" + std::to_string(s.segment_number) + 
                   ", pos=(" + std::to_string(s.end_effector_position.x()) + "," + 
                   std::to_string(s.end_effector_position.y()) + "," + 
                   std::to_string(s.end_effector_position.z()) + 
                   "), prismatic=" + std::to_string(s.prismatic_length) + 
                   ", distance=" + std::to_string(s.fabrik_distance_from_base) + ")";
        });
    
    // FabrikSolutionResult structure
    pybind11::class_<delta::FabrikSolutionResult>(m, "FabrikSolutionResult")
        .def_readonly("final_chain", &delta::FabrikSolutionResult::final_chain)
        .def_readonly("target_position", &delta::FabrikSolutionResult::target_position)
        .def_readonly("achieved_position", &delta::FabrikSolutionResult::achieved_position)
        .def_readonly("converged", &delta::FabrikSolutionResult::converged)
        .def_readonly("final_error", &delta::FabrikSolutionResult::final_error)
        .def_readonly("total_iterations", &delta::FabrikSolutionResult::total_iterations)
        .def_readonly("backward_iterations", &delta::FabrikSolutionResult::backward_iterations)
        .def_readonly("forward_iterations", &delta::FabrikSolutionResult::forward_iterations)
        .def_readonly("convergence_history", &delta::FabrikSolutionResult::convergence_history)
        .def_readonly("solve_time_ms", &delta::FabrikSolutionResult::solve_time_ms)
        .def_readonly("segment_end_effectors", &delta::FabrikSolutionResult::segment_end_effectors)  // NEW!
        .def("__repr__", [](const delta::FabrikSolutionResult& r) {
            return "FabrikSolutionResult(target=(" + 
                   std::to_string(r.target_position.x()) + "," +
                   std::to_string(r.target_position.y()) + "," + 
                   std::to_string(r.target_position.z()) + 
                   "), converged=" + (r.converged ? "True" : "False") +
                   ", error=" + std::to_string(r.final_error) +
                   ", total_iterations=" + std::to_string(r.total_iterations) +
                   ", segments=" + std::to_string(r.segment_end_effectors.size()) +
                   ", time=" + std::to_string(r.solve_time_ms) + "ms)";
        });
    
    // FabrikSolverConfig structure
    pybind11::class_<delta::FabrikSolverConfig>(m, "FabrikSolverConfig")
        .def(pybind11::init<>())
        .def_readwrite("tolerance", &delta::FabrikSolverConfig::tolerance)
        .def_readwrite("max_iterations", &delta::FabrikSolverConfig::max_iterations)
        .def_readwrite("max_backward_forward_cycles", &delta::FabrikSolverConfig::max_backward_forward_cycles)
        .def_readwrite("enable_constraints", &delta::FabrikSolverConfig::enable_constraints)
        .def_readwrite("track_convergence_history", &delta::FabrikSolverConfig::track_convergence_history)
        .def_readwrite("verbose_logging", &delta::FabrikSolverConfig::verbose_logging)
        .def("__repr__", [](const delta::FabrikSolverConfig& c) {
            return "FabrikSolverConfig(tolerance=" + std::to_string(c.tolerance) +
                   ", max_iter=" + std::to_string(c.max_iterations) +
                   ", max_cycles=" + std::to_string(c.max_backward_forward_cycles) +
                   ", constraints=" + (c.enable_constraints ? "True" : "False") +
                   ", history=" + (c.track_convergence_history ? "True" : "False") +
                   ", verbose=" + (c.verbose_logging ? "True" : "False") + ")";
        });
    
    // FabrikSolver class
    pybind11::class_<delta::FabrikSolver>(m, "FabrikSolver")
        .def_static("solve_to_target",
                   &delta::FabrikSolver::solve_to_target,
                   "initial_chain"_a, "target_position"_a, "config"_a,
                   "Solve FABRIK to target with configuration")
        .def_static("solve",
                   &delta::FabrikSolver::solve,
                   "initial_chain"_a, "target_position"_a,
                   "tolerance"_a = delta::FABRIK_TOLERANCE,
                   "max_iterations"_a = delta::FABRIK_MAX_ITERATIONS,
                   "Solve FABRIK to target with simple parameters")
        .def_static("single_fabrik_cycle",
                   &delta::FabrikSolver::single_fabrik_cycle,
                   "chain"_a, "target_position"_a, "config"_a,
                   "Perform single backward-forward cycle")
        .def_static("is_solution_valid",
                   &delta::FabrikSolver::is_solution_valid,
                   "chain"_a, "tolerance"_a = 0.01,
                   "Check if solution is valid")
        .def_static("calculate_chain_error",
                   &delta::FabrikSolver::calculate_chain_error,
                   "chain"_a,
                   "Calculate total chain error")
        .def_static("get_end_effector_position",
                   &delta::FabrikSolver::get_end_effector_position,
                   "chain"_a,
                   "Get end-effector position from chain")
        .def_static("create_fast_config",
                   &delta::FabrikSolver::create_fast_config,
                   "Create fast solving configuration")
        .def_static("create_precise_config",
                   &delta::FabrikSolver::create_precise_config,
                   "Create precise solving configuration")
        .def_static("create_debug_config",
                   &delta::FabrikSolver::create_debug_config,
                   "Create debug configuration with verbose logging");
    
    // Utility functions
    m.def("solve_delta_robot", &delta::fabrik_utils::solve_delta_robot,
          "num_segments"_a, "target"_a, "tolerance"_a = delta::FABRIK_TOLERANCE,
          "Convenience function to solve delta robot with N segments");
    
    m.def("solve_multiple_targets", &delta::fabrik_utils::solve_multiple_targets,
          "initial_chain"_a, "targets"_a, "config"_a,
          "Solve multiple targets in batch");
    
    m.def("find_max_reach", &delta::fabrik_utils::find_max_reach,
          "chain"_a, "direction"_a,
          "Find maximum reach in given direction");
    
    m.def("is_target_reachable", &delta::fabrik_utils::is_target_reachable,
          "chain"_a, "target"_a,
          "Check if target is reachable");
    
    // REMOVED: Redundant constant exposure - rely on delta_types module instead
    // Constants are now centralized in delta_types_bindings.cpp
}