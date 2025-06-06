#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "fabrik_initialization.hpp"
#include "fabrik_backward.hpp"
#include "fabrik_forward.hpp"
#include "fabrik_solver.hpp"

using namespace pybind11::literals;

PYBIND11_MODULE(fabrik_complete, m) {
    m.doc() = "Delta robot complete FABRIK module - Consolidated initialization, backward, forward, and solver";
    
    // =============================================================================
    // FABRIK INITIALIZATION SUBMODULE
    // =============================================================================
    
    // JointType enum
    pybind11::enum_<delta::JointType>(m, "JointType")
        .value("FIXED_BASE", delta::JointType::FIXED_BASE)
        .value("SPHERICAL_120", delta::JointType::SPHERICAL_120)
        .value("END_EFFECTOR", delta::JointType::END_EFFECTOR);
    
    // FabrikJoint structure
    pybind11::class_<delta::FabrikJoint>(m, "FabrikJoint")
        .def(pybind11::init<const delta::Vector3&, delta::JointType, double>(),
             "position"_a, "joint_type"_a, "constraint"_a = 0.0)
        .def_readwrite("position", &delta::FabrikJoint::position)
        .def_readwrite("type", &delta::FabrikJoint::type)
        .def_readwrite("constraint_angle", &delta::FabrikJoint::constraint_angle)
        .def("__repr__", [](const delta::FabrikJoint& j) {
            std::string type_str;
            switch(j.type) {
                case delta::JointType::FIXED_BASE: type_str = "FIXED_BASE"; break;
                case delta::JointType::SPHERICAL_120: type_str = "SPHERICAL_120"; break;
                case delta::JointType::END_EFFECTOR: type_str = "END_EFFECTOR"; break;
            }
            return "FabrikJoint(pos=(" + std::to_string(j.position.x()) + "," + 
                   std::to_string(j.position.y()) + "," + std::to_string(j.position.z()) + 
                   "), type=" + type_str + ", constraint=" + std::to_string(j.constraint_angle) + ")";
        });
    
    // FabrikSegment structure
    pybind11::class_<delta::FabrikSegment>(m, "FabrikSegment")
        .def(pybind11::init<int, int, double>())
        .def_readwrite("start_joint_index", &delta::FabrikSegment::start_joint_index)
        .def_readwrite("end_joint_index", &delta::FabrikSegment::end_joint_index)
        .def_readwrite("length", &delta::FabrikSegment::length)
        .def("__repr__", [](const delta::FabrikSegment& s) {
            return "FabrikSegment(start=" + std::to_string(s.start_joint_index) + 
                   ", end=" + std::to_string(s.end_joint_index) + 
                   ", length=" + std::to_string(s.length) + ")";
        });
    
    // FabrikChain structure
    pybind11::class_<delta::FabrikChain>(m, "FabrikChain")
        .def(pybind11::init<int>())
        .def_readwrite("joints", &delta::FabrikChain::joints)
        .def_readwrite("segments", &delta::FabrikChain::segments)
        .def_readwrite("num_robot_segments", &delta::FabrikChain::num_robot_segments)
        .def("__repr__", [](const delta::FabrikChain& c) {
            return "FabrikChain(segments=" + std::to_string(c.num_robot_segments) + 
                   ", joints=" + std::to_string(c.joints.size()) + 
                   ", chain_segments=" + std::to_string(c.segments.size()) + ")";
        });
    
    // FabrikInitResult structure
    pybind11::class_<delta::FabrikInitResult>(m, "FabrikInitResult")
        .def_readonly("chain", &delta::FabrikInitResult::chain)
        .def_readonly("final_end_effector", &delta::FabrikInitResult::final_end_effector)
        .def_readonly("total_reach", &delta::FabrikInitResult::total_reach)
        .def("__repr__", [](const delta::FabrikInitResult& r) {
            return "FabrikInitResult(end_effector=(" + 
                   std::to_string(r.final_end_effector.x()) + "," +
                   std::to_string(r.final_end_effector.y()) + "," + 
                   std::to_string(r.final_end_effector.z()) + 
                   "), total_reach=" + std::to_string(r.total_reach) + 
                   ", joints=" + std::to_string(r.chain.joints.size()) + ")";
        });
    
    // FabrikInitialization class
    pybind11::class_<delta::FabrikInitialization>(m, "FabrikInitialization")
        .def_static("initialize_straight_up", 
                   &delta::FabrikInitialization::initialize_straight_up,
                   "num_robot_segments"_a = delta::DEFAULT_ROBOT_SEGMENTS,
                   "Initialize robot chain in straight up configuration")
        .def_static("initialize_from_joint_positions",
                   &delta::FabrikInitialization::initialize_from_joint_positions,
                   "num_robot_segments"_a, "joint_positions"_a,
                   "Initialize robot chain from provided joint positions")
        .def_static("initialize_with_direction",
                   &delta::FabrikInitialization::initialize_with_direction,
                   "num_robot_segments"_a, "direction"_a,
                   "Initialize robot chain with custom direction")
        .def_static("calculate_segment_length",
                   &delta::FabrikInitialization::calculate_segment_length,
                   "hypotenuse_distance"_a, "angle_rad"_a = 0.0,
                   "Calculate segment length from hypotenuse and angle")
        .def_static("calculate_joint_position",
                   &delta::FabrikInitialization::calculate_joint_position,
                   "robot_segment_index"_a, "joint_index_in_segment"_a,
                   "Calculate position of specific joint")
        .def_static("get_total_reach",
                   &delta::FabrikInitialization::get_total_reach,
                   "num_robot_segments"_a,
                   "Get maximum reach of robot with N segments")
        .def_static("get_total_joints",
                   &delta::FabrikInitialization::get_total_joints,
                   "num_robot_segments"_a,
                   "Get total number of joints for N segments")
        .def_static("validate_joint_positions",
                   &delta::FabrikInitialization::validate_joint_positions,
                   "num_robot_segments"_a, "joint_positions"_a,
                   "Validate joint positions for initialization")
        .def_static("is_base_at_origin",
                   &delta::FabrikInitialization::is_base_at_origin,
                   "base_position"_a, "tolerance"_a = 1e-6,
                   "Check if base position is at origin");

    // =============================================================================
    // FABRIK BACKWARD SUBMODULE
    // =============================================================================
    
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
                   std::to_string(r.target_position.x()) + "," +
                   std::to_string(r.target_position.y()) + "," + 
                   std::to_string(r.target_position.z()) + 
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

    // =============================================================================
    // FABRIK FORWARD SUBMODULE
    // =============================================================================
    
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

    // =============================================================================
    // FABRIK SOLVER SUBMODULE
    // =============================================================================
    
    // SegmentEndEffectorData structure
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
        .def_readonly("segment_end_effectors", &delta::FabrikSolutionResult::segment_end_effectors)
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
}