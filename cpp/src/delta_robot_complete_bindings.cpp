// consolidated_bindings.cpp - ALL modules in one file with COMPLETE COLLISION PIPELINE + SEGMENT CALCULATOR
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
#include "segment_calculator.hpp"  // NEW: Separated segment calculator
#include "u_points_extractor.hpp"
#include "collision_detector.hpp"
#include "waypoint_converter.hpp"
#include "collision_aware_solver.hpp"

using namespace pybind11::literals;

PYBIND11_MODULE(delta_robot_complete, m) {
    m.doc() = "Complete Delta Robot Module - All functionality including collision-aware solving + optimized segment calculator";
    
    // =============================================================================
    // TYPES & ENUMS
    // =============================================================================
    
    pybind11::enum_<delta::JointType>(m, "JointType")
        .value("FIXED_BASE", delta::JointType::FIXED_BASE)
        .value("SPHERICAL_120", delta::JointType::SPHERICAL_120)
        .value("END_EFFECTOR", delta::JointType::END_EFFECTOR);
    
    // =============================================================================
    // CORE STRUCTURES
    // =============================================================================
    
    pybind11::class_<delta::FabrikJoint>(m, "FabrikJoint")
        .def_readwrite("position", &delta::FabrikJoint::position)
        .def_readwrite("type", &delta::FabrikJoint::type);
    
    pybind11::class_<delta::FabrikChain>(m, "FabrikChain")
        .def_readwrite("joints", &delta::FabrikChain::joints)
        .def_readwrite("segments", &delta::FabrikChain::segments)
        .def_readwrite("num_robot_segments", &delta::FabrikChain::num_robot_segments);
    
    // =============================================================================
    // SEGMENT CALCULATOR STRUCTURES (NEW)
    // =============================================================================
    
    pybind11::class_<delta::SegmentEndEffectorData>(m, "SegmentEndEffectorData")
        .def_readonly("segment_number", &delta::SegmentEndEffectorData::segment_number)
        .def_readonly("end_effector_position", &delta::SegmentEndEffectorData::end_effector_position)
        .def_readonly("direction_from_base", &delta::SegmentEndEffectorData::direction_from_base)
        .def_readonly("prismatic_length", &delta::SegmentEndEffectorData::prismatic_length)
        .def_readonly("h_to_g_distance", &delta::SegmentEndEffectorData::h_to_g_distance)
        .def_readonly("fabrik_distance_from_base", &delta::SegmentEndEffectorData::fabrik_distance_from_base);
    
    pybind11::class_<delta::SegmentCalculationResult>(m, "SegmentCalculationResult")
        .def_readonly("segment_end_effectors", &delta::SegmentCalculationResult::segment_end_effectors)
        .def_readonly("calculation_time_ms", &delta::SegmentCalculationResult::calculation_time_ms)
        .def_readonly("calculation_successful", &delta::SegmentCalculationResult::calculation_successful);
    
    // =============================================================================
    // COLLISION STRUCTURES
    // =============================================================================
    
    pybind11::class_<delta::Obstacle>(m, "Obstacle")
        .def(pybind11::init<const delta::Vector3&, double>())
        .def_readwrite("center", &delta::Obstacle::center)
        .def_readwrite("radius", &delta::Obstacle::radius);
    
    pybind11::class_<delta::CollisionResult>(m, "CollisionResult")
        .def_readonly("has_collision", &delta::CollisionResult::has_collision)
        .def_readonly("waypoints", &delta::CollisionResult::waypoints)
        .def_readonly("min_distance", &delta::CollisionResult::min_distance)
        .def_readonly("collision_points", &delta::CollisionResult::collision_points)
        .def_readonly("computation_time", &delta::CollisionResult::computation_time);
    
    pybind11::class_<delta::WaypointConversionResult>(m, "WaypointConversionResult")
        .def_readonly("joint_positions", &delta::WaypointConversionResult::joint_positions)
        .def_readonly("segment_lengths", &delta::WaypointConversionResult::segment_lengths)
        .def_readonly("joint_angles_deg", &delta::WaypointConversionResult::joint_angles_deg)
        .def_readonly("total_reach", &delta::WaypointConversionResult::total_reach)
        .def_readonly("conversion_successful", &delta::WaypointConversionResult::conversion_successful);
    
    // Collision-aware solver structures
    pybind11::class_<delta::CollisionAwareSolutionResult>(m, "CollisionAwareSolutionResult")
        .def_readonly("fabrik_result", &delta::CollisionAwareSolutionResult::fabrik_result)
        .def_readonly("collision_free", &delta::CollisionAwareSolutionResult::collision_free)
        .def_readonly("collision_iterations", &delta::CollisionAwareSolutionResult::collision_iterations)
        .def_readonly("total_collision_time_ms", &delta::CollisionAwareSolutionResult::total_collision_time_ms)
        .def_readonly("collision_history", &delta::CollisionAwareSolutionResult::collision_history)
        .def_readonly("conversion_successful", &delta::CollisionAwareSolutionResult::conversion_successful);
    
    pybind11::class_<delta::CollisionAwareConfig>(m, "CollisionAwareConfig")
        .def(pybind11::init<>())
        .def_readwrite("max_collision_iterations", &delta::CollisionAwareConfig::max_collision_iterations)
        .def_readwrite("spline_diameter", &delta::CollisionAwareConfig::spline_diameter)
        .def_readwrite("enable_collision_detection", &delta::CollisionAwareConfig::enable_collision_detection)
        .def_readwrite("verbose_logging", &delta::CollisionAwareConfig::verbose_logging);
    
    // =============================================================================
    // RESULT TYPES (Updated with timing and validation)
    // =============================================================================
    
    pybind11::class_<delta::FermatResult>(m, "FermatResult")
        .def_readonly("z_A", &delta::FermatResult::z_A)
        .def_readonly("z_B", &delta::FermatResult::z_B)
        .def_readonly("z_C", &delta::FermatResult::z_C)
        .def_readonly("fermat_point", &delta::FermatResult::fermat_point)
        .def_readonly("computation_time_ms", &delta::FermatResult::computation_time_ms)
        .def_readonly("calculation_successful", &delta::FermatResult::calculation_successful);
    
    pybind11::class_<delta::KinematicsResult>(m, "KinematicsResult")
        .def_readonly("end_effector_position", &delta::KinematicsResult::end_effector_position)
        .def_readonly("prismatic_joint_length", &delta::KinematicsResult::prismatic_joint_length);
    
    pybind11::class_<delta::JointStateResult>(m, "JointStateResult")
        .def_readonly("prismatic_joint", &delta::JointStateResult::prismatic_joint)
        .def_readonly("roll_joint", &delta::JointStateResult::roll_joint)
        .def_readonly("pitch_joint", &delta::JointStateResult::pitch_joint)
        .def_readonly("computation_time_ms", &delta::JointStateResult::computation_time_ms)
        .def_readonly("calculation_successful", &delta::JointStateResult::calculation_successful);
    
    pybind11::class_<delta::OrientationResult>(m, "OrientationResult")
        .def_readonly("transformation_matrix", &delta::OrientationResult::transformation_matrix)
        .def_readonly("end_effector_position", &delta::OrientationResult::end_effector_position);
    
    pybind11::class_<delta::FabrikSolutionResult>(m, "FabrikSolutionResult")
        .def_readonly("final_chain", &delta::FabrikSolutionResult::final_chain)
        .def_readonly("converged", &delta::FabrikSolutionResult::converged)
        .def_readonly("final_error", &delta::FabrikSolutionResult::final_error)
        .def_readonly("solve_time_ms", &delta::FabrikSolutionResult::solve_time_ms);
    
    pybind11::class_<delta::MotorResult>(m, "MotorResult")
        .def_readonly("target_position", &delta::MotorResult::target_position)
        .def_readonly("fabrik_converged", &delta::MotorResult::fabrik_converged)
        .def_readonly("fabrik_error", &delta::MotorResult::fabrik_error)
        .def_readonly("solve_time_ms", &delta::MotorResult::solve_time_ms)
        .def_readonly("original_segment_numbers", &delta::MotorResult::original_segment_numbers)
        .def_readonly("original_segment_positions", &delta::MotorResult::original_segment_positions)
        .def_readonly("fabrik_joint_positions", &delta::MotorResult::fabrik_joint_positions)
        .def_readonly("levels", &delta::MotorResult::levels)
        .def_readonly("segment_calculation_time_ms", &delta::MotorResult::segment_calculation_time_ms);  // NEW
    
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
    // KINEMATICS MODULES (Updated with new solver names)
    // =============================================================================
    
    // Main classes with all functionality
    pybind11::class_<delta::FermatSolver>(m, "FermatSolver")
        .def_static("calculate", 
                   pybind11::overload_cast<double, double, double>(&delta::FermatSolver::calculate))
        .def_static("calculate", 
                   pybind11::overload_cast<const delta::Vector3&>(&delta::FermatSolver::calculate))
        .def_static("is_direction_valid", &delta::FermatSolver::is_direction_valid);
    
    pybind11::class_<delta::JointStateSolver>(m, "JointStateSolver")
        .def_static("calculate", &delta::JointStateSolver::calculate)
        .def_static("calculate_from_fermat", &delta::JointStateSolver::calculate_from_fermat)
        .def_static("is_input_valid", &delta::JointStateSolver::is_input_valid);
    
    // Backward compatibility aliases - same functionality, different names
    m.attr("FermatModule") = m.attr("FermatSolver");
    m.attr("JointStateModule") = m.attr("JointStateSolver");
    
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
        .def_static("solve", &delta::FabrikSolver::solve,
                   "Solve FABRIK inverse kinematics",
                   "initial_chain"_a, "target_position"_a, 
                   "tolerance"_a = delta::FABRIK_TOLERANCE, 
                   "max_iterations"_a = delta::FABRIK_MAX_ITERATIONS);
    
    // =============================================================================
    // SEGMENT CALCULATOR (NEW - Separated from FABRIK for Performance)
    // =============================================================================
    
    pybind11::class_<delta::SegmentCalculator>(m, "SegmentCalculator")
        .def_static("calculate_segment_end_effectors", 
                   pybind11::overload_cast<const delta::FabrikChain&>(&delta::SegmentCalculator::calculate_segment_end_effectors),
                   "Calculate segment end-effector positions from collision-free FABRIK chain",
                   "collision_free_chain"_a)
        .def_static("calculate_segment_end_effectors", 
                   pybind11::overload_cast<const delta::FabrikChain&, int>(&delta::SegmentCalculator::calculate_segment_end_effectors),
                   "Calculate segment end-effectors with custom number of robot segments",
                   "collision_free_chain"_a, "num_robot_segments"_a)
        .def_static("is_chain_valid_for_segments", &delta::SegmentCalculator::is_chain_valid_for_segments,
                   "Validate that a FABRIK chain is suitable for segment calculations",
                   "chain"_a, "min_segments"_a = 1)
        .def_static("estimate_calculation_time", &delta::SegmentCalculator::estimate_calculation_time,
                   "Get the total computational cost estimate for segment calculations",
                   "num_robot_segments"_a);
    
    // =============================================================================
    // COLLISION MODULES (Complete Pipeline)
    // =============================================================================
    
    pybind11::class_<delta::UPointsExtractor>(m, "UPointsExtractor")
        .def_static("extract_u_points", &delta::UPointsExtractor::extract_u_points)
        .def_static("extract_u_points_from_positions", &delta::UPointsExtractor::extract_u_points_from_positions);
    
    pybind11::class_<delta::CollisionDetector>(m, "CollisionDetector")
        .def_static("check_and_avoid", &delta::CollisionDetector::check_and_avoid)
        .def_static("create_test_obstacles", &delta::CollisionDetector::create_test_obstacles);
    
    pybind11::class_<delta::WaypointConverter>(m, "WaypointConverter")
        .def_static("convert_waypoints_to_joints", &delta::WaypointConverter::convert_waypoints_to_joints)
        .def_static("create_fabrik_chain_from_waypoints", &delta::WaypointConverter::create_fabrik_chain_from_waypoints)
        .def_static("validate_waypoints", &delta::WaypointConverter::validate_waypoints);
    
    // Complete collision-aware solver
    pybind11::class_<delta::CollisionAwareSolver>(m, "CollisionAwareSolver")
        .def_static("solve_with_collision_avoidance", 
                   pybind11::overload_cast<const delta::Vector3&, const std::vector<delta::Obstacle>&, const delta::CollisionAwareConfig&>
                   (&delta::CollisionAwareSolver::solve_with_collision_avoidance),
                   "Solve with collision avoidance using default initialization",
                   "target_position"_a, "obstacles"_a, "config"_a = delta::CollisionAwareConfig())
        .def_static("solve_with_collision_avoidance", 
                   pybind11::overload_cast<const delta::Vector3&, const std::vector<delta::Obstacle>&, 
                                          const std::vector<delta::Vector3>&, int, const delta::CollisionAwareConfig&>
                   (&delta::CollisionAwareSolver::solve_with_collision_avoidance),
                   "Solve with collision avoidance using provided initial joint positions",
                   "target_position"_a, "obstacles"_a, "initial_joint_positions"_a, "num_robot_segments"_a, "config"_a = delta::CollisionAwareConfig())
        .def_static("solve", &delta::CollisionAwareSolver::solve,
                   "Simple interface for collision-aware solving",
                   "target_position"_a, "obstacles"_a, "max_iterations"_a = 3)
        .def_static("has_collision", &delta::CollisionAwareSolver::has_collision,
                   "Check if a FABRIK chain has collisions with obstacles",
                   "chain"_a, "obstacles"_a, "spline_diameter"_a = delta::DEFAULT_SPLINE_DIAMETER);
    
    // =============================================================================
    // MOTOR MODULE
    // =============================================================================
    
    pybind11::class_<delta::MotorModule>(m, "MotorModule")
        .def_static("calculate_motors", 
                   pybind11::overload_cast<double, double, double>(&delta::MotorModule::calculate_motors))
        .def_static("calculate_motors", 
                   pybind11::overload_cast<const delta::Vector3&>(&delta::MotorModule::calculate_motors))
        .def_static("calculate_motors", 
                   pybind11::overload_cast<double, double, double, const std::optional<std::vector<delta::Vector3>>&>(&delta::MotorModule::calculate_motors))
        .def_static("calculate_motors", 
                   pybind11::overload_cast<const delta::Vector3&, const std::optional<std::vector<delta::Vector3>>&>(&delta::MotorModule::calculate_motors));
    
    // =============================================================================
    // CONVENIENCE FUNCTIONS
    // =============================================================================
    
    m.def("solve_delta_robot", &delta::fabrik_utils::solve_delta_robot);
    m.def("calculate_motors", [](double x, double y, double z) {
        return delta::MotorModule::calculate_motors(x, y, z);
    });
    
    // Collision-aware convenience functions
    m.def("solve_with_collision_avoidance", [](double x, double y, double z, const std::vector<delta::Obstacle>& obstacles) {
        delta::Vector3 target(x, y, z);
        return delta::CollisionAwareSolver::solve_with_collision_avoidance(target, obstacles);
    }, "Convenience function for collision-aware solving with target coordinates");
    
    // NEW: Segment calculator convenience function
    m.def("calculate_segment_end_effectors", [](const delta::FabrikChain& chain) {
        return delta::SegmentCalculator::calculate_segment_end_effectors(chain);
    }, "Convenience function for calculating segment end-effectors");
    
    // =============================================================================
    // CONSTANTS
    // =============================================================================
    
    m.attr("FABRIK_TOLERANCE") = delta::FABRIK_TOLERANCE;
    m.attr("DEFAULT_ROBOT_SEGMENTS") = delta::DEFAULT_ROBOT_SEGMENTS;
    m.attr("DEFAULT_SPLINE_DIAMETER") = delta::DEFAULT_SPLINE_DIAMETER;
}