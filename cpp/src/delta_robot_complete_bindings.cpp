// consolidated_bindings.cpp - ALL modules in one file with COMPLETE COLLISION PIPELINE + SEGMENT CALCULATOR + STEP 1.2 IMPROVEMENTS
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
#include "kinematics_module.hpp"      // CORRECTED: Use existing header name
#include "orientation_module.hpp"     // CORRECTED: Use existing header name
#include "motor_module.hpp"
#include "segment_calculator.hpp"
#include "u_points_extractor.hpp"
#include "collision_detector.hpp"
#include "waypoint_converter.hpp"
#include "collision_aware_solver.hpp"

using namespace pybind11::literals;

PYBIND11_MODULE(delta_robot_complete, m) {
    m.doc() = "Complete Delta Robot Module - Step 1.2: Level 1 Composite Modules with Timing and Validation";
    
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
    // SEGMENT CALCULATOR STRUCTURES
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
    // COORDINATE FRAME STRUCTURE (NEW - Step 1.2)
    // =============================================================================
    
    pybind11::class_<delta::CoordinateFrame>(m, "CoordinateFrame")
        .def_readonly("origin", &delta::CoordinateFrame::origin)
        .def_readonly("u_axis", &delta::CoordinateFrame::u_axis)
        .def_readonly("v_axis", &delta::CoordinateFrame::v_axis)
        .def_readonly("w_axis", &delta::CoordinateFrame::w_axis);
    
    // =============================================================================
    // RESULT TYPES (Updated with Step 1.2 improvements)
    // =============================================================================
    
    // Level 0 Results (Step 1.1 - Enhanced with timing and validation)
    pybind11::class_<delta::FermatResult>(m, "FermatResult")
        .def_readonly("z_A", &delta::FermatResult::z_A)
        .def_readonly("z_B", &delta::FermatResult::z_B)
        .def_readonly("z_C", &delta::FermatResult::z_C)
        .def_readonly("fermat_point", &delta::FermatResult::fermat_point)
        .def_readonly("computation_time_ms", &delta::FermatResult::computation_time_ms)
        .def_readonly("calculation_successful", &delta::FermatResult::calculation_successful)
        .def_static("failed", &delta::FermatResult::failed,
                   "Create failed result for error handling", "time_ms"_a = 0.0);
    
    pybind11::class_<delta::JointStateResult>(m, "JointStateResult")
        .def_readonly("prismatic_joint", &delta::JointStateResult::prismatic_joint)
        .def_readonly("roll_joint", &delta::JointStateResult::roll_joint)
        .def_readonly("pitch_joint", &delta::JointStateResult::pitch_joint)
        .def_readonly("direction_vector", &delta::JointStateResult::direction_vector)
        .def_readonly("fermat_point", &delta::JointStateResult::fermat_point)
        .def_readonly("z_A", &delta::JointStateResult::z_A)
        .def_readonly("z_B", &delta::JointStateResult::z_B)
        .def_readonly("z_C", &delta::JointStateResult::z_C)
        .def_readonly("computation_time_ms", &delta::JointStateResult::computation_time_ms)
        .def_readonly("calculation_successful", &delta::JointStateResult::calculation_successful)
        .def_static("failed", &delta::JointStateResult::failed,
                   "Create failed result for error handling", 
                   "direction"_a = delta::Vector3(0,0,0), "time_ms"_a = 0.0);
    
    // Level 1 Results (Step 1.2 - Enhanced with timing and validation)
    pybind11::class_<delta::KinematicsResult>(m, "KinematicsResult")
        .def_readonly("end_effector_position", &delta::KinematicsResult::end_effector_position)
        .def_readonly("prismatic_joint_length", &delta::KinematicsResult::prismatic_joint_length)
        .def_readonly("transformed_vector", &delta::KinematicsResult::transformed_vector)
        .def_readonly("original_input", &delta::KinematicsResult::original_input)
        .def_readonly("input_angle_from_z", &delta::KinematicsResult::input_angle_from_z)
        .def_readonly("fermat_data", &delta::KinematicsResult::fermat_data)
        .def_readonly("joint_state_data", &delta::KinematicsResult::joint_state_data)
        .def_readonly("computation_time_ms", &delta::KinematicsResult::computation_time_ms)
        .def_readonly("calculation_successful", &delta::KinematicsResult::calculation_successful)
        .def_static("failed", &delta::KinematicsResult::failed,
                   "Create failed result for error handling",
                   "input"_a = delta::Vector3(0,0,0), "time_ms"_a = 0.0);
    
    pybind11::class_<delta::OrientationResult>(m, "OrientationResult")
        .def_readonly("transformation_matrix", &delta::OrientationResult::transformation_matrix)
        .def_readonly("fermat_point", &delta::OrientationResult::fermat_point)
        .def_readonly("end_effector_position", &delta::OrientationResult::end_effector_position)
        .def_readonly("UVW_at_fermat", &delta::OrientationResult::UVW_at_fermat)
        .def_readonly("IJK_mirrored", &delta::OrientationResult::IJK_mirrored)
        .def_readonly("UVW_prime_aligned", &delta::OrientationResult::UVW_prime_aligned)
        .def_readonly("final_frame", &delta::OrientationResult::final_frame)
        .def_readonly("computation_time_ms", &delta::OrientationResult::computation_time_ms)
        .def_readonly("calculation_successful", &delta::OrientationResult::calculation_successful)
        .def_static("failed", &delta::OrientationResult::failed,
                   "Create failed result for error handling",
                   "input"_a = delta::Vector3(0,0,0), "time_ms"_a = 0.0);
    
    // Other result types
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
        .def_readonly("segment_calculation_time_ms", &delta::MotorResult::segment_calculation_time_ms);
    
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
    // LEVEL 0 MODULES (Step 1.1 - Enhanced with timing and validation)
    // =============================================================================
    
    pybind11::class_<delta::FermatSolver>(m, "FermatSolver")
        .def_static("calculate", 
                   pybind11::overload_cast<double, double, double>(&delta::FermatSolver::calculate),
                   "Calculate Fermat point from coordinates with timing and validation",
                   "x"_a, "y"_a, "z"_a)
        .def_static("calculate", 
                   pybind11::overload_cast<const delta::Vector3&>(&delta::FermatSolver::calculate),
                   "Calculate Fermat point from direction vector with timing and validation",
                   "direction"_a)
        .def_static("is_direction_valid", &delta::FermatSolver::is_direction_valid,
                   "Validate direction vector", "direction"_a)
        .def_static("get_base_A", &delta::FermatSolver::get_base_A)
        .def_static("get_base_B", &delta::FermatSolver::get_base_B)
        .def_static("get_base_C", &delta::FermatSolver::get_base_C);
    
    pybind11::class_<delta::JointStateSolver>(m, "JointStateSolver")
        .def_static("calculate", &delta::JointStateSolver::calculate,
                   "Calculate joint states from components with timing and validation",
                   "direction_vector"_a, "fermat_point"_a, "z_A"_a, "z_B"_a, "z_C"_a)
        .def_static("calculate_from_fermat", &delta::JointStateSolver::calculate_from_fermat,
                   "Calculate joint states from Fermat result with timing and validation",
                   "direction_vector"_a, "fermat_result"_a)
        .def_static("is_input_valid", &delta::JointStateSolver::is_input_valid,
                   "Validate input parameters", "direction_vector"_a, "fermat_point"_a);
    
    // =============================================================================
    // LEVEL 1 COMPOSITE MODULES (UPDATED - Step 1.2)
    // =============================================================================
    
    pybind11::class_<delta::KinematicsModule>(m, "KinematicsModule")
        .def_static("calculate", 
                   pybind11::overload_cast<double, double, double>(&delta::KinematicsModule::calculate),
                   "Calculate kinematics from coordinates with timing and validation",
                   "x"_a, "y"_a, "z"_a)
        .def_static("calculate", 
                   pybind11::overload_cast<const delta::Vector3&>(&delta::KinematicsModule::calculate),
                   "Calculate kinematics from direction vector with timing and validation", 
                   "input_vector"_a)
        .def_static("is_input_valid", &delta::KinematicsModule::is_input_valid,
                   "Validate input direction vector", "input_vector"_a)
        .def_static("set_use_half_angle_transform", &delta::KinematicsModule::set_use_half_angle_transform,
                   "Configure half-angle transformation (future feature)", "enable"_a)
        .def_static("get_use_half_angle_transform", &delta::KinematicsModule::get_use_half_angle_transform,
                   "Get half-angle transformation setting");
    
    pybind11::class_<delta::OrientationModule>(m, "OrientationModule")
        .def_static("calculate", 
                   pybind11::overload_cast<double, double, double>(&delta::OrientationModule::calculate),
                   "Calculate orientation from coordinates with timing and validation",
                   "x"_a, "y"_a, "z"_a)
        .def_static("calculate", 
                   pybind11::overload_cast<const delta::Vector3&>(&delta::OrientationModule::calculate),
                   "Calculate orientation from direction vector with timing and validation",
                   "input_vector"_a)
        .def_static("calculate_from_kinematics", &delta::OrientationModule::calculate_from_kinematics,
                   "Calculate orientation from existing kinematics result (more efficient)",
                   "kinematics_result"_a)
        .def_static("is_input_valid", &delta::OrientationModule::is_input_valid,
                   "Validate input direction vector", "input_vector"_a);
    
    // =============================================================================
    // BACKWARD COMPATIBILITY ALIASES (Step 1.2)
    // =============================================================================
    
    // Step 1.1 backward compatibility
    m.attr("FermatModule") = m.attr("FermatSolver");
    m.attr("JointStateModule") = m.attr("JointStateSolver");
    
    // Step 1.2 backward compatibility - aliases to same classes
    m.attr("KinematicsSolver") = m.attr("KinematicsModule");
    m.attr("OrientationSolver") = m.attr("OrientationModule");
    
    // =============================================================================
    // FABRIK MODULES
    // =============================================================================
    
    pybind11::class_<delta::FabrikInitialization>(m, "FabrikInitialization")
        .def_static("initialize_straight_up", &delta::FabrikInitialization::initialize_straight_up,
                   "Initialize FABRIK chain in straight-up configuration", "num_robot_segments"_a = delta::DEFAULT_ROBOT_SEGMENTS)
        .def_static("initialize_from_joint_positions", &delta::FabrikInitialization::initialize_from_joint_positions,
                   "Initialize FABRIK chain from joint positions", "num_robot_segments"_a, "joint_positions"_a);
    
    pybind11::class_<delta::FabrikSolver>(m, "FabrikSolver")
        .def_static("solve", &delta::FabrikSolver::solve,
                   "Solve FABRIK inverse kinematics",
                   "initial_chain"_a, "target_position"_a, 
                   "tolerance"_a = delta::FABRIK_TOLERANCE, 
                   "max_iterations"_a = delta::FABRIK_MAX_ITERATIONS);
    
    // =============================================================================
    // SEGMENT CALCULATOR (Separated for performance)
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
        .def_static("extract_u_points", &delta::UPointsExtractor::extract_u_points,
                   "Extract U points from FABRIK chain", "solved_chain"_a)
        .def_static("extract_u_points_from_positions", &delta::UPointsExtractor::extract_u_points_from_positions,
                   "Extract U points from joint positions", "joint_positions"_a);
    
    pybind11::class_<delta::CollisionDetector>(m, "CollisionDetector")
        .def_static("check_and_avoid", &delta::CollisionDetector::check_and_avoid,
                   "Check collision and get avoidance waypoints",
                   "u_points"_a, "obstacles"_a, "spline_diameter"_a = delta::DEFAULT_SPLINE_DIAMETER)
        .def_static("create_test_obstacles", &delta::CollisionDetector::create_test_obstacles,
                   "Create test obstacles for collision detection");
    
    pybind11::class_<delta::WaypointConverter>(m, "WaypointConverter")
        .def_static("convert_waypoints_to_joints", &delta::WaypointConverter::convert_waypoints_to_joints,
                   "Convert collision-free waypoints to joint positions", "waypoints"_a)
        .def_static("create_fabrik_chain_from_waypoints", &delta::WaypointConverter::create_fabrik_chain_from_waypoints,
                   "Create FABRIK chain from waypoints", "waypoints"_a, "num_robot_segments"_a)
        .def_static("validate_waypoints", &delta::WaypointConverter::validate_waypoints,
                   "Validate waypoints for conversion", "waypoints"_a);
    
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
                   pybind11::overload_cast<double, double, double>(&delta::MotorModule::calculate_motors),
                   "Calculate motor positions from target coordinates", "target_x"_a, "target_y"_a, "target_z"_a)
        .def_static("calculate_motors", 
                   pybind11::overload_cast<const delta::Vector3&>(&delta::MotorModule::calculate_motors),
                   "Calculate motor positions from target vector", "target_position"_a)
        .def_static("calculate_motors", 
                   pybind11::overload_cast<double, double, double, const std::optional<std::vector<delta::Vector3>>&>(&delta::MotorModule::calculate_motors),
                   "Calculate motor positions with optional initial joint positions",
                   "target_x"_a, "target_y"_a, "target_z"_a, "current_joint_positions"_a)
        .def_static("calculate_motors", 
                   pybind11::overload_cast<const delta::Vector3&, const std::optional<std::vector<delta::Vector3>>&>(&delta::MotorModule::calculate_motors),
                   "Calculate motor positions with optional initial joint positions",
                   "target_position"_a, "current_joint_positions"_a);
    
    // =============================================================================
    // CONVENIENCE FUNCTIONS
    // =============================================================================
    
    m.def("solve_delta_robot", &delta::fabrik_utils::solve_delta_robot,
          "Solve delta robot kinematics", "num_segments"_a, "target"_a, "tolerance"_a = delta::FABRIK_TOLERANCE);
    
    m.def("calculate_motors", [](double x, double y, double z) {
        return delta::MotorModule::calculate_motors(x, y, z);
    }, "Convenience function for motor calculation", "x"_a, "y"_a, "z"_a);
    
    // Collision-aware convenience functions
    m.def("solve_with_collision_avoidance", [](double x, double y, double z, const std::vector<delta::Obstacle>& obstacles) {
        delta::Vector3 target(x, y, z);
        return delta::CollisionAwareSolver::solve_with_collision_avoidance(target, obstacles);
    }, "Convenience function for collision-aware solving with target coordinates",
       "x"_a, "y"_a, "z"_a, "obstacles"_a);
    
    // Segment calculator convenience function
    m.def("calculate_segment_end_effectors", [](const delta::FabrikChain& chain) {
        return delta::SegmentCalculator::calculate_segment_end_effectors(chain);
    }, "Convenience function for calculating segment end-effectors", "chain"_a);
    
    // =============================================================================
    // CONSTANTS
    // =============================================================================
    
    m.attr("FABRIK_TOLERANCE") = delta::FABRIK_TOLERANCE;
    m.attr("DEFAULT_ROBOT_SEGMENTS") = delta::DEFAULT_ROBOT_SEGMENTS;
    m.attr("DEFAULT_SPLINE_DIAMETER") = delta::DEFAULT_SPLINE_DIAMETER;
}