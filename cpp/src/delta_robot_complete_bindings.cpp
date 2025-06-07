// delta_robot_complete_bindings.cpp - Fixed bindings with collision avoidance
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

// Include all headers
#include "../fabrik/fabrik_initialization.hpp"
#include "../fabrik/fabrik_backward.hpp"
#include "../fabrik/fabrik_forward.hpp"
#include "../fabrik/fabrik_solver.hpp"
#include "../collision/spline_collision_avoidance.hpp"
#include "../kinematics/fermat_module.hpp"
#include "../kinematics/joint_state.hpp"
#include "../kinematics/kinematics_module.hpp"
#include "../kinematics/orientation_module.hpp"
#include "../motor/motor_module.hpp"

using namespace pybind11::literals;

PYBIND11_MODULE(delta_robot_complete, m) {
    m.doc() = "Complete Delta Robot Module - All functionality in one place";
    
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
        .def_readwrite("segments", &delta::FabrikChain::segments);
    
    // =============================================================================
    // COLLISION AVOIDANCE STRUCTURES
    // =============================================================================
    
    pybind11::class_<delta::Obstacle>(m, "Obstacle")
        .def(pybind11::init<const delta::Vector3&, double>())
        .def(pybind11::init<double, double, double, double>())
        .def_readwrite("center", &delta::Obstacle::center)
        .def_readwrite("radius", &delta::Obstacle::radius);
    
    pybind11::class_<delta::CollisionAvoidanceResult>(m, "CollisionAvoidanceResult")
        .def_readonly("safe_control_points", &delta::CollisionAvoidanceResult::safe_control_points)
        .def_readonly("collision_free", &delta::CollisionAvoidanceResult::collision_free)
        .def_readonly("total_adjustment", &delta::CollisionAvoidanceResult::total_adjustment)
        .def_readonly("execution_time_ms", &delta::CollisionAvoidanceResult::execution_time_ms)
        .def_readonly("solutions_evaluated", &delta::CollisionAvoidanceResult::solutions_evaluated)
        .def_readonly("collision_obstacle_indices", &delta::CollisionAvoidanceResult::collision_obstacle_indices)
        .def_readonly("collision_points", &delta::CollisionAvoidanceResult::collision_points);
    
    // =============================================================================
    // RESULT TYPES
    // =============================================================================
    
    pybind11::class_<delta::FermatResult>(m, "FermatResult")
        .def_readonly("fermat_point", &delta::FermatResult::fermat_point);
    
    pybind11::class_<delta::KinematicsResult>(m, "KinematicsResult")
        .def_readonly("end_effector_position", &delta::KinematicsResult::end_effector_position)
        .def_readonly("prismatic_joint_length", &delta::KinematicsResult::prismatic_joint_length);
    
    pybind11::class_<delta::JointStateResult>(m, "JointStateResult")
        .def_readonly("prismatic_joint", &delta::JointStateResult::prismatic_joint)
        .def_readonly("roll_joint", &delta::JointStateResult::roll_joint)
        .def_readonly("pitch_joint", &delta::JointStateResult::pitch_joint);
    
    pybind11::class_<delta::OrientationResult>(m, "OrientationResult")
        .def_readonly("transformation_matrix", &delta::OrientationResult::transformation_matrix)
        .def_readonly("end_effector_position", &delta::OrientationResult::end_effector_position);
    
    // UPDATED: FabrikSolutionResult with collision fields
    pybind11::class_<delta::FabrikSolutionResult>(m, "FabrikSolutionResult")
        .def_readonly("final_chain", &delta::FabrikSolutionResult::final_chain)
        .def_readonly("converged", &delta::FabrikSolutionResult::converged)
        .def_readonly("final_error", &delta::FabrikSolutionResult::final_error)
        .def_readonly("spline_points", &delta::FabrikSolutionResult::spline_points)
        .def_readonly("segment_midpoints", &delta::FabrikSolutionResult::segment_midpoints)
        // NEW: Collision avoidance fields
        .def_readonly("safe_spline_points", &delta::FabrikSolutionResult::safe_spline_points)
        .def_readonly("collision_avoidance_applied", &delta::FabrikSolutionResult::collision_avoidance_applied)
        .def_readonly("collision_result", &delta::FabrikSolutionResult::collision_result);
    
    pybind11::class_<delta::MotorResult>(m, "MotorResult")
        .def_readonly("target_position", &delta::MotorResult::target_position)
        .def_readonly("fabrik_converged", &delta::MotorResult::fabrik_converged)
        .def_readonly("fabrik_error", &delta::MotorResult::fabrik_error)
        .def_readonly("solve_time_ms", &delta::MotorResult::solve_time_ms)
        .def_readonly("original_segment_numbers", &delta::MotorResult::original_segment_numbers)
        .def_readonly("original_segment_positions", &delta::MotorResult::original_segment_positions)
        .def_readonly("fabrik_joint_positions", &delta::MotorResult::fabrik_joint_positions)
        .def_readonly("levels", &delta::MotorResult::levels);
    
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
    // KINEMATICS MODULES
    // =============================================================================
    
    pybind11::class_<delta::FermatModule>(m, "FermatModule")
        .def_static("calculate", 
                   pybind11::overload_cast<double, double, double>(&delta::FermatModule::calculate))
        .def_static("calculate", 
                   pybind11::overload_cast<const delta::Vector3&>(&delta::FermatModule::calculate));
    
    pybind11::class_<delta::JointStateModule>(m, "JointStateModule")
        .def_static("calculate_from_fermat", &delta::JointStateModule::calculate_from_fermat);
    
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
    
    // Create FabrikSolverConfig first (needed for default arguments)
    pybind11::class_<delta::FabrikSolverConfig>(m, "FabrikSolverConfig")
        .def(pybind11::init<>())
        .def_readwrite("tolerance", &delta::FabrikSolverConfig::tolerance)
        .def_readwrite("max_iterations", &delta::FabrikSolverConfig::max_iterations)
        .def_readwrite("max_backward_forward_cycles", &delta::FabrikSolverConfig::max_backward_forward_cycles)
        .def_readwrite("enable_constraints", &delta::FabrikSolverConfig::enable_constraints)
        .def_readwrite("track_convergence_history", &delta::FabrikSolverConfig::track_convergence_history)
        .def_readwrite("verbose_logging", &delta::FabrikSolverConfig::verbose_logging);
    
    // FabrikSolver with collision methods
    pybind11::class_<delta::FabrikSolver>(m, "FabrikSolver")
        .def_static("solve", &delta::FabrikSolver::solve,
                   "initial_chain"_a, "target_position"_a, 
                   "tolerance"_a = delta::FABRIK_TOLERANCE, 
                   "max_iterations"_a = delta::FABRIK_MAX_ITERATIONS)
        .def_static("extract_spline_points", &delta::FabrikSolver::extract_spline_points)
        .def_static("calculate_segment_midpoints", &delta::FabrikSolver::calculate_segment_midpoints)
        // Collision avoidance methods with explicit defaults
        .def_static("solve_with_collision_avoidance", 
                   [](const delta::FabrikChain& chain, const delta::Vector3& target, 
                      const std::vector<delta::Obstacle>& obstacles) {
                       delta::FabrikSolverConfig config;
                       return delta::FabrikSolver::solve_with_collision_avoidance(
                           chain, target, obstacles, config, delta::SPLINE_THICKNESS, delta::COLLISION_SAFETY_MARGIN);
                   },
                   "initial_chain"_a, "target_position"_a, "obstacles"_a)
        .def_static("solve_safe", &delta::FabrikSolver::solve_safe,
                   "initial_chain"_a, "target_position"_a, "obstacles"_a,
                   "tolerance"_a = delta::FABRIK_TOLERANCE, 
                   "max_iterations"_a = delta::FABRIK_MAX_ITERATIONS);
    
    // Collision Avoidance Module
    pybind11::class_<delta::SplineCollisionAvoidance>(m, "SplineCollisionAvoidance")
        .def_static("avoid_collisions", 
                   [](const std::vector<delta::Vector3>& spline_points, 
                      const std::vector<delta::Obstacle>& obstacles) {
                       return delta::SplineCollisionAvoidance::avoid_collisions(
                           spline_points, obstacles, delta::SPLINE_THICKNESS, delta::COLLISION_SAFETY_MARGIN);
                   },
                   "spline_points"_a, "obstacles"_a)
        .def_static("has_collision", &delta::SplineCollisionAvoidance::has_collision,
                   "spline_points"_a, "obstacles"_a, "spline_thickness"_a)
        .def_static("find_closest_point_on_spline", &delta::SplineCollisionAvoidance::find_closest_point_on_spline);
    
    // =============================================================================
    // MOTOR MODULE
    // =============================================================================
    
    pybind11::class_<delta::MotorModule>(m, "MotorModule")
        .def_static("calculate_motors", 
                   pybind11::overload_cast<double, double, double>(&delta::MotorModule::calculate_motors))
        .def_static("calculate_motors", 
                   pybind11::overload_cast<const delta::Vector3&>(&delta::MotorModule::calculate_motors));
    
    // =============================================================================
    // CONVENIENCE FUNCTIONS
    // =============================================================================
    
    m.def("solve_delta_robot", &delta::fabrik_utils::solve_delta_robot,
          "num_segments"_a, "target"_a, "tolerance"_a = delta::FABRIK_TOLERANCE);
    
    m.def("calculate_motors", [](double x, double y, double z) {
        return delta::MotorModule::calculate_motors(x, y, z);
    });
    
    m.def("solve_with_spline", [](int num_segments, const delta::Vector3& target, double tolerance) {
        return delta::fabrik_utils::solve_delta_robot(num_segments, target, tolerance);
    }, "Solve FABRIK and return result with spline points",
    "num_segments"_a, "target"_a, "tolerance"_a = delta::FABRIK_TOLERANCE);
    
    // Collision-aware convenience functions
    m.def("solve_delta_robot_safe", &delta::fabrik_utils::solve_delta_robot_safe,
          "num_segments"_a, "target"_a, "obstacles"_a, "tolerance"_a = delta::FABRIK_TOLERANCE);
    
    m.def("create_sphere_obstacle", &delta::fabrik_utils::create_sphere_obstacle,
          "x"_a, "y"_a, "z"_a, "radius"_a);
    
    m.def("create_obstacles_from_positions", &delta::fabrik_utils::create_obstacles_from_positions);
    m.def("create_obstacles_from_coordinates", &delta::fabrik_utils::create_obstacles_from_coordinates);
    
    // Collision detection convenience
    m.def("check_spline_collision", [](const std::vector<delta::Vector3>& spline_points, 
                                      const std::vector<delta::Obstacle>& obstacles,
                                      double thickness) {
        return delta::SplineCollisionAvoidance::has_collision(spline_points, obstacles, thickness);
    }, "Check if spline collides with obstacles",
    "spline_points"_a, "obstacles"_a, "thickness"_a);
    
    // =============================================================================
    // CONSTANTS
    // =============================================================================
    
    m.attr("FABRIK_TOLERANCE") = delta::FABRIK_TOLERANCE;
    m.attr("DEFAULT_ROBOT_SEGMENTS") = delta::DEFAULT_ROBOT_SEGMENTS;
    m.attr("SPLINE_THICKNESS") = delta::SPLINE_THICKNESS;
    m.attr("COLLISION_SAFETY_MARGIN") = delta::COLLISION_SAFETY_MARGIN;
    m.attr("COLLISION_AVOIDANCE_TARGET_TIME_MS") = delta::COLLISION_AVOIDANCE_TARGET_TIME_MS;
}