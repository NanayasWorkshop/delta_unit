#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include "delta_core.hpp"

using namespace pybind11::literals;

PYBIND11_MODULE(delta_core, m) {
    m.doc() = "Delta Robot Core Layer - Foundation types, constants, and utilities";
    
    // ========================================================================
    // VERSION INFORMATION
    // ========================================================================
    
    pybind11::class_<delta::core::Version>(m, "Version")
        .def_readonly("major", &delta::core::Version::major)
        .def_readonly("minor", &delta::core::Version::minor)
        .def_readonly("patch", &delta::core::Version::patch)
        .def("to_string", &delta::core::Version::to_string)
        .def("is_compatible_with", &delta::core::Version::is_compatible_with)
        .def("__repr__", [](const delta::core::Version& v) {
            return "Version(" + v.to_string() + ")";
        });
    
    m.def("get_version", &delta::core::get_version, "Get current core library version");
    
    // ========================================================================
    // CONSTANTS
    // ========================================================================
    
    // Robot Physical Constants
    auto robot_constants = m.def_submodule("robot", "Robot physical constants");
    robot_constants.attr("RADIUS") = delta::core::robot::RADIUS;
    robot_constants.attr("MIN_HEIGHT") = delta::core::robot::MIN_HEIGHT;
    robot_constants.attr("WORKING_HEIGHT") = delta::core::robot::WORKING_HEIGHT;
    robot_constants.attr("MOTOR_LIMIT") = delta::core::robot::MOTOR_LIMIT;
    robot_constants.attr("DEFAULT_SEGMENTS") = delta::core::robot::DEFAULT_SEGMENTS;
    
    // FABRIK Algorithm Constants
    auto fabrik_constants = m.def_submodule("fabrik", "FABRIK algorithm constants");
    fabrik_constants.attr("TOLERANCE") = delta::core::fabrik::TOLERANCE;
    fabrik_constants.attr("MAX_ITERATIONS") = delta::core::fabrik::MAX_ITERATIONS;
    fabrik_constants.attr("MAX_CYCLES") = delta::core::fabrik::MAX_CYCLES;
    fabrik_constants.attr("SPHERICAL_JOINT_CONE_ANGLE_RAD") = delta::core::fabrik::SPHERICAL_JOINT_CONE_ANGLE_RAD;
    
    // Mathematical Constants
    auto math_constants = m.def_submodule("math", "Mathematical constants");
    math_constants.attr("EPSILON") = delta::core::math::EPSILON;
    math_constants.attr("PI") = delta::core::math::PI;
    math_constants.attr("TWO_PI") = delta::core::math::TWO_PI;
    math_constants.attr("HALF_PI") = delta::core::math::HALF_PI;
    
    // Geometry Constants
    auto geometry_constants = m.def_submodule("geometry", "Geometry constants");
    geometry_constants.attr("BASE_A_ANGLE") = delta::core::geometry::BASE_A_ANGLE;
    geometry_constants.attr("BASE_B_ANGLE") = delta::core::geometry::BASE_B_ANGLE;
    geometry_constants.attr("BASE_C_ANGLE") = delta::core::geometry::BASE_C_ANGLE;
    
    // Utility functions
    m.def("rad_to_deg", &delta::core::rad_to_deg, "Convert radians to degrees");
    m.def("deg_to_rad", &delta::core::deg_to_rad, "Convert degrees to radians");
    
    // ========================================================================
    // TYPES
    // ========================================================================
    
    // Vector3 and Matrix types are automatically handled by pybind11/eigen.h
    
    // Configuration
    pybind11::class_<delta::core::Configuration>(m, "Configuration")
        .def(pybind11::init<>())
        .def_readwrite("tolerance", &delta::core::Configuration::tolerance)
        .def_readwrite("max_iterations", &delta::core::Configuration::max_iterations)
        .def_readwrite("enable_constraints", &delta::core::Configuration::enable_constraints)
        .def_readwrite("verbose_logging", &delta::core::Configuration::verbose_logging)
        .def_readwrite("track_performance", &delta::core::Configuration::track_performance)
        .def("is_valid", &delta::core::Configuration::is_valid)
        .def("__repr__", [](const delta::core::Configuration& c) {
            return "Configuration(tolerance=" + std::to_string(c.tolerance) +
                   ", max_iterations=" + std::to_string(c.max_iterations) +
                   ", enable_constraints=" + (c.enable_constraints ? "True" : "False") + ")";
        });
    
    // CoordinateFrame
    pybind11::class_<delta::core::CoordinateFrame>(m, "CoordinateFrame")
        .def(pybind11::init<const delta::core::Vector3&, const delta::core::Vector3&, 
                           const delta::core::Vector3&, const delta::core::Vector3&>(),
             "origin"_a = delta::core::Vector3(0,0,0), "u_axis"_a = delta::core::Vector3(1,0,0),
             "v_axis"_a = delta::core::Vector3(0,1,0), "w_axis"_a = delta::core::Vector3(0,0,1))
        .def_readonly("origin", &delta::core::CoordinateFrame::origin)
        .def_readonly("u_axis", &delta::core::CoordinateFrame::u_axis)
        .def_readonly("v_axis", &delta::core::CoordinateFrame::v_axis)
        .def_readonly("w_axis", &delta::core::CoordinateFrame::w_axis)
        .def("is_orthonormal", &delta::core::CoordinateFrame::is_orthonormal, "tolerance"_a = 1e-6)
        .def("to_matrix", &delta::core::CoordinateFrame::to_matrix)
        .def("__repr__", [](const delta::core::CoordinateFrame& f) {
            return "CoordinateFrame(origin=(" + std::to_string(f.origin.x()) + "," + 
                   std::to_string(f.origin.y()) + "," + std::to_string(f.origin.z()) + "))";
        });
    
    // JointType enum
    pybind11::enum_<delta::core::JointType>(m, "JointType")
        .value("FIXED_BASE", delta::core::JointType::FIXED_BASE)
        .value("SPHERICAL_120", delta::core::JointType::SPHERICAL_120)
        .value("END_EFFECTOR", delta::core::JointType::END_EFFECTOR);
    
    // SolverStatus enum
    pybind11::enum_<delta::core::SolverStatus>(m, "SolverStatus")
        .value("SUCCESS", delta::core::SolverStatus::SUCCESS)
        .value("MAX_ITERATIONS", delta::core::SolverStatus::MAX_ITERATIONS)
        .value("STALLED", delta::core::SolverStatus::STALLED)
        .value("INVALID_TARGET", delta::core::SolverStatus::INVALID_TARGET)
        .value("CONFIGURATION_ERROR", delta::core::SolverStatus::CONFIGURATION_ERROR)
        .value("NUMERICAL_ERROR", delta::core::SolverStatus::NUMERICAL_ERROR);
    
    m.def("solver_status_to_string", 
          [](delta::core::SolverStatus status) { return delta::core::to_string(status); },
          "Convert SolverStatus to string");
    
    // Timer
    pybind11::class_<delta::core::Timer>(m, "Timer")
        .def(pybind11::init<>())
        .def("reset", &delta::core::Timer::reset)
        .def("elapsed_ms", &delta::core::Timer::elapsed_ms)
        .def("elapsed_seconds", &delta::core::Timer::elapsed_seconds)
        .def("__repr__", [](const delta::core::Timer& t) {
            return "Timer(elapsed=" + std::to_string(t.elapsed_ms()) + "ms)";
        });
    
    // PerformanceMetrics
    pybind11::class_<delta::core::PerformanceMetrics>(m, "PerformanceMetrics")
        .def(pybind11::init<>())
        .def_readwrite("solve_time_ms", &delta::core::PerformanceMetrics::solve_time_ms)
        .def_readwrite("total_iterations", &delta::core::PerformanceMetrics::total_iterations)
        .def_readwrite("function_calls", &delta::core::PerformanceMetrics::function_calls)
        .def_readwrite("final_error", &delta::core::PerformanceMetrics::final_error)
        .def_readwrite("status", &delta::core::PerformanceMetrics::status)
        .def("reset", &delta::core::PerformanceMetrics::reset)
        .def("__repr__", [](const delta::core::PerformanceMetrics& p) {
            return "PerformanceMetrics(time=" + std::to_string(p.solve_time_ms) + 
                   "ms, iterations=" + std::to_string(p.total_iterations) + 
                   ", error=" + std::to_string(p.final_error) + ")";
        });
    
    // ========================================================================
    // ERROR HANDLING
    // ========================================================================
    
    // ErrorCode enum
    pybind11::enum_<delta::core::error::ErrorCode>(m, "ErrorCode")
        .value("SUCCESS", delta::core::error::ErrorCode::SUCCESS)
        .value("INVALID_TOLERANCE", delta::core::error::ErrorCode::INVALID_TOLERANCE)
        .value("INVALID_ITERATIONS", delta::core::error::ErrorCode::INVALID_ITERATIONS)
        .value("INVALID_SEGMENTS", delta::core::error::ErrorCode::INVALID_SEGMENTS)
        .value("INVALID_CONFIGURATION", delta::core::error::ErrorCode::INVALID_CONFIGURATION)
        .value("DIVISION_BY_ZERO", delta::core::error::ErrorCode::DIVISION_BY_ZERO)
        .value("INVALID_VECTOR", delta::core::error::ErrorCode::INVALID_VECTOR)
        .value("INVALID_MATRIX", delta::core::error::ErrorCode::INVALID_MATRIX)
        .value("NUMERICAL_INSTABILITY", delta::core::error::ErrorCode::NUMERICAL_INSTABILITY)
        .value("MAX_ITERATIONS_REACHED", delta::core::error::ErrorCode::MAX_ITERATIONS_REACHED)
        .value("SOLVER_STALLED", delta::core::error::ErrorCode::SOLVER_STALLED)
        .value("TARGET_UNREACHABLE", delta::core::error::ErrorCode::TARGET_UNREACHABLE)
        .value("INVALID_INITIAL_STATE", delta::core::error::ErrorCode::INVALID_INITIAL_STATE)
        .value("JOINT_LIMIT_EXCEEDED", delta::core::error::ErrorCode::JOINT_LIMIT_EXCEEDED)
        .value("WORKSPACE_VIOLATION", delta::core::error::ErrorCode::WORKSPACE_VIOLATION)
        .value("CONE_CONSTRAINT_VIOLATION", delta::core::error::ErrorCode::CONE_CONSTRAINT_VIOLATION)
        .value("INVALID_INPUT", delta::core::error::ErrorCode::INVALID_INPUT)
        .value("NULL_POINTER", delta::core::error::ErrorCode::NULL_POINTER)
        .value("INVALID_DIMENSIONS", delta::core::error::ErrorCode::INVALID_DIMENSIONS)
        .value("OUT_OF_RANGE", delta::core::error::ErrorCode::OUT_OF_RANGE);
    
    m.def("error_code_to_string", 
          [](delta::core::error::ErrorCode code) { return delta::core::error::to_string(code); },
          "Convert ErrorCode to string");
    
    // LogLevel enum
    pybind11::enum_<delta::core::error::LogLevel>(m, "LogLevel")
        .value("DEBUG", delta::core::error::LogLevel::DEBUG)
        .value("INFO", delta::core::error::LogLevel::INFO)
        .value("WARNING", delta::core::error::LogLevel::WARNING)
        .value("ERROR", delta::core::error::LogLevel::ERROR);
    
    // Logger class
    pybind11::class_<delta::core::error::Logger>(m, "Logger")
        .def_static("set_level", &delta::core::error::Logger::set_level)
        .def_static("enable", &delta::core::error::Logger::enable, "enabled"_a = true)
        .def_static("is_enabled", &delta::core::error::Logger::is_enabled)
        .def_static("debug", &delta::core::error::Logger::debug)
        .def_static("info", &delta::core::error::Logger::info)
        .def_static("warning", &delta::core::error::Logger::warning)
        .def_static("error", &delta::core::error::Logger::error);
    
    // Exception classes
    pybind11::register_exception<delta::core::error::DeltaRobotException>(m, "DeltaRobotException");
    pybind11::register_exception<delta::core::error::ConfigurationError>(m, "ConfigurationError");
    pybind11::register_exception<delta::core::error::NumericalError>(m, "NumericalError");
    pybind11::register_exception<delta::core::error::ConvergenceError>(m, "ConvergenceError");
    pybind11::register_exception<delta::core::error::ConstraintError>(m, "ConstraintError");
    pybind11::register_exception<delta::core::error::ValidationError>(m, "ValidationError");
    
    // ========================================================================
    // MATHEMATICAL UTILITIES
    // ========================================================================
    
    // VectorOps class
    auto vector_ops = m.def_submodule("vector_ops", "Vector operations");
    vector_ops.def("angle_between", &delta::core::math::VectorOps::angle_between);
    vector_ops.def("distance", &delta::core::math::VectorOps::distance);
    vector_ops.def("distance_squared", &delta::core::math::VectorOps::distance_squared);
    vector_ops.def("is_unit_vector", &delta::core::math::VectorOps::is_unit_vector, "tolerance"_a = delta::core::math::EPSILON);
    vector_ops.def("is_zero_vector", &delta::core::math::VectorOps::is_zero_vector, "tolerance"_a = delta::core::math::EPSILON);
    vector_ops.def("safe_normalize", &delta::core::math::VectorOps::safe_normalize, "min_norm"_a = delta::core::math::EPSILON);
    
    // GeometryOps class
    auto geometry_ops = m.def_submodule("geometry_ops", "Geometric operations");
    geometry_ops.def("calculate_plane_normal", &delta::core::math::GeometryOps::calculate_plane_normal);
    geometry_ops.def("calculate_z_intersection", &delta::core::math::GeometryOps::calculate_z_intersection);
    geometry_ops.def("project_direction_onto_cone", &delta::core::math::GeometryOps::project_direction_onto_cone);
    
    // BasePositions class
    auto base_positions = m.def_submodule("base_positions", "Robot base positions");
    base_positions.def("get_base_position_A", &delta::core::math::BasePositions::get_base_position_A);
    base_positions.def("get_base_position_B", &delta::core::math::BasePositions::get_base_position_B);
    base_positions.def("get_base_position_C", &delta::core::math::BasePositions::get_base_position_C);
    base_positions.def("get_all_base_positions", &delta::core::math::BasePositions::get_all_base_positions);
    base_positions.def("validate_base_triangle", &delta::core::math::BasePositions::validate_base_triangle, "tolerance"_a = delta::core::math::EPSILON);
    
    // ========================================================================
    // CORE LIBRARY FUNCTIONS
    // ========================================================================
    
    m.def("initialize", &delta::core::initialize, 
          "enable_logging"_a = false, "log_level"_a = delta::core::error::LogLevel::INFO,
          "Initialize the delta core library");
    
    m.def("shutdown", &delta::core::shutdown, "Shutdown the delta core library");
    m.def("is_initialized", &delta::core::is_initialized, "Check if core library is initialized");
    
    m.def("get_environment_info", &delta::core::get_environment_info, 
          "Get information about the current environment");
    
    m.def("get_performance_benchmarks", &delta::core::get_performance_benchmarks,
          "iterations"_a = 10000, "Get performance benchmarks for core operations");
    
    // Configuration presets
    auto presets = m.def_submodule("presets", "Configuration presets");
    presets.def("get_fast_config", &delta::core::presets::get_fast_config);
    presets.def("get_precise_config", &delta::core::presets::get_precise_config);
    presets.def("get_debug_config", &delta::core::presets::get_debug_config);
    presets.def("get_default_config", &delta::core::presets::get_default_config);
    
    // ========================================================================
    // CONVENIENCE FUNCTIONS
    // ========================================================================
    
    // Convenience Vector3 constructor function
    m.def("Vector3", [](double x=0, double y=0, double z=0) { 
        return delta::core::Vector3(x, y, z); 
    }, "x"_a=0, "y"_a=0, "z"_a=0, "Create Vector3");
    
    // Convenience Matrix4 constructor function
    m.def("Matrix4", []() { 
        return delta::core::Matrix4::Identity(); 
    }, "Create 4x4 identity matrix");
    
    // ========================================================================
    // BACKWARD COMPATIBILITY
    // ========================================================================
    
    // Legacy constants for smooth transition
    m.attr("ROBOT_RADIUS") = delta::core::robot::RADIUS;
    m.attr("MIN_HEIGHT") = delta::core::robot::MIN_HEIGHT;
    m.attr("WORKING_HEIGHT") = delta::core::robot::WORKING_HEIGHT;
    m.attr("MOTOR_LIMIT") = delta::core::robot::MOTOR_LIMIT;
    m.attr("DEFAULT_ROBOT_SEGMENTS") = delta::core::robot::DEFAULT_SEGMENTS;
    
    m.attr("FABRIK_TOLERANCE") = delta::core::fabrik::TOLERANCE;
    m.attr("FABRIK_MAX_ITERATIONS") = delta::core::fabrik::MAX_ITERATIONS;
    m.attr("SPHERICAL_JOINT_CONE_ANGLE_RAD") = delta::core::fabrik::SPHERICAL_JOINT_CONE_ANGLE_RAD;
    m.attr("EPSILON_MATH") = delta::core::math::EPSILON;
    
    m.attr("BASE_A_ANGLE") = delta::core::geometry::BASE_A_ANGLE;
    m.attr("BASE_B_ANGLE") = delta::core::geometry::BASE_B_ANGLE;
    m.attr("BASE_C_ANGLE") = delta::core::geometry::BASE_C_ANGLE;
    
    // Legacy base position functions
    m.def("get_base_position_A", &delta::core::math::BasePositions::get_base_position_A);
    m.def("get_base_position_B", &delta::core::math::BasePositions::get_base_position_B);
    m.def("get_base_position_C", &delta::core::math::BasePositions::get_base_position_C);
}