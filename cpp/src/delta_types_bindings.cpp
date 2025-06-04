#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "math_utils.hpp"
#include "orientation_module.hpp"

using namespace pybind11::literals;

PYBIND11_MODULE(delta_types, m) {
    m.doc() = "Delta robot shared types module - Foundation for all other modules (Eigen-based)";
    
    // Vector3 (Eigen::Vector3d) - Automatic conversion via pybind11/eigen.h
    // No manual registration needed! pybind11/eigen.h handles this automatically
    
    // Matrix4 (Eigen::Matrix4d) - Automatic conversion via pybind11/eigen.h
    // No manual registration needed! pybind11/eigen.h handles this automatically
    
    // CoordinateFrame - Manual registration (uses Vector3 internally)
    pybind11::class_<delta::CoordinateFrame>(m, "CoordinateFrame")
        .def(pybind11::init<const delta::Vector3&, const delta::Vector3&, 
                           const delta::Vector3&, const delta::Vector3&>(),
             "origin"_a = delta::Vector3(0,0,0), "u_axis"_a = delta::Vector3(1,0,0),
             "v_axis"_a = delta::Vector3(0,1,0), "w_axis"_a = delta::Vector3(0,0,1))
        .def_readonly("origin", &delta::CoordinateFrame::origin)
        .def_readonly("u_axis", &delta::CoordinateFrame::u_axis)
        .def_readonly("v_axis", &delta::CoordinateFrame::v_axis)
        .def_readonly("w_axis", &delta::CoordinateFrame::w_axis)
        .def("__repr__", [](const delta::CoordinateFrame& f) {
            return std::string("CoordinateFrame(origin=") + 
                   "(" + std::to_string(f.origin.x()) + "," + std::to_string(f.origin.y()) + "," + std::to_string(f.origin.z()) + ")" +
                   ", u=" + "(" + std::to_string(f.u_axis.x()) + "," + std::to_string(f.u_axis.y()) + "," + std::to_string(f.u_axis.z()) + ")" +
                   ", v=" + "(" + std::to_string(f.v_axis.x()) + "," + std::to_string(f.v_axis.y()) + "," + std::to_string(f.v_axis.z()) + ")" +
                   ", w=" + "(" + std::to_string(f.w_axis.x()) + "," + std::to_string(f.w_axis.y()) + "," + std::to_string(f.w_axis.z()) + "))";
        });
    
    // Constants module integration - expose essential constants from constants.hpp
    // Robot Physical Constants
    m.attr("ROBOT_RADIUS") = delta::ROBOT_RADIUS;
    m.attr("MIN_HEIGHT") = delta::MIN_HEIGHT;
    m.attr("WORKING_HEIGHT") = delta::WORKING_HEIGHT;
    m.attr("MOTOR_LIMIT") = delta::MOTOR_LIMIT;
    
    // FABRIK Configuration Constants
    m.attr("DEFAULT_ROBOT_SEGMENTS") = delta::DEFAULT_ROBOT_SEGMENTS;
    m.attr("SPHERICAL_JOINT_CONE_ANGLE_RAD") = delta::SPHERICAL_JOINT_CONE_ANGLE_RAD;
    
    // FABRIK Solver Constants
    m.attr("FABRIK_TOLERANCE") = delta::FABRIK_TOLERANCE;
    m.attr("FABRIK_MAX_ITERATIONS") = delta::FABRIK_MAX_ITERATIONS;
    m.attr("EPSILON_MATH") = delta::EPSILON_MATH;
    
    // Geometry Constants - Base actuator positions
    m.attr("BASE_A_ANGLE") = delta::BASE_A_ANGLE;
    m.attr("BASE_B_ANGLE") = delta::BASE_B_ANGLE;
    m.attr("BASE_C_ANGLE") = delta::BASE_C_ANGLE;
    
    // Utility functions
    m.def("rad_to_deg", &delta::rad_to_deg, "Convert radians to degrees");
    m.def("deg_to_rad", &delta::deg_to_rad, "Convert degrees to radians");
    
    // Base position functions
    m.def("get_base_position_A", &delta::get_base_position_A, "Get base A position");
    m.def("get_base_position_B", &delta::get_base_position_B, "Get base B position");
    m.def("get_base_position_C", &delta::get_base_position_C, "Get base C position");
    
    // Convenience aliases for Python users
    m.def("Vector3", [](double x=0, double y=0, double z=0) { 
        return delta::Vector3(x, y, z); 
    }, "x"_a=0, "y"_a=0, "z"_a=0, "Create Vector3 (Eigen::Vector3d)");
    
    m.def("Matrix4", []() { 
        return delta::Matrix4::Identity(); 
    }, "Create 4x4 identity matrix (Eigen::Matrix4d)");
}