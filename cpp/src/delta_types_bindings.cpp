#include <pybind11/pybind11.h>
#include "math_utils.hpp"
#include "orientation_module.hpp"

using namespace pybind11::literals;

PYBIND11_MODULE(delta_types, m) {
    m.doc() = "Delta robot shared types module - Foundation for all other modules";
    
    // Vector3 - Primary registration (used by all modules)
    pybind11::class_<delta::Vector3>(m, "Vector3")
        .def(pybind11::init<double, double, double>(), "x"_a = 0, "y"_a = 0, "z"_a = 0)
        .def_readwrite("x", &delta::Vector3::x)
        .def_readwrite("y", &delta::Vector3::y)
        .def_readwrite("z", &delta::Vector3::z)
        .def("norm", &delta::Vector3::norm, "Calculate vector magnitude")
        .def("normalized", &delta::Vector3::normalized, "Return normalized vector")
        .def("dot", &delta::Vector3::dot, "Calculate dot product with another vector")
        .def("__add__", &delta::Vector3::operator+)
        .def("__sub__", &delta::Vector3::operator-)
        .def("__mul__", &delta::Vector3::operator*)
        .def("__truediv__", &delta::Vector3::operator/, "Division operator for scalar division")
        .def("__repr__", [](const delta::Vector3& v) {
            return "Vector3(" + std::to_string(v.x) + ", " + std::to_string(v.y) + ", " + std::to_string(v.z) + ")";
        });
    
    // Matrix4x4 - Primary registration
    pybind11::class_<delta::Matrix4x4>(m, "Matrix4x4")
        .def(pybind11::init<>())
        .def("__getitem__", [](const delta::Matrix4x4& m, pybind11::tuple indices) {
            if (indices.size() != 2) throw pybind11::index_error();
            int i = indices[0].cast<int>();
            int j = indices[1].cast<int>();
            if (i < 0 || i >= 4 || j < 0 || j >= 4) throw pybind11::index_error();
            return m.data[i][j];
        })
        .def("__setitem__", [](delta::Matrix4x4& m, pybind11::tuple indices, double value) {
            if (indices.size() != 2) throw pybind11::index_error();
            int i = indices[0].cast<int>();
            int j = indices[1].cast<int>();
            if (i < 0 || i >= 4 || j < 0 || j >= 4) throw pybind11::index_error();
            m.data[i][j] = value;
        })
        .def("set_translation", &delta::Matrix4x4::set_translation)
        .def("set_rotation", &delta::Matrix4x4::set_rotation)
        .def("__repr__", [](const delta::Matrix4x4& m) {
            std::string result = "Matrix4x4(\n";
            for (int i = 0; i < 4; i++) {
                result += "  [";
                for (int j = 0; j < 4; j++) {
                    result += std::to_string(m.data[i][j]);
                    if (j < 3) result += ", ";
                }
                result += "]\n";
            }
            result += ")";
            return result;
        });
    
    // CoordinateFrame - Primary registration
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
                   "(" + std::to_string(f.origin.x) + "," + std::to_string(f.origin.y) + "," + std::to_string(f.origin.z) + ")" +
                   ", u=" + "(" + std::to_string(f.u_axis.x) + "," + std::to_string(f.u_axis.y) + "," + std::to_string(f.u_axis.z) + ")" +
                   ", v=" + "(" + std::to_string(f.v_axis.x) + "," + std::to_string(f.v_axis.y) + "," + std::to_string(f.v_axis.z) + ")" +
                   ", w=" + "(" + std::to_string(f.w_axis.x) + "," + std::to_string(f.w_axis.y) + "," + std::to_string(f.w_axis.z) + "))";
        });
    
    // Constants module integration - expose key constants
    m.attr("ROBOT_RADIUS") = delta::ROBOT_RADIUS;
    m.attr("MIN_HEIGHT") = delta::MIN_HEIGHT;
    m.attr("WORKING_HEIGHT") = delta::WORKING_HEIGHT;
    m.attr("MOTOR_LIMIT") = delta::MOTOR_LIMIT;
    m.attr("WORKSPACE_CONE_ANGLE_RAD") = delta::WORKSPACE_CONE_ANGLE_RAD;
    
    // Utility functions
    m.def("rad_to_deg", &delta::rad_to_deg, "Convert radians to degrees");
    m.def("deg_to_rad", &delta::deg_to_rad, "Convert degrees to radians");
    
    // Base position functions
    m.def("get_base_position_A", &delta::get_base_position_A, "Get base A position");
    m.def("get_base_position_B", &delta::get_base_position_B, "Get base B position");
    m.def("get_base_position_C", &delta::get_base_position_C, "Get base C position");
}