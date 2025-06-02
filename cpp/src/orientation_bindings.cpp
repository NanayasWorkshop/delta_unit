#include <pybind11/pybind11.h>
#include "orientation_module.hpp"

using namespace pybind11::literals;

PYBIND11_MODULE(orientation_module, m) {
    m.doc() = "Delta robot orientation calculation module";
    
    // Matrix4x4 structure
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
    
    // CoordinateFrame structure
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
    
    // OrientationResult - main output structure
    pybind11::class_<delta::OrientationResult>(m, "OrientationResult")
        .def_readonly("transformation_matrix", &delta::OrientationResult::transformation_matrix)
        .def_readonly("fermat_point", &delta::OrientationResult::fermat_point)
        .def_readonly("end_effector_position", &delta::OrientationResult::end_effector_position)
        .def_readonly("UVW_at_fermat", &delta::OrientationResult::UVW_at_fermat)
        .def_readonly("final_frame", &delta::OrientationResult::final_frame)
        .def("__repr__", [](const delta::OrientationResult& r) {
            return std::string("OrientationResult(end_effector=(") + 
                   std::to_string(r.end_effector_position.x) + "," +
                   std::to_string(r.end_effector_position.y) + "," + 
                   std::to_string(r.end_effector_position.z) + 
                   "), fermat=(" + std::to_string(r.fermat_point.x) + "," +
                   std::to_string(r.fermat_point.y) + "," + 
                   std::to_string(r.fermat_point.z) + "))";
        });
    
    // OrientationModule - main interface
    pybind11::class_<delta::OrientationModule>(m, "OrientationModule")
        .def_static("calculate", 
                   pybind11::overload_cast<double, double, double>(&delta::OrientationModule::calculate),
                   "Calculate from x,y,z coordinates")
        .def_static("calculate", 
                   pybind11::overload_cast<const delta::Vector3&>(&delta::OrientationModule::calculate),
                   "Calculate from Vector3")
        .def_static("calculate_from_kinematics", 
                   &delta::OrientationModule::calculate_from_kinematics,
                   "Calculate from existing KinematicsResult");
}