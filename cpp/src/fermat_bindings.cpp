#include <pybind11/pybind11.h>
#include "fermat_module.hpp"

using namespace pybind11::literals;

PYBIND11_MODULE(fermat_module, m) {
    m.doc() = "Delta robot Fermat calculation module";
    
    // Vector3 (reused from math_utils)
    pybind11::class_<delta::Vector3>(m, "Vector3")
        .def(pybind11::init<double, double, double>(), "x"_a = 0, "y"_a = 0, "z"_a = 0)
        .def_readwrite("x", &delta::Vector3::x)
        .def_readwrite("y", &delta::Vector3::y)
        .def_readwrite("z", &delta::Vector3::z)
        .def("__repr__", [](const delta::Vector3& v) {
            return "Vector3(" + std::to_string(v.x) + ", " + std::to_string(v.y) + ", " + std::to_string(v.z) + ")";
        });
    
    // FermatResult - clean output structure
    pybind11::class_<delta::FermatResult>(m, "FermatResult")
        .def_readonly("z_A", &delta::FermatResult::z_A)
        .def_readonly("z_B", &delta::FermatResult::z_B)
        .def_readonly("z_C", &delta::FermatResult::z_C)
        .def_readonly("fermat_point", &delta::FermatResult::fermat_point)
        .def("__repr__", [](const delta::FermatResult& r) {
            return "FermatResult(z_A=" + std::to_string(r.z_A) + 
                   ", z_B=" + std::to_string(r.z_B) + 
                   ", z_C=" + std::to_string(r.z_C) + 
                   ", fermat=(" + std::to_string(r.fermat_point.x) + "," +
                   std::to_string(r.fermat_point.y) + "," + 
                   std::to_string(r.fermat_point.z) + "))";
        });
    
    // FermatModule - main interface
    pybind11::class_<delta::FermatModule>(m, "FermatModule")
        .def_static("calculate", 
                   pybind11::overload_cast<double, double, double>(&delta::FermatModule::calculate),
                   "Calculate from x,y,z coordinates")
        .def_static("calculate", 
                   pybind11::overload_cast<const delta::Vector3&>(&delta::FermatModule::calculate),
                   "Calculate from Vector3")
        .def_static("get_base_A", &delta::FermatModule::get_base_A)
        .def_static("get_base_B", &delta::FermatModule::get_base_B)
        .def_static("get_base_C", &delta::FermatModule::get_base_C);
}
