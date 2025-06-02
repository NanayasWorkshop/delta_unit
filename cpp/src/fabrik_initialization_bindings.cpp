#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "fabrik_initialization.hpp"

using namespace pybind11::literals;

PYBIND11_MODULE(fabrik_initialization, m) {
    m.doc() = "Delta robot FABRIK initialization module";
    
    // NO Vector3 registration - assumes delta_types is imported
    
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
            return "FabrikJoint(pos=(" + std::to_string(j.position.x) + "," + 
                   std::to_string(j.position.y) + "," + std::to_string(j.position.z) + 
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
                   std::to_string(r.final_end_effector.x) + "," +
                   std::to_string(r.final_end_effector.y) + "," + 
                   std::to_string(r.final_end_effector.z) + 
                   "), total_reach=" + std::to_string(r.total_reach) + 
                   ", joints=" + std::to_string(r.chain.joints.size()) + ")";
        });
    
    // FabrikInitialization class
    pybind11::class_<delta::FabrikInitialization>(m, "FabrikInitialization")
        .def_static("initialize_straight_up", 
                   &delta::FabrikInitialization::initialize_straight_up,
                   "num_robot_segments"_a = delta::DEFAULT_ROBOT_SEGMENTS,
                   "Initialize robot chain in straight up configuration")
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
                   "Get total number of joints for N segments");
    
    // Expose constants for convenience
    m.attr("DEFAULT_ROBOT_SEGMENTS") = delta::DEFAULT_ROBOT_SEGMENTS;
    m.attr("SPHERICAL_JOINT_CONE_ANGLE_RAD") = delta::SPHERICAL_JOINT_CONE_ANGLE_RAD;
    m.attr("SPHERICAL_JOINT_CONE_ANGLE_DEG") = delta::SPHERICAL_JOINT_CONE_ANGLE_DEG;
}