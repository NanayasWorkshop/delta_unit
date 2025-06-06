// consolidated_bindings.cpp - ALL modules in one file (SIMPLE VERSION)
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
#include "collision_utils.hpp"  // NEW: Collision utilities

using namespace pybind11::literals;

// NEW: Test function for collision utilities
std::string test_collision_utilities() {
    // Create test collision world
    delta::CollisionWorld world = delta::create_test_collision_world();
    
    // Print debug info to console
    delta::print_collision_world_debug(world);
    
    // Generate JSON string
    std::string json_output = delta::collision_world_to_json_string(world);
    
    // Also save to file for visualization
    delta::save_collision_world_json(world, "cpp_collision_test.json");
    
    return json_output;
}

// NEW: Isaac Sim to C++ bridge function (complex version)
std::string test_isaac_collision_bridge(const pybind11::dict& isaac_data) {
    // Convert pybind11::dict to std::map for easier handling
    std::map<std::string, pybind11::object> isaac_map;
    for (auto item : isaac_data) {
        isaac_map[item.first.cast<std::string>()] = pybind11::reinterpret_borrow<pybind11::object>(item.second);
    }
    
    // Parse Isaac Sim data to C++ collision world
    delta::CollisionWorld world = delta::parse_isaac_collision_dict(isaac_map);
    
    // Generate JSON and save file
    std::string json_output = delta::collision_world_to_json_string(world);
    delta::save_collision_world_json(world, "isaac_to_cpp_collision.json");
    
    return json_output;
}

// NEW: Simple Isaac Sim bridge function (works with converted data)
std::string test_isaac_simple_bridge(const pybind11::dict& converted_data) {
    delta::CollisionWorld world;
    
    try {
        std::cout << "=== Parsing converted Isaac Sim collision data ===" << std::endl;
        
        // Parse spheres
        if (converted_data.contains("spheres")) {
            auto spheres_list = converted_data["spheres"].cast<std::vector<pybind11::dict>>();
            
            for (const auto& sphere_dict : spheres_list) {
                delta::CollisionSphere sphere;
                
                // Parse center (individual x,y,z values)
                sphere.center = delta::Vector3(
                    sphere_dict["center_x"].cast<double>(),
                    sphere_dict["center_y"].cast<double>(),
                    sphere_dict["center_z"].cast<double>()
                );
                
                // Parse radius
                sphere.radius = static_cast<float>(sphere_dict["radius"].cast<double>());
                
                // Parse quaternion (individual x,y,z,w values)
                sphere.quat_x = static_cast<float>(sphere_dict["quat_x"].cast<double>());
                sphere.quat_y = static_cast<float>(sphere_dict["quat_y"].cast<double>());
                sphere.quat_z = static_cast<float>(sphere_dict["quat_z"].cast<double>());
                sphere.quat_w = static_cast<float>(sphere_dict["quat_w"].cast<double>());
                
                // Parse rotation matrix (flattened array)
                auto rot_matrix_flat = sphere_dict["rotation_matrix"].cast<std::vector<double>>();
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        sphere.rotation_matrix(i, j) = rot_matrix_flat[i * 3 + j];
                    }
                }
                
                // Parse flags
                sphere.is_uniform_scale = sphere_dict["is_uniform_scale"].cast<bool>();
                
                world.add_sphere(sphere);
                std::cout << "  Added Isaac sphere: center=(" << sphere.center.x() << ", " << sphere.center.y() << ", " << sphere.center.z() << ") radius=" << sphere.radius << std::endl;
            }
        }
        
        // Parse boxes
        if (converted_data.contains("boxes")) {
            auto boxes_list = converted_data["boxes"].cast<std::vector<pybind11::dict>>();
            
            for (const auto& box_dict : boxes_list) {
                delta::CollisionBox box;
                
                // Parse center (individual x,y,z values)
                box.center = delta::Vector3(
                    box_dict["center_x"].cast<double>(),
                    box_dict["center_y"].cast<double>(),
                    box_dict["center_z"].cast<double>()
                );
                
                // Parse size (individual x,y,z values)
                box.size = delta::Vector3(
                    box_dict["size_x"].cast<double>(),
                    box_dict["size_y"].cast<double>(),
                    box_dict["size_z"].cast<double>()
                );
                
                // Parse quaternion (individual x,y,z,w values)
                box.quat_x = static_cast<float>(box_dict["quat_x"].cast<double>());
                box.quat_y = static_cast<float>(box_dict["quat_y"].cast<double>());
                box.quat_z = static_cast<float>(box_dict["quat_z"].cast<double>());
                box.quat_w = static_cast<float>(box_dict["quat_w"].cast<double>());
                
                // Parse rotation matrix (flattened array)
                auto rot_matrix_flat = box_dict["rotation_matrix"].cast<std::vector<double>>();
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        box.rotation_matrix(i, j) = rot_matrix_flat[i * 3 + j];
                    }
                }
                
                // Parse flags
                box.is_axis_aligned = box_dict["is_axis_aligned"].cast<bool>();
                
                world.add_box(box);
                std::cout << "  Added Isaac box: center=(" << box.center.x() << ", " << box.center.y() << ", " << box.center.z() << ") size=(" << box.size.x() << ", " << box.size.y() << ", " << box.size.z() << ")" << std::endl;
            }
        }
        
        // Parse cylinders (if any)
        if (converted_data.contains("cylinders")) {
            auto cylinders_list = converted_data["cylinders"].cast<std::vector<pybind11::dict>>();
            
            for (const auto& cyl_dict : cylinders_list) {
                delta::CollisionCylinder cylinder;
                
                // Parse center (individual x,y,z values)
                cylinder.center = delta::Vector3(
                    cyl_dict["center_x"].cast<double>(),
                    cyl_dict["center_y"].cast<double>(),
                    cyl_dict["center_z"].cast<double>()
                );
                
                // Parse dimensions
                cylinder.radius = static_cast<float>(cyl_dict["radius"].cast<double>());
                cylinder.height = static_cast<float>(cyl_dict["height"].cast<double>());
                
                // Parse quaternion (individual x,y,z,w values)
                cylinder.quat_x = static_cast<float>(cyl_dict["quat_x"].cast<double>());
                cylinder.quat_y = static_cast<float>(cyl_dict["quat_y"].cast<double>());
                cylinder.quat_z = static_cast<float>(cyl_dict["quat_z"].cast<double>());
                cylinder.quat_w = static_cast<float>(cyl_dict["quat_w"].cast<double>());
                
                // Parse rotation matrix (flattened array)
                auto rot_matrix_flat = cyl_dict["rotation_matrix"].cast<std::vector<double>>();
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        cylinder.rotation_matrix(i, j) = rot_matrix_flat[i * 3 + j];
                    }
                }
                
                // Parse flags
                cylinder.is_circular = cyl_dict["is_circular"].cast<bool>();
                cylinder.is_axis_aligned = cyl_dict["is_axis_aligned"].cast<bool>();
                
                world.add_cylinder(cylinder);
                std::cout << "  Added Isaac cylinder: center=(" << cylinder.center.x() << ", " << cylinder.center.y() << ", " << cylinder.center.z() << ") radius=" << cylinder.radius << " height=" << cylinder.height << std::endl;
            }
        }
        
        std::cout << "✅ Isaac Sim collision parsing SUCCESS: " << world.total_primitives() << " primitives" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error parsing converted Isaac Sim data: " << e.what() << std::endl;
        std::cerr << "Falling back to test collision world" << std::endl;
        return delta::collision_world_to_json_string(delta::create_test_collision_world());
    }
    
    // Generate JSON and save file
    std::string json_output = delta::collision_world_to_json_string(world);
    delta::save_collision_world_json(world, "isaac_real_collision.json");
    
    return json_output;
}

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
    // RESULT TYPES (Simple, no helpers)
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
    
    pybind11::class_<delta::FabrikSolutionResult>(m, "FabrikSolutionResult")
        .def_readonly("final_chain", &delta::FabrikSolutionResult::final_chain)
        .def_readonly("converged", &delta::FabrikSolutionResult::converged)
        .def_readonly("final_error", &delta::FabrikSolutionResult::final_error);
    
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
    
    pybind11::class_<delta::FabrikSolver>(m, "FabrikSolver")
        .def_static("solve", &delta::FabrikSolver::solve);
    
    // =============================================================================
    // MOTOR MODULE
    // =============================================================================
    
    pybind11::class_<delta::MotorModule>(m, "MotorModule")
        .def_static("calculate_motors", 
                   pybind11::overload_cast<double, double, double>(&delta::MotorModule::calculate_motors))
        .def_static("calculate_motors", 
                   pybind11::overload_cast<const delta::Vector3&>(&delta::MotorModule::calculate_motors));
    
    // =============================================================================
    // COLLISION TESTING (Day 1)
    // =============================================================================
    
    m.def("test_collision_utilities", &test_collision_utilities, 
          "Test collision utilities - creates test data and returns JSON");
    
    m.def("test_isaac_collision_bridge", &test_isaac_collision_bridge,
          "Bridge: Convert Isaac Sim collision data to C++ format and return JSON");
    
    m.def("test_isaac_simple_bridge", &test_isaac_simple_bridge,
          "Simple bridge: Convert pre-processed Isaac Sim data to C++ format");
    
    // =============================================================================
    // CONVENIENCE FUNCTIONS
    // =============================================================================
    
    m.def("solve_delta_robot", &delta::fabrik_utils::solve_delta_robot);
    m.def("calculate_motors", [](double x, double y, double z) {
        return delta::MotorModule::calculate_motors(x, y, z);
    });
    
    // =============================================================================
    // CONSTANTS
    // =============================================================================
    
    m.attr("FABRIK_TOLERANCE") = delta::FABRIK_TOLERANCE;
    m.attr("DEFAULT_ROBOT_SEGMENTS") = delta::DEFAULT_ROBOT_SEGMENTS;
}