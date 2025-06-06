#include "collision_utils.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

namespace delta {

// === DEBUGGING FUNCTIONS ===

void print_collision_world_debug(const CollisionWorld& world) {
    std::cout << "\n=== COLLISION WORLD DEBUG ===" << std::endl;
    std::cout << "Total primitives: " << world.total_primitives() << std::endl;
    std::cout << "  Spheres: " << world.spheres.size() << std::endl;
    std::cout << "  Boxes: " << world.boxes.size() << std::endl;
    std::cout << "  Cylinders: " << world.cylinders.size() << std::endl;
    
    if (world.is_empty()) {
        std::cout << "  (Empty collision world)" << std::endl;
        return;
    }
    
    // Print spheres
    for (size_t i = 0; i < world.spheres.size(); ++i) {
        print_collision_sphere_debug(world.spheres[i], static_cast<int>(i));
    }
    
    // Print boxes
    for (size_t i = 0; i < world.boxes.size(); ++i) {
        print_collision_box_debug(world.boxes[i], static_cast<int>(i));
    }
    
    // Print cylinders
    for (size_t i = 0; i < world.cylinders.size(); ++i) {
        print_collision_cylinder_debug(world.cylinders[i], static_cast<int>(i));
    }
    
    std::cout << "=== END COLLISION WORLD ===" << std::endl;
}

void print_collision_sphere_debug(const CollisionSphere& sphere, int index) {
    std::cout << "  Sphere";
    if (index >= 0) std::cout << " [" << index << "]";
    std::cout << ":" << std::endl;
    
    std::cout << "    Center: (" << std::fixed << std::setprecision(3) 
              << sphere.center.x() << ", " << sphere.center.y() << ", " << sphere.center.z() << ") mm" << std::endl;
    std::cout << "    Radius: " << sphere.radius << " mm" << std::endl;
    std::cout << "    Quaternion: (" << sphere.quat_x << ", " << sphere.quat_y 
              << ", " << sphere.quat_z << ", " << sphere.quat_w << ")" << std::endl;
    std::cout << "    Uniform scale: " << (sphere.is_uniform_scale ? "Yes" : "No") << std::endl;
}

void print_collision_box_debug(const CollisionBox& box, int index) {
    std::cout << "  Box";
    if (index >= 0) std::cout << " [" << index << "]";
    std::cout << ":" << std::endl;
    
    std::cout << "    Center: (" << std::fixed << std::setprecision(3) 
              << box.center.x() << ", " << box.center.y() << ", " << box.center.z() << ") mm" << std::endl;
    std::cout << "    Size: (" << box.size.x() << ", " << box.size.y() << ", " << box.size.z() << ") mm" << std::endl;
    std::cout << "    Quaternion: (" << box.quat_x << ", " << box.quat_y 
              << ", " << box.quat_z << ", " << box.quat_w << ")" << std::endl;
    std::cout << "    Type: " << (box.is_axis_aligned ? "AABB" : "OBB") << std::endl;
}

void print_collision_cylinder_debug(const CollisionCylinder& cylinder, int index) {
    std::cout << "  Cylinder";
    if (index >= 0) std::cout << " [" << index << "]";
    std::cout << ":" << std::endl;
    
    std::cout << "    Center: (" << std::fixed << std::setprecision(3) 
              << cylinder.center.x() << ", " << cylinder.center.y() << ", " << cylinder.center.z() << ") mm" << std::endl;
    std::cout << "    Radius: " << cylinder.radius << " mm" << std::endl;
    std::cout << "    Height: " << cylinder.height << " mm" << std::endl;
    std::cout << "    Quaternion: (" << cylinder.quat_x << ", " << cylinder.quat_y 
              << ", " << cylinder.quat_z << ", " << cylinder.quat_w << ")" << std::endl;
    std::cout << "    Shape: " << (cylinder.is_circular ? "Circular" : "Elliptical") << std::endl;
    std::cout << "    Alignment: " << (cylinder.is_axis_aligned ? "Z-aligned" : "Rotated") << std::endl;
}

// === JSON EXPORT FUNCTIONS ===

std::string collision_world_to_json_string(const CollisionWorld& world) {
    std::stringstream json;
    json << std::fixed << std::setprecision(3);
    
    json << "{\n";
    json << "  \"metadata\": {\n";
    json << "    \"total_primitives\": " << world.total_primitives() << ",\n";
    json << "    \"sphere_count\": " << world.spheres.size() << ",\n";
    json << "    \"box_count\": " << world.boxes.size() << ",\n";
    json << "    \"cylinder_count\": " << world.cylinders.size() << "\n";
    json << "  },\n";
    
    // Spheres
    json << "  \"spheres\": [\n";
    for (size_t i = 0; i < world.spheres.size(); ++i) {
        const auto& sphere = world.spheres[i];
        json << "    {\n";
        json << "      \"center\": [" << sphere.center.x() << ", " << sphere.center.y() << ", " << sphere.center.z() << "],\n";
        json << "      \"radius\": " << sphere.radius << ",\n";
        json << "      \"quaternion\": [" << sphere.quat_x << ", " << sphere.quat_y << ", " << sphere.quat_z << ", " << sphere.quat_w << "],\n";
        json << "      \"is_uniform_scale\": " << (sphere.is_uniform_scale ? "true" : "false") << "\n";
        json << "    }";
        if (i < world.spheres.size() - 1) json << ",";
        json << "\n";
    }
    json << "  ],\n";
    
    // Boxes
    json << "  \"boxes\": [\n";
    for (size_t i = 0; i < world.boxes.size(); ++i) {
        const auto& box = world.boxes[i];
        json << "    {\n";
        json << "      \"center\": [" << box.center.x() << ", " << box.center.y() << ", " << box.center.z() << "],\n";
        json << "      \"size\": [" << box.size.x() << ", " << box.size.y() << ", " << box.size.z() << "],\n";
        json << "      \"quaternion\": [" << box.quat_x << ", " << box.quat_y << ", " << box.quat_z << ", " << box.quat_w << "],\n";
        json << "      \"is_axis_aligned\": " << (box.is_axis_aligned ? "true" : "false") << "\n";
        json << "    }";
        if (i < world.boxes.size() - 1) json << ",";
        json << "\n";
    }
    json << "  ],\n";
    
    // Cylinders
    json << "  \"cylinders\": [\n";
    for (size_t i = 0; i < world.cylinders.size(); ++i) {
        const auto& cylinder = world.cylinders[i];
        json << "    {\n";
        json << "      \"center\": [" << cylinder.center.x() << ", " << cylinder.center.y() << ", " << cylinder.center.z() << "],\n";
        json << "      \"radius\": " << cylinder.radius << ",\n";
        json << "      \"height\": " << cylinder.height << ",\n";
        json << "      \"quaternion\": [" << cylinder.quat_x << ", " << cylinder.quat_y << ", " << cylinder.quat_z << ", " << cylinder.quat_w << "],\n";
        json << "      \"is_circular\": " << (cylinder.is_circular ? "true" : "false") << ",\n";
        json << "      \"is_axis_aligned\": " << (cylinder.is_axis_aligned ? "true" : "false") << "\n";
        json << "    }";
        if (i < world.cylinders.size() - 1) json << ",";
        json << "\n";
    }
    json << "  ]\n";
    
    json << "}";
    
    return json.str();
}

void save_collision_world_json(const CollisionWorld& world, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file '" << filename << "' for writing" << std::endl;
        return;
    }
    
    file << collision_world_to_json_string(world);
    file.close();
    
    std::cout << "Collision world saved to: " << filename << std::endl;
    std::cout << "  " << world.total_primitives() << " primitives exported" << std::endl;
}

// === ISAAC SIM DATA PARSING ===

CollisionWorld parse_isaac_collision_data(const void* isaac_data) {
    // This will be called from Python bindings with Isaac Sim collision data
    // For now, return test data - we'll add real parsing next
    
    std::cout << "Warning: parse_isaac_collision_data() using fallback test data" << std::endl;
    return create_test_collision_world();
}

// NEW: Parse Isaac Sim collision data from Python dict
CollisionWorld parse_isaac_collision_dict(const std::map<std::string, pybind11::object>& isaac_dict) {
    CollisionWorld world;
    
    try {
        std::cout << "=== Parsing Isaac Sim collision data ===" << std::endl;
        
        // Parse spheres
        if (isaac_dict.find("spheres") != isaac_dict.end()) {
            auto spheres_list = isaac_dict.at("spheres").cast<std::vector<std::map<std::string, pybind11::object>>>();
            
            for (const auto& sphere_dict : spheres_list) {
                CollisionSphere sphere;
                
                // Parse center (tuple → Vector3)
                auto center_tuple = sphere_dict.at("center").cast<std::tuple<double, double, double>>();
                sphere.center = Vector3(std::get<0>(center_tuple), std::get<1>(center_tuple), std::get<2>(center_tuple));
                
                // Parse radius
                sphere.radius = static_cast<float>(sphere_dict.at("radius").cast<double>());
                
                // Parse quaternion (tuple → individual floats)
                auto quat_tuple = sphere_dict.at("quaternion").cast<std::tuple<double, double, double, double>>();
                sphere.quat_x = static_cast<float>(std::get<0>(quat_tuple));
                sphere.quat_y = static_cast<float>(std::get<1>(quat_tuple));
                sphere.quat_z = static_cast<float>(std::get<2>(quat_tuple));
                sphere.quat_w = static_cast<float>(std::get<3>(quat_tuple));
                
                // Parse rotation matrix (list of lists → Matrix3)
                auto rot_matrix_list = sphere_dict.at("rotation_matrix").cast<std::vector<std::vector<double>>>();
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        sphere.rotation_matrix(i, j) = rot_matrix_list[i][j];
                    }
                }
                
                // Parse flags
                sphere.is_uniform_scale = sphere_dict.at("is_uniform_scale").cast<bool>();
                
                world.add_sphere(sphere);
                std::cout << "  Added sphere: center=(" << sphere.center.x() << ", " << sphere.center.y() << ", " << sphere.center.z() << ") radius=" << sphere.radius << std::endl;
            }
        }
        
        // Parse boxes
        if (isaac_dict.find("boxes") != isaac_dict.end()) {
            auto boxes_list = isaac_dict.at("boxes").cast<std::vector<std::map<std::string, pybind11::object>>>();
            
            for (const auto& box_dict : boxes_list) {
                CollisionBox box;
                
                // Parse center (tuple → Vector3)
                auto center_tuple = box_dict.at("center").cast<std::tuple<double, double, double>>();
                box.center = Vector3(std::get<0>(center_tuple), std::get<1>(center_tuple), std::get<2>(center_tuple));
                
                // Parse size (tuple → Vector3)
                auto size_tuple = box_dict.at("size").cast<std::tuple<double, double, double>>();
                box.size = Vector3(std::get<0>(size_tuple), std::get<1>(size_tuple), std::get<2>(size_tuple));
                
                // Parse quaternion (tuple → individual floats)
                auto quat_tuple = box_dict.at("quaternion").cast<std::tuple<double, double, double, double>>();
                box.quat_x = static_cast<float>(std::get<0>(quat_tuple));
                box.quat_y = static_cast<float>(std::get<1>(quat_tuple));
                box.quat_z = static_cast<float>(std::get<2>(quat_tuple));
                box.quat_w = static_cast<float>(std::get<3>(quat_tuple));
                
                // Parse rotation matrix (list of lists → Matrix3)
                auto rot_matrix_list = box_dict.at("rotation_matrix").cast<std::vector<std::vector<double>>>();
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        box.rotation_matrix(i, j) = rot_matrix_list[i][j];
                    }
                }
                
                // Parse flags
                box.is_axis_aligned = box_dict.at("is_axis_aligned").cast<bool>();
                
                world.add_box(box);
                std::cout << "  Added box: center=(" << box.center.x() << ", " << box.center.y() << ", " << box.center.z() << ") size=(" << box.size.x() << ", " << box.size.y() << ", " << box.size.z() << ")" << std::endl;
            }
        }
        
        // Parse cylinders (if any)
        if (isaac_dict.find("cylinders") != isaac_dict.end()) {
            auto cylinders_list = isaac_dict.at("cylinders").cast<std::vector<std::map<std::string, pybind11::object>>>();
            
            for (const auto& cyl_dict : cylinders_list) {
                CollisionCylinder cylinder;
                
                // Parse center (tuple → Vector3)
                auto center_tuple = cyl_dict.at("center").cast<std::tuple<double, double, double>>();
                cylinder.center = Vector3(std::get<0>(center_tuple), std::get<1>(center_tuple), std::get<2>(center_tuple));
                
                // Parse dimensions
                cylinder.radius = static_cast<float>(cyl_dict.at("radius").cast<double>());
                cylinder.height = static_cast<float>(cyl_dict.at("height").cast<double>());
                
                // Parse quaternion (tuple → individual floats)
                auto quat_tuple = cyl_dict.at("quaternion").cast<std::tuple<double, double, double, double>>();
                cylinder.quat_x = static_cast<float>(std::get<0>(quat_tuple));
                cylinder.quat_y = static_cast<float>(std::get<1>(quat_tuple));
                cylinder.quat_z = static_cast<float>(std::get<2>(quat_tuple));
                cylinder.quat_w = static_cast<float>(std::get<3>(quat_tuple));
                
                // Parse rotation matrix (list of lists → Matrix3)
                auto rot_matrix_list = cyl_dict.at("rotation_matrix").cast<std::vector<std::vector<double>>>();
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        cylinder.rotation_matrix(i, j) = rot_matrix_list[i][j];
                    }
                }
                
                // Parse flags
                cylinder.is_circular = cyl_dict.at("is_circular").cast<bool>();
                cylinder.is_axis_aligned = cyl_dict.at("is_axis_aligned").cast<bool>();
                
                world.add_cylinder(cylinder);
                std::cout << "  Added cylinder: center=(" << cylinder.center.x() << ", " << cylinder.center.y() << ", " << cylinder.center.z() << ") radius=" << cylinder.radius << " height=" << cylinder.height << std::endl;
            }
        }
        
        std::cout << "✅ Isaac Sim collision parsing complete: " << world.total_primitives() << " primitives" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error parsing Isaac Sim collision data: " << e.what() << std::endl;
        std::cerr << "Falling back to test collision world" << std::endl;
        return create_test_collision_world();
    }
    
    return world;
}

// === TEST DATA CREATION ===

CollisionWorld create_test_collision_world() {
    CollisionWorld world;
    
    // Add test sphere
    CollisionSphere sphere1(Vector3(100, 50, 200), 75.0f);
    sphere1.is_uniform_scale = true;
    world.add_sphere(sphere1);
    
    // Add test box (AABB)
    CollisionBox box1(Vector3(-150, 100, 300), Vector3(100, 50, 200));
    box1.is_axis_aligned = true;
    world.add_box(box1);
    
    // Add test box (OBB - rotated)
    CollisionBox box2(Vector3(200, -100, 250), Vector3(80, 80, 150));
    box2.is_axis_aligned = false;
    box2.quat_x = 0.0f;
    box2.quat_y = 0.0f; 
    box2.quat_z = 0.383f;  // ~45 degree rotation around Z
    box2.quat_w = 0.924f;
    world.add_box(box2);
    
    // Add test cylinder
    CollisionCylinder cylinder1(Vector3(0, -200, 180), 60.0f, 120.0f);
    cylinder1.is_circular = true;
    cylinder1.is_axis_aligned = true;
    world.add_cylinder(cylinder1);
    
    std::cout << "Created test collision world with " << world.total_primitives() << " primitives" << std::endl;
    
    return world;
}

} // namespace delta