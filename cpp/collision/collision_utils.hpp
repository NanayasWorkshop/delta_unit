#ifndef DELTA_COLLISION_UTILS_HPP
#define DELTA_COLLISION_UTILS_HPP

#include "../core/math_utils.hpp"
#include <vector>
#include <string>
#include <map>
#include <pybind11/pybind11.h>

namespace delta {

// === COLLISION PRIMITIVES ===

struct CollisionSphere {
    Vector3 center;                    // Center position (mm)
    float radius;                      // Radius (mm)
    Matrix3 rotation_matrix;           // 3x3 rotation matrix
    float quat_x, quat_y, quat_z, quat_w;  // Quaternion (x,y,z,w)
    bool is_uniform_scale;             // Optimization flag from Isaac Sim
    
    CollisionSphere() : center(0,0,0), radius(0), rotation_matrix(Matrix3::Identity()),
                       quat_x(0), quat_y(0), quat_z(0), quat_w(1), is_uniform_scale(true) {}
                       
    CollisionSphere(const Vector3& c, float r) 
        : center(c), radius(r), rotation_matrix(Matrix3::Identity()),
          quat_x(0), quat_y(0), quat_z(0), quat_w(1), is_uniform_scale(true) {}
};

struct CollisionBox {
    Vector3 center;                    // Center position (mm)
    Vector3 size;                      // Dimensions (width, height, depth) (mm)
    Matrix3 rotation_matrix;           // 3x3 rotation matrix
    float quat_x, quat_y, quat_z, quat_w;  // Quaternion (x,y,z,w)
    bool is_axis_aligned;              // AABB vs OBB optimization flag
    
    CollisionBox() : center(0,0,0), size(1,1,1), rotation_matrix(Matrix3::Identity()),
                    quat_x(0), quat_y(0), quat_z(0), quat_w(1), is_axis_aligned(true) {}
                    
    CollisionBox(const Vector3& c, const Vector3& s) 
        : center(c), size(s), rotation_matrix(Matrix3::Identity()),
          quat_x(0), quat_y(0), quat_z(0), quat_w(1), is_axis_aligned(true) {}
};

struct CollisionCylinder {
    Vector3 center;                    // Center position (mm)
    float radius;                      // Radius (mm)
    float height;                      // Height (mm)
    Matrix3 rotation_matrix;           // 3x3 rotation matrix
    float quat_x, quat_y, quat_z, quat_w;  // Quaternion (x,y,z,w)
    bool is_circular;                  // Circular vs elliptical
    bool is_axis_aligned;              // Z-aligned optimization flag
    
    CollisionCylinder() : center(0,0,0), radius(1), height(1), rotation_matrix(Matrix3::Identity()),
                         quat_x(0), quat_y(0), quat_z(0), quat_w(1), is_circular(true), is_axis_aligned(true) {}
                         
    CollisionCylinder(const Vector3& c, float r, float h) 
        : center(c), radius(r), height(h), rotation_matrix(Matrix3::Identity()),
          quat_x(0), quat_y(0), quat_z(0), quat_w(1), is_circular(true), is_axis_aligned(true) {}
};

// === COLLISION WORLD ===

struct CollisionWorld {
    std::vector<CollisionSphere> spheres;
    std::vector<CollisionBox> boxes;
    std::vector<CollisionCylinder> cylinders;
    
    // Statistics
    bool is_empty() const {
        return spheres.empty() && boxes.empty() && cylinders.empty();
    }
    
    size_t total_primitives() const {
        return spheres.size() + boxes.size() + cylinders.size();
    }
    
    void clear() {
        spheres.clear();
        boxes.clear();
        cylinders.clear();
    }
    
    // Add primitives
    void add_sphere(const CollisionSphere& sphere) {
        spheres.push_back(sphere);
    }
    
    void add_box(const CollisionBox& box) {
        boxes.push_back(box);
    }
    
    void add_cylinder(const CollisionCylinder& cylinder) {
        cylinders.push_back(cylinder);
    }
};

// === DEBUGGING & EXPORT FUNCTIONS ===

// Console debug output
void print_collision_world_debug(const CollisionWorld& world);
void print_collision_sphere_debug(const CollisionSphere& sphere, int index = -1);
void print_collision_box_debug(const CollisionBox& box, int index = -1);
void print_collision_cylinder_debug(const CollisionCylinder& cylinder, int index = -1);

// JSON export for visualization
void save_collision_world_json(const CollisionWorld& world, const std::string& filename);
std::string collision_world_to_json_string(const CollisionWorld& world);

// === ISAAC SIM DATA PARSING ===

// Parse collision data from Isaac Sim (Phase 1 format)
CollisionWorld parse_isaac_collision_data(const void* isaac_data);

// NEW: Parse Isaac Sim collision data from Python dict
CollisionWorld parse_isaac_collision_dict(const std::map<std::string, pybind11::object>& isaac_dict);

// Test data creation for development
CollisionWorld create_test_collision_world();

} // namespace delta

#endif // DELTA_COLLISION_UTILS_HPP