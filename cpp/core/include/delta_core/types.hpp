#ifndef DELTA_CORE_TYPES_HPP
#define DELTA_CORE_TYPES_HPP

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <string>
#include <chrono>

namespace delta::core {

// Eigen type aliases for consistency across the project
using Vector3 = Eigen::Vector3d;
using Matrix3 = Eigen::Matrix3d;
using Matrix4 = Eigen::Matrix4d;
using Quaternion = Eigen::Quaterniond;

// Common result wrapper for error handling
template<typename T>
struct Result {
    T data;
    bool success;
    std::string error_message;
    
    // Success constructor
    Result(T&& d) : data(std::move(d)), success(true) {}
    Result(const T& d) : data(d), success(true) {}
    
    // Error constructor
    Result(const std::string& error) : success(false), error_message(error) {}
    
    // Convenience methods
    bool is_success() const { return success; }
    bool is_error() const { return !success; }
    
    // Value access (throws if error)
    T& value() { 
        if (!success) throw std::runtime_error("Accessing value of error result: " + error_message);
        return data; 
    }
    const T& value() const { 
        if (!success) throw std::runtime_error("Accessing value of error result: " + error_message);
        return data; 
    }
    
    // Safe value access
    T value_or(const T& default_value) const {
        return success ? data : default_value;
    }
};

// Performance timing utilities
struct Timer {
    std::chrono::high_resolution_clock::time_point start_time;
    
    Timer() { reset(); }
    
    void reset() {
        start_time = std::chrono::high_resolution_clock::now();
    }
    
    double elapsed_ms() const {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        return duration.count() / 1000.0;  // Convert to milliseconds
    }
    
    double elapsed_seconds() const {
        return elapsed_ms() / 1000.0;
    }
};

// Common configuration structure
struct Configuration {
    double tolerance = 0.01;
    int max_iterations = 100;
    bool enable_constraints = true;
    bool verbose_logging = false;
    bool track_performance = false;
    
    // Validation
    bool is_valid() const {
        return tolerance > 0.0 && tolerance < 1.0 && 
               max_iterations > 0 && max_iterations <= 10000;
    }
};

// Coordinate frame representation
struct CoordinateFrame {
    Vector3 origin;
    Vector3 u_axis;  // X-axis equivalent
    Vector3 v_axis;  // Y-axis equivalent  
    Vector3 w_axis;  // Z-axis equivalent
    
    CoordinateFrame(const Vector3& orig = Vector3(0,0,0),
                    const Vector3& u = Vector3(1,0,0),
                    const Vector3& v = Vector3(0,1,0),
                    const Vector3& w = Vector3(0,0,1))
        : origin(orig), u_axis(u), v_axis(v), w_axis(w) {}
    
    // Validation
    bool is_orthonormal(double tolerance = 1e-6) const {
        // Check if axes are unit vectors
        if (std::abs(u_axis.norm() - 1.0) > tolerance ||
            std::abs(v_axis.norm() - 1.0) > tolerance ||
            std::abs(w_axis.norm() - 1.0) > tolerance) {
            return false;
        }
        
        // Check if axes are orthogonal
        if (std::abs(u_axis.dot(v_axis)) > tolerance ||
            std::abs(v_axis.dot(w_axis)) > tolerance ||
            std::abs(w_axis.dot(u_axis)) > tolerance) {
            return false;
        }
        
        return true;
    }
    
    // Create transformation matrix
    Matrix4 to_matrix() const {
        Matrix4 matrix = Matrix4::Identity();
        matrix.block<3,3>(0,0).col(0) = u_axis;
        matrix.block<3,3>(0,0).col(1) = v_axis;
        matrix.block<3,3>(0,0).col(2) = w_axis;
        matrix.block<3,1>(0,3) = origin;
        return matrix;
    }
};

// Joint types for robot structure
enum class JointType {
    FIXED_BASE,      // Base connector (0,0,0)
    SPHERICAL_120,   // Spherical joint with 120° constraint
    END_EFFECTOR     // Final end-effector point
};

// Solver status enumeration
enum class SolverStatus {
    SUCCESS,              // Successfully converged
    MAX_ITERATIONS,       // Reached maximum iterations
    STALLED,             // Algorithm stalled (no progress)
    INVALID_TARGET,      // Target is unreachable
    CONFIGURATION_ERROR, // Invalid configuration
    NUMERICAL_ERROR      // Numerical instability
};

// Convert solver status to string
inline std::string to_string(SolverStatus status) {
    switch (status) {
        case SolverStatus::SUCCESS: return "SUCCESS";
        case SolverStatus::MAX_ITERATIONS: return "MAX_ITERATIONS";
        case SolverStatus::STALLED: return "STALLED";
        case SolverStatus::INVALID_TARGET: return "INVALID_TARGET";
        case SolverStatus::CONFIGURATION_ERROR: return "CONFIGURATION_ERROR";
        case SolverStatus::NUMERICAL_ERROR: return "NUMERICAL_ERROR";
        default: return "UNKNOWN";
    }
}

// Performance metrics structure
struct PerformanceMetrics {
    double solve_time_ms = 0.0;
    int total_iterations = 0;
    int function_calls = 0;
    double final_error = 0.0;
    SolverStatus status = SolverStatus::SUCCESS;
    
    void reset() {
        solve_time_ms = 0.0;
        total_iterations = 0;
        function_calls = 0;
        final_error = 0.0;
        status = SolverStatus::SUCCESS;
    }
};

} // namespace delta::core

// Backward compatibility aliases (to be removed in future versions)
namespace delta {
    using Vector3 = core::Vector3;
    using Matrix3 = core::Matrix3;
    using Matrix4 = core::Matrix4;
    using CoordinateFrame = core::CoordinateFrame;
    using JointType = core::JointType;
}

#endif // DELTA_CORE_TYPES_HPP