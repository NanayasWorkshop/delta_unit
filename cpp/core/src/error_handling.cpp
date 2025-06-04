#include "delta_core/error_handling.hpp"
#include "delta_core/constants.hpp"
#include <iostream>
#include <iomanip>
#include <ctime>

namespace delta::core::error {

// ============================================================================
// ERROR CODE STRING CONVERSION
// ============================================================================

std::string to_string(ErrorCode code) {
    switch (code) {
        case ErrorCode::SUCCESS: return "SUCCESS";
        
        // Configuration errors
        case ErrorCode::INVALID_TOLERANCE: return "INVALID_TOLERANCE";
        case ErrorCode::INVALID_ITERATIONS: return "INVALID_ITERATIONS";
        case ErrorCode::INVALID_SEGMENTS: return "INVALID_SEGMENTS";
        case ErrorCode::INVALID_CONFIGURATION: return "INVALID_CONFIGURATION";
        
        // Mathematical errors
        case ErrorCode::DIVISION_BY_ZERO: return "DIVISION_BY_ZERO";
        case ErrorCode::INVALID_VECTOR: return "INVALID_VECTOR";
        case ErrorCode::INVALID_MATRIX: return "INVALID_MATRIX";
        case ErrorCode::NUMERICAL_INSTABILITY: return "NUMERICAL_INSTABILITY";
        
        // Solver errors
        case ErrorCode::MAX_ITERATIONS_REACHED: return "MAX_ITERATIONS_REACHED";
        case ErrorCode::SOLVER_STALLED: return "SOLVER_STALLED";
        case ErrorCode::TARGET_UNREACHABLE: return "TARGET_UNREACHABLE";
        case ErrorCode::INVALID_INITIAL_STATE: return "INVALID_INITIAL_STATE";
        
        // Constraint errors
        case ErrorCode::JOINT_LIMIT_EXCEEDED: return "JOINT_LIMIT_EXCEEDED";
        case ErrorCode::WORKSPACE_VIOLATION: return "WORKSPACE_VIOLATION";
        case ErrorCode::CONE_CONSTRAINT_VIOLATION: return "CONE_CONSTRAINT_VIOLATION";
        
        // Validation errors
        case ErrorCode::INVALID_INPUT: return "INVALID_INPUT";
        case ErrorCode::NULL_POINTER: return "NULL_POINTER";
        case ErrorCode::INVALID_DIMENSIONS: return "INVALID_DIMENSIONS";
        case ErrorCode::OUT_OF_RANGE: return "OUT_OF_RANGE";
        
        default: return "UNKNOWN_ERROR";
    }
}

// ============================================================================
// ERROR HANDLER IMPLEMENTATION
// ============================================================================

void ErrorHandler::validate_tolerance(double tolerance) {
    if (tolerance <= 0.0 || tolerance >= 1.0) {
        throw ConfigurationError("Invalid tolerance: " + std::to_string(tolerance) + 
                               " (must be between 0 and 1)");
    }
}

void ErrorHandler::validate_iterations(int iterations) {
    if (iterations <= 0 || iterations > 10000) {
        throw ConfigurationError("Invalid iterations: " + std::to_string(iterations) + 
                               " (must be between 1 and 10000)");
    }
}

void ErrorHandler::validate_segments(int segments) {
    if (segments <= 0 || segments > 100) {
        throw ConfigurationError("Invalid segments: " + std::to_string(segments) + 
                               " (must be between 1 and 100)");
    }
}

void ErrorHandler::validate_vector(const Vector3& v, const std::string& name) {
    if (!v.allFinite()) {
        throw ValidationError("Invalid " + name + ": contains NaN or infinite values");
    }
}

void ErrorHandler::validate_unit_vector(const Vector3& v, const std::string& name, double tolerance) {
    validate_vector(v, name);
    
    double norm = v.norm();
    if (std::abs(norm - 1.0) > tolerance) {
        throw ValidationError("Invalid " + name + ": not a unit vector (norm = " + 
                             std::to_string(norm) + ")");
    }
}

void ErrorHandler::validate_matrix(const Matrix3& m, const std::string& name) {
    if (!m.allFinite()) {
        throw ValidationError("Invalid " + name + ": contains NaN or infinite values");
    }
}

void ErrorHandler::validate_transformation_matrix(const Matrix4& m, const std::string& name) {
    if (!m.allFinite()) {
        throw ValidationError("Invalid " + name + ": contains NaN or infinite values");
    }
    
    // Check that bottom row is [0, 0, 0, 1]
    if (std::abs(m(3,0)) > math::EPSILON || 
        std::abs(m(3,1)) > math::EPSILON || 
        std::abs(m(3,2)) > math::EPSILON || 
        std::abs(m(3,3) - 1.0) > math::EPSILON) {
        throw ValidationError("Invalid " + name + ": bottom row must be [0, 0, 0, 1]");
    }
}

void ErrorHandler::validate_range(double value, double min, double max, const std::string& name) {
    if (value < min || value > max) {
        throw ValidationError(name + " = " + std::to_string(value) + 
                             " is out of range [" + std::to_string(min) + ", " + 
                             std::to_string(max) + "]");
    }
}

void ErrorHandler::validate_positive(double value, const std::string& name) {
    if (value <= 0.0) {
        throw ValidationError(name + " = " + std::to_string(value) + 
                             " must be positive");
    }
}

void ErrorHandler::validate_non_negative(double value, const std::string& name) {
    if (value < 0.0) {
        throw ValidationError(name + " = " + std::to_string(value) + 
                             " must be non-negative");
    }
}

// ============================================================================
// LOGGER IMPLEMENTATION
// ============================================================================

// Static member definitions
LogLevel Logger::current_level_ = LogLevel::INFO;
bool Logger::enabled_ = false;

std::string get_timestamp() {
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

std::string log_level_to_string(LogLevel level) {
    switch (level) {
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO: return "INFO";
        case LogLevel::WARNING: return "WARNING";
        case LogLevel::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

void Logger::debug(const std::string& message) {
    if (enabled_ && current_level_ <= LogLevel::DEBUG) {
        std::cout << "[" << get_timestamp() << "] [DEBUG] " << message << std::endl;
    }
}

void Logger::info(const std::string& message) {
    if (enabled_ && current_level_ <= LogLevel::INFO) {
        std::cout << "[" << get_timestamp() << "] [INFO] " << message << std::endl;
    }
}

void Logger::warning(const std::string& message) {
    if (enabled_ && current_level_ <= LogLevel::WARNING) {
        std::cout << "[" << get_timestamp() << "] [WARNING] " << message << std::endl;
    }
}

void Logger::error(const std::string& message) {
    if (enabled_ && current_level_ <= LogLevel::ERROR) {
        std::cerr << "[" << get_timestamp() << "] [ERROR] " << message << std::endl;
    }
}

} // namespace delta::core::error