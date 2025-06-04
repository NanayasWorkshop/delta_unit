#ifndef DELTA_CORE_ERROR_HANDLING_HPP
#define DELTA_CORE_ERROR_HANDLING_HPP

#include <stdexcept>
#include <string>
#include <sstream>

namespace delta::core::error {

// ============================================================================
// CUSTOM EXCEPTION TYPES
// ============================================================================

// Base exception for all delta robot errors
class DeltaRobotException : public std::runtime_error {
public:
    explicit DeltaRobotException(const std::string& message) 
        : std::runtime_error("Delta Robot Error: " + message) {}
};

// Configuration-related errors
class ConfigurationError : public DeltaRobotException {
public:
    explicit ConfigurationError(const std::string& message)
        : DeltaRobotException("Configuration Error: " + message) {}
};

// Mathematical/numerical errors
class NumericalError : public DeltaRobotException {
public:
    explicit NumericalError(const std::string& message)
        : DeltaRobotException("Numerical Error: " + message) {}
};

// Solver convergence errors
class ConvergenceError : public DeltaRobotException {
public:
    explicit ConvergenceError(const std::string& message)
        : DeltaRobotException("Convergence Error: " + message) {}
};

// Constraint violation errors
class ConstraintError : public DeltaRobotException {
public:
    explicit ConstraintError(const std::string& message)
        : DeltaRobotException("Constraint Error: " + message) {}
};

// Input validation errors
class ValidationError : public DeltaRobotException {
public:
    explicit ValidationError(const std::string& message)
        : DeltaRobotException("Validation Error: " + message) {}
};

// ============================================================================
// ERROR CODES
// ============================================================================

enum class ErrorCode {
    SUCCESS = 0,
    
    // Configuration errors (100-199)
    INVALID_TOLERANCE = 100,
    INVALID_ITERATIONS = 101,
    INVALID_SEGMENTS = 102,
    INVALID_CONFIGURATION = 103,
    
    // Mathematical errors (200-299)
    DIVISION_BY_ZERO = 200,
    INVALID_VECTOR = 201,
    INVALID_MATRIX = 202,
    NUMERICAL_INSTABILITY = 203,
    
    // Solver errors (300-399)
    MAX_ITERATIONS_REACHED = 300,
    SOLVER_STALLED = 301,
    TARGET_UNREACHABLE = 302,
    INVALID_INITIAL_STATE = 303,
    
    // Constraint errors (400-499)
    JOINT_LIMIT_EXCEEDED = 400,
    WORKSPACE_VIOLATION = 401,
    CONE_CONSTRAINT_VIOLATION = 402,
    
    // Validation errors (500-599)
    INVALID_INPUT = 500,
    NULL_POINTER = 501,
    INVALID_DIMENSIONS = 502,
    OUT_OF_RANGE = 503
};

// Convert error code to string
std::string to_string(ErrorCode code);

// ============================================================================
// ERROR HANDLING UTILITIES
// ============================================================================

class ErrorHandler {
public:
    // Validation functions
    static void validate_tolerance(double tolerance);
    static void validate_iterations(int iterations);
    static void validate_segments(int segments);
    static void validate_vector(const Vector3& v, const std::string& name = "vector");
    static void validate_unit_vector(const Vector3& v, const std::string& name = "unit vector", 
                                   double tolerance = 1e-6);
    static void validate_matrix(const Matrix3& m, const std::string& name = "matrix");
    static void validate_transformation_matrix(const Matrix4& m, const std::string& name = "transformation");
    
    // Range validation
    static void validate_range(double value, double min, double max, const std::string& name);
    static void validate_positive(double value, const std::string& name);
    static void validate_non_negative(double value, const std::string& name);
    
    // Pointer validation
    template<typename T>
    static void validate_not_null(const T* ptr, const std::string& name) {
        if (ptr == nullptr) {
            throw ValidationError("Null pointer: " + name);
        }
    }
    
    // Container validation
    template<typename Container>
    static void validate_not_empty(const Container& container, const std::string& name) {
        if (container.empty()) {
            throw ValidationError("Empty container: " + name);
        }
    }
    
    template<typename Container>
    static void validate_size(const Container& container, size_t expected_size, const std::string& name) {
        if (container.size() != expected_size) {
            std::ostringstream oss;
            oss << "Invalid size for " << name << ": expected " << expected_size 
                << ", got " << container.size();
            throw ValidationError(oss.str());
        }
    }
};

// ============================================================================
// ASSERTION MACROS
// ============================================================================

#ifdef DELTA_DEBUG
    #define DELTA_ASSERT(condition, message) \
        do { \
            if (!(condition)) { \
                std::ostringstream oss; \
                oss << "Assertion failed: " << #condition << " - " << message \
                    << " (file: " << __FILE__ << ", line: " << __LINE__ << ")"; \
                throw DeltaRobotException(oss.str()); \
            } \
        } while(0)
    
    #define DELTA_ASSERT_MSG(condition, message) DELTA_ASSERT(condition, message)
#else
    #define DELTA_ASSERT(condition, message) ((void)0)
    #define DELTA_ASSERT_MSG(condition, message) ((void)0)
#endif

// Runtime checks (always enabled)
#define DELTA_CHECK(condition, message) \
    do { \
        if (!(condition)) { \
            throw DeltaRobotException(std::string("Check failed: ") + message); \
        } \
    } while(0)

#define DELTA_CHECK_MSG(condition, message) DELTA_CHECK(condition, message)

// ============================================================================
// LOGGING UTILITIES
// ============================================================================

enum class LogLevel {
    DEBUG = 0,
    INFO = 1,
    WARNING = 2,
    ERROR = 3
};

class Logger {
private:
    static LogLevel current_level_;
    static bool enabled_;
    
public:
    static void set_level(LogLevel level) { current_level_ = level; }
    static void enable(bool enabled = true) { enabled_ = enabled; }
    static bool is_enabled() { return enabled_; }
    
    static void debug(const std::string& message);
    static void info(const std::string& message);
    static void warning(const std::string& message);
    static void error(const std::string& message);
    
    // Template for formatted logging
    template<typename... Args>
    static void debug_f(const std::string& format, Args&&... args) {
        if (enabled_ && current_level_ <= LogLevel::DEBUG) {
            debug(format_string(format, std::forward<Args>(args)...));
        }
    }
    
    template<typename... Args>
    static void info_f(const std::string& format, Args&&... args) {
        if (enabled_ && current_level_ <= LogLevel::INFO) {
            info(format_string(format, std::forward<Args>(args)...));
        }
    }
    
    template<typename... Args>
    static void warning_f(const std::string& format, Args&&... args) {
        if (enabled_ && current_level_ <= LogLevel::WARNING) {
            warning(format_string(format, std::forward<Args>(args)...));
        }
    }
    
    template<typename... Args>
    static void error_f(const std::string& format, Args&&... args) {
        if (enabled_ && current_level_ <= LogLevel::ERROR) {
            error(format_string(format, std::forward<Args>(args)...));
        }
    }

private:
    template<typename... Args>
    static std::string format_string(const std::string& format, Args&&... args) {
        std::ostringstream oss;
        format_impl(oss, format, std::forward<Args>(args)...);
        return oss.str();
    }
    
    static void format_impl(std::ostringstream& oss, const std::string& format) {
        oss << format;
    }
    
    template<typename T, typename... Args>
    static void format_impl(std::ostringstream& oss, const std::string& format, T&& value, Args&&... args) {
        auto pos = format.find("{}");
        if (pos != std::string::npos) {
            oss << format.substr(0, pos) << value;
            format_impl(oss, format.substr(pos + 2), std::forward<Args>(args)...);
        } else {
            oss << format;
        }
    }
};

// ============================================================================
// CONVENIENCE MACROS FOR LOGGING
// ============================================================================

#ifdef DELTA_DEBUG
    #define DELTA_LOG_DEBUG(message) delta::core::error::Logger::debug(message)
    #define DELTA_LOG_DEBUG_F(format, ...) delta::core::error::Logger::debug_f(format, __VA_ARGS__)
#else
    #define DELTA_LOG_DEBUG(message) ((void)0)
    #define DELTA_LOG_DEBUG_F(format, ...) ((void)0)
#endif

#define DELTA_LOG_INFO(message) delta::core::error::Logger::info(message)
#define DELTA_LOG_INFO_F(format, ...) delta::core::error::Logger::info_f(format, __VA_ARGS__)

#define DELTA_LOG_WARNING(message) delta::core::error::Logger::warning(message)
#define DELTA_LOG_WARNING_F(format, ...) delta::core::error::Logger::warning_f(format, __VA_ARGS__)

#define DELTA_LOG_ERROR(message) delta::core::error::Logger::error(message)
#define DELTA_LOG_ERROR_F(format, ...) delta::core::error::Logger::error_f(format, __VA_ARGS__)

// ============================================================================
// RESULT HELPERS
// ============================================================================

// Helper functions for creating Result objects with proper error handling
template<typename T>
Result<T> make_success(T&& value) {
    return Result<T>(std::forward<T>(value));
}

template<typename T>
Result<T> make_error(ErrorCode code, const std::string& message = "") {
    std::string full_message = to_string(code);
    if (!message.empty()) {
        full_message += ": " + message;
    }
    return Result<T>(full_message);
}

template<typename T>
Result<T> make_error(const std::string& message) {
    return Result<T>(message);
}

// Safe execution wrapper
template<typename Func>
auto safe_execute(Func&& func) -> Result<decltype(func())> {
    try {
        return make_success(func());
    } catch (const DeltaRobotException& e) {
        return make_error<decltype(func())>(e.what());
    } catch (const std::exception& e) {
        return make_error<decltype(func())>("Unexpected error: " + std::string(e.what()));
    } catch (...) {
        return make_error<decltype(func())>("Unknown error occurred");
    }
}

} // namespace delta::core::error

// Make error types available at delta namespace level for convenience
namespace delta {
    using DeltaRobotException = core::error::DeltaRobotException;
    using ConfigurationError = core::error::ConfigurationError;
    using NumericalError = core::error::NumericalError;
    using ConvergenceError = core::error::ConvergenceError;
    using ConstraintError = core::error::ConstraintError;
    using ValidationError = core::error::ValidationError;
    using ErrorCode = core::error::ErrorCode;
}

#endif // DELTA_CORE_ERROR_HANDLING_HPP