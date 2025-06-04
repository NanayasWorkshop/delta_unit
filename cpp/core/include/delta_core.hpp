#ifndef DELTA_CORE_HPP
#define DELTA_CORE_HPP

/**
 * @file delta_core.hpp
 * @brief Main header for Delta Robot Core Layer
 * 
 * This is the foundation layer that provides:
 * - Basic types and constants
 * - Mathematical utilities  
 * - Error handling and logging
 * - Common data structures
 * 
 * All other layers depend on this core layer.
 * 
 * Usage:
 *   #include <delta_core.hpp>
 * 
 * This will include all core functionality needed for delta robot calculations.
 */

// Core includes in dependency order
#include "delta_core/constants.hpp"
#include "delta_core/types.hpp" 
#include "delta_core/error_handling.hpp"
#include "delta_core/math_core.hpp"

// Version information
#define DELTA_CORE_VERSION_MAJOR 1
#define DELTA_CORE_VERSION_MINOR 0
#define DELTA_CORE_VERSION_PATCH 0

namespace delta::core {

// ============================================================================
// VERSION INFORMATION
// ============================================================================

struct Version {
    int major = DELTA_CORE_VERSION_MAJOR;
    int minor = DELTA_CORE_VERSION_MINOR;
    int patch = DELTA_CORE_VERSION_PATCH;
    
    std::string to_string() const {
        return std::to_string(major) + "." + 
               std::to_string(minor) + "." + 
               std::to_string(patch);
    }
    
    bool is_compatible_with(const Version& other) const {
        // Compatible if major version matches and minor version is >= required
        return major == other.major && minor >= other.minor;
    }
};

// Get current version
inline Version get_version() {
    return Version{};
}

// ============================================================================
// CORE INITIALIZATION
// ============================================================================

/**
 * @brief Initialize the delta core library
 * 
 * This function should be called once before using any delta robot functionality.
 * It sets up logging, validates the environment, and initializes internal state.
 * 
 * @param enable_logging Enable or disable logging (default: false)
 * @param log_level Minimum log level to display (default: INFO)
 * @return true if initialization successful, false otherwise
 */
bool initialize(bool enable_logging = false, 
               error::LogLevel log_level = error::LogLevel::INFO);

/**
 * @brief Shutdown the delta core library
 * 
 * Clean up resources and reset internal state.
 */
void shutdown();

/**
 * @brief Check if the core library is initialized
 * 
 * @return true if initialized, false otherwise
 */
bool is_initialized();

// ============================================================================
// VALIDATION AND TESTING
// ============================================================================

/**
 * @brief Validate the core library installation
 * 
 * Performs comprehensive checks to ensure the library is working correctly:
 * - Eigen integration
 * - Mathematical operations
 * - Error handling
 * - Constants validation
 * 
 * @return Result containing validation status and any error messages
 */
Result<bool> validate_installation();

/**
 * @brief Run core library self-tests
 * 
 * Executes a suite of unit tests for core functionality.
 * 
 * @return Result containing test results and detailed report
 */
Result<std::string> run_self_tests();

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Get information about the current environment
 * 
 * Returns details about:
 * - Eigen version and configuration
 * - Compiler information
 * - Build configuration
 * - Available features
 * 
 * @return String containing environment information
 */
std::string get_environment_info();

/**
 * @brief Get performance benchmarks for core operations
 * 
 * Runs performance tests for key mathematical operations and returns timing data.
 * Useful for validating performance across different platforms.
 * 
 * @param iterations Number of iterations for each benchmark (default: 10000)
 * @return String containing benchmark results
 */
std::string get_performance_benchmarks(int iterations = 10000);

// ============================================================================
// COMMON CONFIGURATION PRESETS
// ============================================================================

namespace presets {
    
    /**
     * @brief Get fast solving configuration
     * 
     * Optimized for speed with relaxed tolerances.
     */
    Configuration get_fast_config();
    
    /**
     * @brief Get precise solving configuration
     * 
     * Optimized for accuracy with tight tolerances.
     */
    Configuration get_precise_config();
    
    /**
     * @brief Get debug configuration
     * 
     * Includes verbose logging and validation checks.
     */
    Configuration get_debug_config();
    
    /**
     * @brief Get default configuration
     * 
     * Balanced settings suitable for most applications.
     */
    Configuration get_default_config();

} // namespace presets

} // namespace delta::core

// ============================================================================
// BACKWARD COMPATIBILITY
// ============================================================================

namespace delta {
    // Expose core functionality at delta namespace level for easier access
    using namespace core;
    
    // Legacy initialization function (deprecated)
    [[deprecated("Use delta::core::initialize() instead")]]
    inline bool init_delta_robot() {
        return core::initialize();
    }
    
    // Legacy validation function (deprecated)
    [[deprecated("Use delta::core::validate_installation() instead")]]
    inline bool verify_installation() {
        auto result = core::validate_installation();
        return result.is_success() && result.value();
    }
}

// ============================================================================
// FEATURE DETECTION MACROS
// ============================================================================

// Define feature availability macros
#define DELTA_HAS_EIGEN 1
#define DELTA_HAS_ERROR_HANDLING 1
#define DELTA_HAS_LOGGING 1
#define DELTA_HAS_PERFORMANCE_TIMING 1

// C++ standard detection
#if __cplusplus >= 201703L
    #define DELTA_HAS_CPP17 1
#endif

#if __cplusplus >= 202002L
    #define DELTA_HAS_CPP20 1
#endif

// Compiler-specific optimizations
#ifdef __GNUC__
    #define DELTA_FORCE_INLINE __attribute__((always_inline)) inline
#elif defined(_MSC_VER)
    #define DELTA_FORCE_INLINE __forceinline
#else
    #define DELTA_FORCE_INLINE inline
#endif

// Platform detection
#ifdef _WIN32
    #define DELTA_PLATFORM_WINDOWS 1
#elif defined(__linux__)
    #define DELTA_PLATFORM_LINUX 1
#elif defined(__APPLE__)
    #define DELTA_PLATFORM_MACOS 1
#endif

#endif // DELTA_CORE_HPP