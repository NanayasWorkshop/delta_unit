#include "delta_core.hpp"
#include <sstream>
#include <chrono>
#include <random>

namespace delta::core {

// ============================================================================
// STATIC VARIABLES
// ============================================================================

static bool g_initialized = false;

// ============================================================================
// CORE INITIALIZATION IMPLEMENTATION
// ============================================================================

bool initialize(bool enable_logging, error::LogLevel log_level) {
    if (g_initialized) {
        return true;  // Already initialized
    }
    
    try {
        // Setup logging
        error::Logger::enable(enable_logging);
        error::Logger::set_level(log_level);
        
        if (enable_logging) {
            error::Logger::info("Initializing Delta Robot Core Library v" + get_version().to_string());
        }
        
        // Validate Eigen installation
        Vector3 test_vector(1.0, 2.0, 3.0);
        if (!test_vector.allFinite()) {
            throw error::NumericalError("Eigen Vector3 validation failed");
        }
        
        Matrix4 test_matrix = Matrix4::Identity();
        if (test_matrix(3,3) != 1.0) {
            throw error::NumericalError("Eigen Matrix4 validation failed");
        }
        
        // Validate base positions
        if (!math::BasePositions::validate_base_triangle()) {
            throw error::ConfigurationError("Base triangle validation failed");
        }
        
        g_initialized = true;
        
        if (enable_logging) {
            error::Logger::info("Delta Robot Core Library initialized successfully");
        }
        
        return true;
        
    } catch (const std::exception& e) {
        if (enable_logging) {
            error::Logger::error("Failed to initialize Delta Robot Core Library: " + std::string(e.what()));
        }
        return false;
    }
}

void shutdown() {
    if (!g_initialized) {
        return;  // Already shutdown
    }
    
    error::Logger::info("Shutting down Delta Robot Core Library");
    
    // Reset logging
    error::Logger::enable(false);
    
    g_initialized = false;
}

bool is_initialized() {
    return g_initialized;
}

// ============================================================================
// VALIDATION AND TESTING IMPLEMENTATION
// ============================================================================

Result<bool> validate_installation() {
    try {
        std::ostringstream validation_report;
        validation_report << "Delta Robot Core Library Validation Report\n";
        validation_report << "==========================================\n\n";
        
        // 1. Version check
        Version version = get_version();
        validation_report << "Version: " << version.to_string() << "\n";
        
        // 2. Eigen integration test
        validation_report << "Testing Eigen integration...\n";
        Vector3 v1(1.0, 2.0, 3.0);
        Vector3 v2(4.0, 5.0, 6.0);
        double dot_product = v1.dot(v2);
        Vector3 cross_product = v1.cross(v2);
        
        if (!v1.allFinite() || !v2.allFinite() || !cross_product.allFinite()) {
            return error::make_error<bool>("Eigen vector operations failed");
        }
        
        validation_report << "  ✓ Vector operations working\n";
        
        Matrix3 rotation = Matrix3::Identity();
        if (!rotation.allFinite() || std::abs(rotation.determinant() - 1.0) > math::EPSILON) {
            return error::make_error<bool>("Eigen matrix operations failed");
        }
        
        validation_report << "  ✓ Matrix operations working\n";
        
        // 3. Mathematical utilities test
        validation_report << "Testing mathematical utilities...\n";
        
        double angle = math::VectorOps::angle_between(v1, v2);
        if (angle < 0 || angle > math::PI) {
            return error::make_error<bool>("Vector angle calculation failed");
        }
        
        validation_report << "  ✓ Vector operations working\n";
        
        Vector3 normalized = math::VectorOps::safe_normalize(v1);
        if (!math::VectorOps::is_unit_vector(normalized, 1e-6)) {
            return error::make_error<bool>("Vector normalization failed");
        }
        
        validation_report << "  ✓ Vector normalization working\n";
        
        // 4. Base positions test
        validation_report << "Testing base positions...\n";
        
        Vector3 base_A = math::BasePositions::get_base_position_A();
        Vector3 base_B = math::BasePositions::get_base_position_B();
        Vector3 base_C = math::BasePositions::get_base_position_C();
        
        if (!base_A.allFinite() || !base_B.allFinite() || !base_C.allFinite()) {
            return error::make_error<bool>("Base position calculation failed");
        }
        
        if (!math::BasePositions::validate_base_triangle()) {
            return error::make_error<bool>("Base triangle validation failed");
        }
        
        validation_report << "  ✓ Base positions valid\n";
        
        // 5. Constants validation
        validation_report << "Testing constants...\n";
        
        if (robot::RADIUS <= 0 || robot::MIN_HEIGHT <= 0 || robot::WORKING_HEIGHT <= 0) {
            return error::make_error<bool>("Invalid robot constants");
        }
        
        if (fabrik::TOLERANCE <= 0 || fabrik::MAX_ITERATIONS <= 0) {
            return error::make_error<bool>("Invalid FABRIK constants");
        }
        
        validation_report << "  ✓ Constants valid\n";
        
        // 6. Error handling test
        validation_report << "Testing error handling...\n";
        
        try {
            error::ErrorHandler::validate_tolerance(-1.0);
            return error::make_error<bool>("Error handling not working - should have thrown exception");
        } catch (const error::ConfigurationError&) {
            // Expected behavior
            validation_report << "  ✓ Error handling working\n";
        }
        
        validation_report << "\nValidation completed successfully!\n";
        error::Logger::info(validation_report.str());
        
        return error::make_success(true);
        
    } catch (const std::exception& e) {
        return error::make_error<bool>("Validation failed: " + std::string(e.what()));
    }
}

Result<std::string> run_self_tests() {
    try {
        std::ostringstream test_report;
        test_report << "Delta Robot Core Library Self-Test Report\n";
        test_report << "=========================================\n\n";
        
        int total_tests = 0;
        int passed_tests = 0;
        
        // Test 1: Vector operations
        test_report << "Test 1: Vector Operations\n";
        total_tests++;
        
        Vector3 v1(3.0, 4.0, 0.0);
        Vector3 v2(1.0, 0.0, 0.0);
        
        double expected_magnitude = 5.0;
        double actual_magnitude = v1.norm();
        
        if (std::abs(actual_magnitude - expected_magnitude) < math::EPSILON) {
            test_report << "  ✓ Vector magnitude calculation\n";
            passed_tests++;
        } else {
            test_report << "  ✗ Vector magnitude calculation failed\n";
        }
        
        // Test 2: Angle calculations
        test_report << "\nTest 2: Angle Calculations\n";
        total_tests++;
        
        Vector3 x_axis(1.0, 0.0, 0.0);
        Vector3 y_axis(0.0, 1.0, 0.0);
        double angle = math::VectorOps::angle_between(x_axis, y_axis);
        double expected_angle = math::HALF_PI;
        
        if (std::abs(angle - expected_angle) < math::EPSILON) {
            test_report << "  ✓ 90-degree angle calculation\n";
            passed_tests++;
        } else {
            test_report << "  ✗ 90-degree angle calculation failed\n";
        }
        
        // Test 3: Rodrigues rotation
        test_report << "\nTest 3: Rodrigues Rotation\n";
        total_tests++;
        
        Vector3 z_axis(0.0, 0.0, 1.0);
        Vector3 rotated = math::RotationOps::rodrigues_rotation(x_axis, z_axis, math::HALF_PI);
        
        if (std::abs(rotated.x()) < math::EPSILON && 
            std::abs(rotated.y() - 1.0) < math::EPSILON && 
            std::abs(rotated.z()) < math::EPSILON) {
            test_report << "  ✓ Rodrigues rotation (90° around Z)\n";
            passed_tests++;
        } else {
            test_report << "  ✗ Rodrigues rotation failed\n";
        }
        
        // Test 4: Plane normal calculation
        test_report << "\nTest 4: Plane Normal Calculation\n";
        total_tests++;
        
        Vector3 p1(0.0, 0.0, 0.0);
        Vector3 p2(1.0, 0.0, 0.0);
        Vector3 p3(0.0, 1.0, 0.0);
        Vector3 normal = math::GeometryOps::calculate_plane_normal(p1, p2, p3);
        
        if (std::abs(normal.x()) < math::EPSILON && 
            std::abs(normal.y()) < math::EPSILON && 
            std::abs(normal.z() - 1.0) < math::EPSILON) {
            test_report << "  ✓ XY plane normal calculation\n";
            passed_tests++;
        } else {
            test_report << "  ✗ XY plane normal calculation failed\n";
        }
        
        // Test 5: Base triangle properties
        test_report << "\nTest 5: Base Triangle Properties\n";
        total_tests++;
        
        std::vector<Vector3> bases = math::BasePositions::get_all_base_positions();
        double area = math::BasePositions::calculate_base_triangle_area();
        
        if (bases.size() == 3 && area > 0) {
            test_report << "  ✓ Base triangle has positive area: " << area << "\n";
            passed_tests++;
        } else {
            test_report << "  ✗ Base triangle area calculation failed\n";
        }
        
        // Test 6: Configuration validation
        test_report << "\nTest 6: Configuration Validation\n";
        total_tests++;
        
        Configuration valid_config;
        valid_config.tolerance = 0.01;
        valid_config.max_iterations = 100;
        
        if (valid_config.is_valid()) {
            test_report << "  ✓ Valid configuration accepted\n";
            passed_tests++;
        } else {
            test_report << "  ✗ Valid configuration rejected\n";
        }
        
        // Test 7: Timer functionality
        test_report << "\nTest 7: Timer Functionality\n";
        total_tests++;
        
        Timer timer;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        double elapsed = timer.elapsed_ms();
        
        if (elapsed >= 10.0 && elapsed < 100.0) {  // Should be around 10ms, allow some margin
            test_report << "  ✓ Timer measurement: " << elapsed << "ms\n";
            passed_tests++;
        } else {
            test_report << "  ✗ Timer measurement failed: " << elapsed << "ms\n";
        }
        
        // Test summary
        test_report << "\n" << std::string(50, '=') << "\n";
        test_report << "Test Summary: " << passed_tests << "/" << total_tests << " tests passed\n";
        
        if (passed_tests == total_tests) {
            test_report << "✓ All tests PASSED!\n";
        } else {
            test_report << "✗ Some tests FAILED!\n";
        }
        
        return error::make_success(test_report.str());
        
    } catch (const std::exception& e) {
        return error::make_error<std::string>("Self-tests failed: " + std::string(e.what()));
    }
}

// ============================================================================
// UTILITY FUNCTIONS IMPLEMENTATION
// ============================================================================

std::string get_environment_info() {
    std::ostringstream info;
    
    info << "Delta Robot Core Library Environment Information\n";
    info << "===============================================\n\n";
    
    // Version information
    info << "Core Library Version: " << get_version().to_string() << "\n";
    
    // Eigen information
    info << "Eigen Version: " << EIGEN_WORLD_VERSION << "." 
         << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << "\n";
    
    // C++ standard
    info << "C++ Standard: ";
    #ifdef DELTA_HAS_CPP20
        info << "C++20\n";
    #elif defined(DELTA_HAS_CPP17)
        info << "C++17\n";
    #else
        info << "C++14 or earlier\n";
    #endif
    
    // Compiler information
    info << "Compiler: ";
    #ifdef __GNUC__
        info << "GCC " << __GNUC__ << "." << __GNUC_MINOR__ << "\n";
    #elif defined(_MSC_VER)
        info << "MSVC " << _MSC_VER << "\n";
    #elif defined(__clang__)
        info << "Clang " << __clang_major__ << "." << __clang_minor__ << "\n";
    #else
        info << "Unknown\n";
    #endif
    
    // Platform information
    info << "Platform: ";
    #ifdef DELTA_PLATFORM_WINDOWS
        info << "Windows\n";
    #elif defined(DELTA_PLATFORM_LINUX)
        info << "Linux\n";
    #elif defined(DELTA_PLATFORM_MACOS)
        info << "macOS\n";
    #else
        info << "Unknown\n";
    #endif
    
    // Features
    info << "\nAvailable Features:\n";
    info << "  - Eigen Integration: Yes\n";
    info << "  - Error Handling: Yes\n";
    info << "  - Logging: Yes\n";
    info << "  - Performance Timing: Yes\n";
    
    // Configuration
    info << "\nCurrent Configuration:\n";
    info << "  - Initialized: " << (is_initialized() ? "Yes" : "No") << "\n";
    info << "  - Logging Enabled: " << (error::Logger::is_enabled() ? "Yes" : "No") << "\n";
    
    return info.str();
}

std::string get_performance_benchmarks(int iterations) {
    std::ostringstream benchmarks;
    
    benchmarks << "Delta Robot Core Library Performance Benchmarks\n";
    benchmarks << "==============================================\n\n";
    benchmarks << "Iterations per test: " << iterations << "\n\n";
    
    Timer timer;
    
    // Benchmark 1: Vector operations
    benchmarks << "Vector Operations:\n";
    
    timer.reset();
    for (int i = 0; i < iterations; ++i) {
        Vector3 v1(i * 0.001, i * 0.002, i * 0.003);
        Vector3 v2((i + 1) * 0.001, (i + 1) * 0.002, (i + 1) * 0.003);
        volatile double result = v1.dot(v2);  // volatile to prevent optimization
        (void)result;
    }
    double vector_dot_time = timer.elapsed_ms();
    benchmarks << "  Dot product: " << vector_dot_time << "ms "
               << "(" << (iterations / vector_dot_time * 1000.0) << " ops/sec)\n";
    
    timer.reset();
    for (int i = 0; i < iterations; ++i) {
        Vector3 v1(i * 0.001, i * 0.002, i * 0.003);
        Vector3 v2((i + 1) * 0.001, (i + 1) * 0.002, (i + 1) * 0.003);
        volatile Vector3 result = v1.cross(v2);
        (void)result;
    }
    double vector_cross_time = timer.elapsed_ms();
    benchmarks << "  Cross product: " << vector_cross_time << "ms "
               << "(" << (iterations / vector_cross_time * 1000.0) << " ops/sec)\n";
    
    // Benchmark 2: Mathematical functions
    benchmarks << "\nMathematical Functions:\n";
    
    timer.reset();
    for (int i = 0; i < iterations; ++i) {
        Vector3 v1(i * 0.001, i * 0.002, i * 0.003);
        Vector3 v2((i + 1) * 0.001, (i + 1) * 0.002, (i + 1) * 0.003);
        volatile double result = math::VectorOps::angle_between(v1, v2);
        (void)result;
    }
    double angle_time = timer.elapsed_ms();
    benchmarks << "  Angle between vectors: " << angle_time << "ms "
               << "(" << (iterations / angle_time * 1000.0) << " ops/sec)\n";
    
    timer.reset();
    for (int i = 0; i < iterations; ++i) {
        Vector3 v(i * 0.001, i * 0.002, i * 0.003);
        volatile Vector3 result = math::VectorOps::safe_normalize(v);
        (void)result;
    }
    double normalize_time = timer.elapsed_ms();
    benchmarks << "  Vector normalization: " << normalize_time << "ms "
               << "(" << (iterations / normalize_time * 1000.0) << " ops/sec)\n";
    
    // Benchmark 3: Rodrigues rotation
    benchmarks << "\nRotation Operations:\n";
    
    Vector3 axis(0.0, 0.0, 1.0);
    timer.reset();
    for (int i = 0; i < iterations; ++i) {
        Vector3 v(i * 0.001, i * 0.002, 0.0);
        double angle = i * 0.001;
        volatile Vector3 result = math::RotationOps::rodrigues_rotation(v, axis, angle);
        (void)result;
    }
    double rodrigues_time = timer.elapsed_ms();
    benchmarks << "  Rodrigues rotation: " << rodrigues_time << "ms "
               << "(" << (iterations / rodrigues_time * 1000.0) << " ops/sec)\n";
    
    benchmarks << "\nNote: Higher ops/sec indicates better performance.\n";
    benchmarks << "These benchmarks may vary depending on hardware and compiler optimizations.\n";
    
    return benchmarks.str();
}

// ============================================================================
// CONFIGURATION PRESETS IMPLEMENTATION
// ============================================================================

namespace presets {

Configuration get_fast_config() {
    Configuration config;
    config.tolerance = fabrik::TOLERANCE * 10.0;  // 0.1
    config.max_iterations = fabrik::MAX_ITERATIONS / 5;  // 20
    config.enable_constraints = true;
    config.verbose_logging = false;
    config.track_performance = false;
    return config;
}

Configuration get_precise_config() {
    Configuration config;
    config.tolerance = fabrik::TOLERANCE / 10.0;  // 0.001
    config.max_iterations = fabrik::MAX_ITERATIONS * 2;  // 200
    config.enable_constraints = true;
    config.verbose_logging = false;
    config.track_performance = true;
    return config;
}

Configuration get_debug_config() {
    Configuration config;
    config.tolerance = fabrik::TOLERANCE;  // 0.01
    config.max_iterations = fabrik::MAX_ITERATIONS;  // 100
    config.enable_constraints = true;
    config.verbose_logging = true;
    config.track_performance = true;
    return config;
}

Configuration get_default_config() {
    Configuration config;
    config.tolerance = fabrik::TOLERANCE;
    config.max_iterations = fabrik::MAX_ITERATIONS;
    config.enable_constraints = true;
    config.verbose_logging = false;
    config.track_performance = false;
    return config;
}

} // namespace presets

} // namespace delta::core