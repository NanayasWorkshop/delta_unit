from setuptools import setup, find_packages
from pybind11.setup_helpers import Pybind11Extension, build_ext
import pybind11

# Find Eigen installation
import os
eigen_include_dirs = []

# Common Eigen installation paths
eigen_paths = [
    '/usr/include/eigen3',           # Ubuntu/Debian
    '/usr/local/include/eigen3',     # Manual installation
    '/opt/homebrew/include/eigen3',  # macOS Homebrew (Apple Silicon)
    '/usr/local/Cellar/eigen/*/include/eigen3',  # macOS Homebrew (Intel)
]

for path in eigen_paths:
    if os.path.exists(path):
        eigen_include_dirs.append(path)
        print(f"Found Eigen at: {path}")
        break

if not eigen_include_dirs:
    print("Warning: Eigen not found in standard paths")
    print("Please install Eigen with: sudo apt-get install libeigen3-dev")

# Common compilation flags for Eigen
eigen_flags = [
    '-O3',  # Optimization
    '-DEIGEN_MPL2_ONLY',  # Use only MPL2 licensed parts of Eigen
    '-DEIGEN_DONT_PARALLELIZE',  # Disable Eigen's own parallelization (optional)
    '-Wno-maybe-uninitialized',  # Suppress Eigen's false positive warnings
    '-Wno-unused-but-set-variable',  # Suppress other Eigen warnings
]

ext_modules = [
    # CONSOLIDATED MODULE with Complete Collision Detection Pipeline + Optimized Segment Calculator
    Pybind11Extension(
        "delta_robot.delta_robot_complete",
        [
            # Single consolidated binding file
            "cpp/src/delta_robot_complete_bindings.cpp",
            
            # All core implementation files
            "cpp/core/math_utils.cpp",
            "cpp/core/constraint_utils.cpp",
            
            # Kinematics implementations
            "cpp/kinematics/fermat_module.cpp",
            "cpp/kinematics/joint_state.cpp", 
            "cpp/kinematics/kinematics_module.cpp",
            "cpp/kinematics/orientation_module.cpp",
            
            # FABRIK implementations (optimized - no segment extraction)
            "cpp/fabrik/fabrik_initialization.cpp",
            "cpp/fabrik/fabrik_backward.cpp",
            "cpp/fabrik/fabrik_forward.cpp",
            "cpp/fabrik/fabrik_solver.cpp",
            
            # Motor implementation
            "cpp/motor/motor_module.cpp",
            "cpp/motor/segment_calculator.cpp",  # NEW: Separated segment calculator for performance
            
            # Complete collision detection pipeline implementation
            "cpp/collision/u_points_extractor.cpp",
            "cpp/collision/collision_detector.cpp",
            "cpp/collision/waypoint_converter.cpp",
            "cpp/collision/collision_aware_solver.cpp",  # Main orchestrator
        ],
        include_dirs=[
            "cpp/include", 
            "cpp/core", 
            "cpp/fabrik", 
            "cpp/kinematics", 
            "cpp/motor",
            "cpp/collision",
            pybind11.get_include()
        ] + eigen_include_dirs,
        language='c++',
        cxx_std=17,
        extra_compile_args=eigen_flags,
    ),
]

setup(
    name="delta_unit",
    version="1.3.0",  # Updated version for optimized segment calculator
    description="Delta Robot Kinematics and Control with Complete Collision Detection Pipeline + Optimized Segment Calculator",
    author="Delta Robot Team",
    packages=find_packages(),
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.7",
    install_requires=[
        "numpy>=1.18.0",
    ],
    extras_require={
        "visualization": [
            "plotly>=5.0.0",
        ],
    },
)