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
    # Delta Types module - Foundation module with shared types (EIGEN-BASED)
    Pybind11Extension(
        "delta_robot.delta_types",
        ["cpp/src/delta_types_bindings.cpp", "cpp/core/math_utils.cpp"],
        include_dirs=["cpp/include", "cpp/core", "cpp/fabrik", "cpp/kinematics", pybind11.get_include()] + eigen_include_dirs,
        language='c++',
        cxx_std=17,
        extra_compile_args=eigen_flags,
    ),
    
    # CONSOLIDATED KINEMATICS MODULE - Replaces 4 separate modules (MAJOR SIMPLIFICATION!)
    Pybind11Extension(
        "delta_robot.kinematics_complete",
        [
            # Consolidated binding file
            "cpp/kinematics/kinematics_bindings.cpp",
            # All kinematics implementation files
            "cpp/kinematics/fermat_module.cpp",
            "cpp/kinematics/joint_state.cpp",
            "cpp/kinematics/kinematics_module.cpp",
            "cpp/kinematics/orientation_module.cpp",
            # Base utilities
            "cpp/core/math_utils.cpp"
        ],
        include_dirs=["cpp/include", "cpp/core", "cpp/fabrik", "cpp/kinematics", pybind11.get_include()] + eigen_include_dirs,
        language='c++',
        cxx_std=17,
        extra_compile_args=eigen_flags,
    ),
    
    # CONSOLIDATED FABRIK MODULE - Replaces 4 separate modules (from Phase 3)
    Pybind11Extension(
        "delta_robot.fabrik_complete",
        [
            # Consolidated binding file
            "cpp/fabrik/fabrik_bindings.cpp",
            # All FABRIK implementation files
            "cpp/fabrik/fabrik_initialization.cpp",
            "cpp/fabrik/fabrik_backward.cpp",
            "cpp/fabrik/fabrik_forward.cpp",
            "cpp/fabrik/fabrik_solver.cpp",
            # Kinematics dependencies (required by fabrik_forward.cpp)
            "cpp/kinematics/kinematics_module.cpp",
            "cpp/kinematics/fermat_module.cpp", 
            "cpp/kinematics/joint_state.cpp",
            # Base utilities
            "cpp/core/math_utils.cpp",
            "cpp/core/constraint_utils.cpp"
        ],
        include_dirs=["cpp/include", "cpp/core", "cpp/fabrik", "cpp/kinematics", pybind11.get_include()] + eigen_include_dirs,
        language='c++',
        cxx_std=17,
        extra_compile_args=eigen_flags,
    ),
    
    # Motor Module - Orchestrates FABRIK + Kinematics + Orientation (uses Eigen) - PHASE 6: MOVED TO cpp/motor/
    Pybind11Extension(
        "delta_robot.motor_module",
        [
            # Primary sources - NOW IN cpp/motor/
            "cpp/motor/motor_module_bindings.cpp", 
            "cpp/motor/motor_module.cpp",
            # FABRIK solver dependencies (complete chain)
            "cpp/fabrik/fabrik_solver.cpp",
            "cpp/fabrik/fabrik_backward.cpp",
            "cpp/fabrik/fabrik_forward.cpp",
            "cpp/fabrik/fabrik_initialization.cpp",
            # Kinematics and Orientation dependencies
            "cpp/kinematics/kinematics_module.cpp",
            "cpp/kinematics/orientation_module.cpp",
            "cpp/kinematics/fermat_module.cpp", 
            "cpp/kinematics/joint_state.cpp",
            # Base math utilities
            "cpp/core/math_utils.cpp",
            "cpp/core/constraint_utils.cpp"
        ],
        include_dirs=["cpp/include", "cpp/core", "cpp/fabrik", "cpp/kinematics", "cpp/motor", pybind11.get_include()] + eigen_include_dirs,
        language='c++',
        cxx_std=17,
        extra_compile_args=eigen_flags,
    ),
    ]

setup(
    name="delta_unit",
    packages=find_packages(),
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.7",
    install_requires=[
        "numpy",  # Required for Eigen integration
    ],
    package_data={
        'delta_robot': ['*.so'],
    },
    include_package_data=True,
)