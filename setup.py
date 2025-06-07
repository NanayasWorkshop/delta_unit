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
    # CONSOLIDATED MODULE - Replaces all 4 separate modules (70% code reduction)
    # NOW WITH COLLISION AVOIDANCE SUPPORT
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
            
            # FABRIK implementations
            "cpp/fabrik/fabrik_initialization.cpp",
            "cpp/fabrik/fabrik_backward.cpp",
            "cpp/fabrik/fabrik_forward.cpp",
            "cpp/fabrik/fabrik_solver.cpp",
            
            # Motor implementation
            "cpp/motor/motor_module.cpp",
            
            # NEW: Collision avoidance implementation
            "cpp/collision/spline_collision_avoidance.cpp",
        ],
        include_dirs=[
            "cpp/include", 
            "cpp/core", 
            "cpp/fabrik", 
            "cpp/kinematics", 
            "cpp/motor",
            "cpp/collision",  # NEW: collision avoidance include directory
            pybind11.get_include()
        ] + eigen_include_dirs,
        language='c++',
        cxx_std=17,
        extra_compile_args=eigen_flags,
    ),
]

setup(
    name="delta_unit",
    version="1.1.0",  # Updated version for collision avoidance support
    description="Delta Robot Kinematics and Control with Collision Avoidance",
    long_description="""
    Complete Delta Robot Python Package with FABRIK inverse kinematics, 
    motor control, and real-time spline collision avoidance.
    
    Features:
    - High-performance FABRIK inverse kinematics solver (0.06ms)
    - Real-time spline collision avoidance (<0.3ms)
    - Complete motor control pipeline
    - Interactive 3D visualization support
    - Analytical spline bending with constraint preservation
    """,
    author="Delta Robot Team",
    packages=find_packages(),
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.7",
    install_requires=[
        "numpy>=1.18.0",  # Required for Eigen integration
    ],
    extras_require={
        "visualization": [
            "plotly>=5.0.0",
            "scipy>=1.6.0",
        ],
        "dev": [
            "pytest>=6.0.0",
            "pytest-cov>=2.10.0",
        ],
    },
    package_data={
        'delta_robot': ['*.so', '*.pyd'],  # Include compiled extensions
    },
    include_package_data=True,
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: C++",
        "Topic :: Scientific/Engineering :: Mathematics",
        "Topic :: Scientific/Engineering :: Physics",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
    keywords=[
        "robotics", 
        "delta-robot", 
        "kinematics", 
        "fabrik", 
        "collision-avoidance",
        "motion-planning",
        "real-time",
        "inverse-kinematics"
    ],
    project_urls={
        "Documentation": "https://github.com/your-repo/delta-robot",
        "Source": "https://github.com/your-repo/delta-robot",
        "Tracker": "https://github.com/your-repo/delta-robot/issues",
    },
)