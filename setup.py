from setuptools import setup, find_packages
from pybind11.setup_helpers import Pybind11Extension, build_ext
import pybind11
import os

# Find Eigen installation
eigen_include_dirs = []
eigen_paths = [
    '/usr/include/eigen3',           
    '/usr/local/include/eigen3',     
    '/opt/homebrew/include/eigen3',  
    '/usr/local/Cellar/eigen/*/include/eigen3',  
]

for path in eigen_paths:
    if os.path.exists(path):
        eigen_include_dirs.append(path)
        print(f"Found Eigen at: {path}")
        break

if not eigen_include_dirs:
    print("Warning: Eigen not found in standard paths")
    print("Please install Eigen with: sudo apt-get install libeigen3-dev")

# Common compilation settings
common_include_dirs = ["cpp/include", "cpp/core/include", pybind11.get_include()] + eigen_include_dirs
common_flags = [
    '-O3',
    '-DEIGEN_MPL2_ONLY',
    '-DEIGEN_DONT_PARALLELIZE',
    '-Wno-maybe-uninitialized',
    '-Wno-unused-but-set-variable',
]

# Define source file groups to avoid duplication
SOURCE_GROUPS = {
    # Core layer (foundation)
    'core_layer': [
        "cpp/core/src/math_core.cpp",
        "cpp/core/src/error_handling.cpp",
        "cpp/core/src/delta_core_impl.cpp"
    ],
    
    # Legacy math base (for backward compatibility during transition)
    'math_base_legacy': [
        "cpp/src/math_utils.cpp"
    ],
    
    # Individual modules (will be refactored in later phases)
    'fermat_core': [
        "cpp/src/fermat_module.cpp"
    ],
    'joint_state_core': [
        "cpp/src/joint_state.cpp"
    ],
    'kinematics_core': [
        "cpp/src/kinematics_module.cpp",
        "cpp/src/fermat_module.cpp",
        "cpp/src/joint_state.cpp",
        "cpp/src/math_utils.cpp"
    ],
    'orientation_core': [
        "cpp/src/orientation_module.cpp",
        "cpp/src/kinematics_module.cpp",
        "cpp/src/fermat_module.cpp",
        "cpp/src/joint_state.cpp",
        "cpp/src/math_utils.cpp"
    ],
    'fabrik_initialization': [
        "cpp/src/fabrik_initialization.cpp",
        "cpp/src/math_utils.cpp"
    ],
    'fabrik_backward_core': [
        "cpp/src/fabrik_backward.cpp",
        "cpp/src/fabrik_initialization.cpp",
        "cpp/src/math_utils.cpp"
    ],
    'fabrik_forward_core': [
        "cpp/src/fabrik_forward.cpp",
        "cpp/src/fabrik_initialization.cpp",
        "cpp/src/kinematics_module.cpp",
        "cpp/src/fermat_module.cpp",
        "cpp/src/joint_state.cpp",
        "cpp/src/math_utils.cpp"
    ],
    'fabrik_solver_complete': [
        "cpp/src/fabrik_solver.cpp",
        "cpp/src/fabrik_backward.cpp",
        "cpp/src/fabrik_forward.cpp",
        "cpp/src/fabrik_initialization.cpp",
        "cpp/src/kinematics_module.cpp",
        "cpp/src/fermat_module.cpp",
        "cpp/src/joint_state.cpp",
        "cpp/src/math_utils.cpp"
    ]
}

def create_extension(name, binding_file, source_group_key=None, extra_sources=None, include_core=True):
    """Create a Pybind11Extension with reduced duplication."""
    sources = [binding_file]
    
    # Add core layer if requested (recommended for all new modules)
    if include_core:
        sources.extend(SOURCE_GROUPS['core_layer'])
    
    # Add specific source group
    if source_group_key and source_group_key in SOURCE_GROUPS:
        sources.extend(SOURCE_GROUPS[source_group_key])
    
    # Add extra sources
    if extra_sources:
        sources.extend(extra_sources)
    
    return Pybind11Extension(
        name,
        sources,
        include_dirs=common_include_dirs,
        language='c++',
        cxx_std=17,
        extra_compile_args=common_flags,
    )

ext_modules = [
    # ========================================================================
    # PHASE 1: CORE LAYER (NEW)
    # ========================================================================
    
    # NEW: Core layer - Foundation module with all core functionality
    create_extension(
        "delta_robot.delta_core",
        "cpp/core/bindings/core_bindings.cpp",
        include_core=False,  # This IS the core, don't include itself
        extra_sources=SOURCE_GROUPS['core_layer']
    ),
    
    # ========================================================================
    # EXISTING MODULES (TRANSITIONAL - will be refactored in later phases)
    # ========================================================================
    
    # Legacy types module (will be deprecated in favor of delta_core)
    create_extension(
        "delta_robot.delta_types",
        "cpp/src/delta_types_bindings.cpp",
        "math_base_legacy",
        include_core=False  # Legacy module, keep separate during transition
    ),
    
    # Core calculation modules (will be moved to geometry layer in Phase 2)
    create_extension(
        "delta_robot.fermat_module",
        "cpp/src/fermat_bindings.cpp",
        "math_base_legacy",
        extra_sources=SOURCE_GROUPS['fermat_core']
    ),
    
    create_extension(
        "delta_robot.joint_state_module",
        "cpp/src/joint_state_bindings.cpp",
        "math_base_legacy",
        extra_sources=SOURCE_GROUPS['joint_state_core']
    ),
    
    # Kinematics modules (will be moved to kinematics layer in Phase 3)
    create_extension(
        "delta_robot.kinematics_module",
        "cpp/src/kinematics_bindings.cpp",
        "kinematics_core"
    ),
    
    create_extension(
        "delta_robot.orientation_module",
        "cpp/src/orientation_bindings.cpp",
        "orientation_core"
    ),
    
    # FABRIK modules (will be moved to algorithm layer in Phase 4)
    create_extension(
        "delta_robot.fabrik_initialization",
        "cpp/src/fabrik_initialization_bindings.cpp",
        "fabrik_initialization"
    ),
    
    create_extension(
        "delta_robot.fabrik_backward",
        "cpp/src/fabrik_backward_bindings.cpp",
        "fabrik_backward_core"
    ),
    
    create_extension(
        "delta_robot.fabrik_forward",
        "cpp/src/fabrik_forward_bindings.cpp",
        "fabrik_forward_core"
    ),
    
    create_extension(
        "delta_robot.fabrik_solver",
        "cpp/src/fabrik_solver_bindings.cpp",
        "fabrik_solver_complete"
    ),
    
    # Motor module (will be moved to control layer in Phase 5)
    create_extension(
        "delta_robot.motor_module",
        "cpp/src/motor_module_bindings.cpp",
        "fabrik_solver_complete",
        extra_sources=["cpp/src/motor_module.cpp", "cpp/src/orientation_module.cpp"]
    ),
]

# ============================================================================
# CONDITIONAL COMPILATION
# ============================================================================

# Allow building only specific modules during development
import sys
if '--core-only' in sys.argv:
    sys.argv.remove('--core-only')
    ext_modules = [ext for ext in ext_modules if 'delta_core' in ext.name]
    print("Building core layer only")

elif '--legacy-only' in sys.argv:
    sys.argv.remove('--legacy-only')
    ext_modules = [ext for ext in ext_modules if 'delta_core' not in ext.name]
    print("Building legacy modules only")

setup(
    name="delta_unit",
    packages=find_packages(),
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.7",
    install_requires=["numpy"],
    package_data={'delta_robot': ['*.so']},
    include_package_data=True,
    
    # Development dependencies
    extras_require={
        'dev': [
            'pytest',
            'pytest-cov',
            'black',
            'flake8',
        ],
        'docs': [
            'sphinx',
            'sphinx-rtd-theme',
        ]
    },
    
    # Metadata
    version="1.0.0",
    description="Delta Robot Kinematics and Control Library",
    long_description="A comprehensive library for delta robot kinematics, inverse kinematics (FABRIK), and motor control with layered architecture.",
    author="Delta Robot Team",
    license="MIT",
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
        "Topic :: Scientific/Engineering",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
)