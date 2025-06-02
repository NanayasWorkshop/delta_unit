from setuptools import setup, find_packages
from pybind11.setup_helpers import Pybind11Extension, build_ext
import pybind11

ext_modules = [
    # Delta Types module - Foundation module with shared types
    Pybind11Extension(
        "delta_robot.delta_types",
        ["cpp/src/delta_types_bindings.cpp", "cpp/src/math_utils.cpp"],
        include_dirs=["cpp/include", pybind11.get_include()],
        language='c++',
        cxx_std=17,
    ),
    # Fermat module - NO Vector3 registration
    Pybind11Extension(
        "delta_robot.fermat_module",
        ["cpp/src/fermat_bindings.cpp", "cpp/src/fermat_module.cpp", "cpp/src/math_utils.cpp"],
        include_dirs=["cpp/include", pybind11.get_include()],
        language='c++',
        cxx_std=17,
    ),
    # Joint state module - NO Vector3 registration
    Pybind11Extension(
        "delta_robot.joint_state_module",
        ["cpp/src/joint_state_bindings.cpp", "cpp/src/joint_state.cpp", "cpp/src/math_utils.cpp"],
        include_dirs=["cpp/include", pybind11.get_include()],
        language='c++',
        cxx_std=17,
    ),
    # Kinematics module - NO Vector3 registration
    Pybind11Extension(
        "delta_robot.kinematics_module",
        ["cpp/src/kinematics_bindings.cpp", "cpp/src/kinematics_module.cpp", 
         "cpp/src/fermat_module.cpp", "cpp/src/joint_state.cpp", "cpp/src/math_utils.cpp"],
        include_dirs=["cpp/include", pybind11.get_include()],
        language='c++',
        cxx_std=17,
    ),
    # Orientation module - NO Vector3/Matrix4x4 registration
    Pybind11Extension(
        "delta_robot.orientation_module",
        ["cpp/src/orientation_bindings.cpp", "cpp/src/orientation_module.cpp",
         "cpp/src/kinematics_module.cpp", "cpp/src/fermat_module.cpp", 
         "cpp/src/joint_state.cpp", "cpp/src/math_utils.cpp"],
        include_dirs=["cpp/include", pybind11.get_include()],
        language='c++',
        cxx_std=17,
    ),
    # FABRIK Initialization module
    Pybind11Extension(
        "delta_robot.fabrik_initialization",
        ["cpp/src/fabrik_initialization_bindings.cpp", "cpp/src/fabrik_initialization.cpp", "cpp/src/math_utils.cpp"],
        include_dirs=["cpp/include", pybind11.get_include()],
        language='c++',
        cxx_std=17,
    ),
    # FABRIK Backward module
    Pybind11Extension(
        "delta_robot.fabrik_backward",
        ["cpp/src/fabrik_backward_bindings.cpp", "cpp/src/fabrik_backward.cpp", "cpp/src/fabrik_initialization.cpp", "cpp/src/math_utils.cpp"],
        include_dirs=["cpp/include", pybind11.get_include()],
        language='c++',
        cxx_std=17,
    ),
    # FABRIK Forward module - NEW
    Pybind11Extension(
        "delta_robot.fabrik_forward",
        ["cpp/src/fabrik_forward_bindings.cpp", "cpp/src/fabrik_forward.cpp", 
         "cpp/src/fabrik_initialization.cpp", "cpp/src/fermat_module.cpp", 
         "cpp/src/joint_state.cpp", "cpp/src/math_utils.cpp"],
        include_dirs=["cpp/include", pybind11.get_include()],
        language='c++',
        cxx_std=17,
    ),
]

setup(
    name="delta_unit",
    packages=find_packages(),
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.7",
    package_data={
        'delta_robot': ['*.so'],
    },
    include_package_data=True,
)