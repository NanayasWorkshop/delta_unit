from setuptools import setup
from pybind11.setup_helpers import Pybind11Extension, build_ext
import pybind11

ext_modules = [
    # Fermat module - Vector3 registered here
    Pybind11Extension(
        "fermat_module",
        ["cpp/src/fermat_bindings.cpp", "cpp/src/fermat_module.cpp", "cpp/src/math_utils.cpp"],
        include_dirs=["cpp/include", pybind11.get_include()],
        language='c++',
        cxx_std=17,
    ),
    # Joint state module - NO Vector3 registration
    Pybind11Extension(
        "joint_state_module",
        ["cpp/src/joint_state_bindings.cpp", "cpp/src/joint_state.cpp", "cpp/src/math_utils.cpp"],
        include_dirs=["cpp/include", pybind11.get_include()],
        language='c++',
        cxx_std=17,
    ),
    # Kinematics module - Vector3 only if not already available
    Pybind11Extension(
        "kinematics_module",
        ["cpp/src/kinematics_bindings.cpp", "cpp/src/kinematics_module.cpp", 
         "cpp/src/fermat_module.cpp", "cpp/src/joint_state.cpp", "cpp/src/math_utils.cpp"],
        include_dirs=["cpp/include", pybind11.get_include()],
        language='c++',
        cxx_std=17,
    ),
    # Orientation module - NO Vector3 registration, assumes from kinematics
    Pybind11Extension(
        "orientation_module",
        ["cpp/src/orientation_bindings.cpp", "cpp/src/orientation_module.cpp",
         "cpp/src/kinematics_module.cpp", "cpp/src/fermat_module.cpp", 
         "cpp/src/joint_state.cpp", "cpp/src/math_utils.cpp"],
        include_dirs=["cpp/include", pybind11.get_include()],
        language='c++',
        cxx_std=17,
    ),
]

setup(
    name="delta_unit",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.7",
)