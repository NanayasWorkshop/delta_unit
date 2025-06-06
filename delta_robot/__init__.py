"""
Delta Robot Python Package (Eigen-based)
"""
import numpy as np

# Import foundation types first
try:
    from . import delta_types
    
    # Make types available at package level
    # Note: Vector3 and Matrix4 are now Eigen types, automatically converted by pybind11
    Vector3 = delta_types.Vector3  # Convenience constructor function
    Matrix4 = delta_types.Matrix4  # Convenience constructor function
    CoordinateFrame = delta_types.CoordinateFrame
    
    # Make ALL constants available - Robot Physical Constants
    ROBOT_RADIUS = delta_types.ROBOT_RADIUS
    MIN_HEIGHT = delta_types.MIN_HEIGHT
    WORKING_HEIGHT = delta_types.WORKING_HEIGHT
    MOTOR_LIMIT = delta_types.MOTOR_LIMIT
    
    # FABRIK Configuration Constants
    DEFAULT_ROBOT_SEGMENTS = delta_types.DEFAULT_ROBOT_SEGMENTS
    SPHERICAL_JOINT_CONE_ANGLE_RAD = delta_types.SPHERICAL_JOINT_CONE_ANGLE_RAD
    
    # FABRIK Solver Constants
    FABRIK_TOLERANCE = delta_types.FABRIK_TOLERANCE
    FABRIK_MAX_ITERATIONS = delta_types.FABRIK_MAX_ITERATIONS
    EPSILON_MATH = delta_types.EPSILON_MATH
    
    # Geometry Constants
    BASE_A_ANGLE = delta_types.BASE_A_ANGLE
    BASE_B_ANGLE = delta_types.BASE_B_ANGLE
    BASE_C_ANGLE = delta_types.BASE_C_ANGLE
    
    # Utility functions
    rad_to_deg = delta_types.rad_to_deg
    deg_to_rad = delta_types.deg_to_rad
    get_base_position_A = delta_types.get_base_position_A
    get_base_position_B = delta_types.get_base_position_B
    get_base_position_C = delta_types.get_base_position_C
    
except ImportError as e:
    raise ImportError(f"Failed to import delta_types module: {e}")

# Import consolidated kinematics module
try:
    from . import kinematics_complete
    
    # Create convenient aliases that maintain backward compatibility
    class fermat:
        FermatModule = kinematics_complete.FermatModule
        FermatResult = kinematics_complete.FermatResult
    
    class joint_state:
        JointStateModule = kinematics_complete.JointStateModule
        JointStateResult = kinematics_complete.JointStateResult
    
    class kinematics:
        KinematicsModule = kinematics_complete.KinematicsModule
        KinematicsResult = kinematics_complete.KinematicsResult
    
    class orientation:
        OrientationModule = kinematics_complete.OrientationModule
        OrientationResult = kinematics_complete.OrientationResult
        
except ImportError as e:
    raise ImportError(f"Failed to import kinematics modules: {e}")

# Import FABRIK modules
try:
    from . import fabrik_complete as fabrik
    
    # Create convenient aliases for FABRIK (backward compatibility)
    fabrik_init = fabrik
    fabrik_back = fabrik
    fabrik_fwd = fabrik
    # Main fabrik is already assigned above
    
except ImportError:
    fabrik_init = None
    fabrik_back = None
    fabrik_fwd = None
    fabrik = None

# Import Motor module
try:
    from . import motor_module as motor
except ImportError:
    motor = None

# Make all modules available
types = delta_types

def calculate_complete_pipeline(x, y, z):
    """Complete delta robot calculation pipeline."""
    # Create Eigen Vector3 using numpy array (automatic conversion)
    direction_vector = np.array([x, y, z])
    
    fermat_result = fermat.FermatModule.calculate(x, y, z)
    joint_result = joint_state.JointStateModule.calculate_from_fermat(direction_vector, fermat_result)
    kinematics_result = kinematics.KinematicsModule.calculate(x, y, z)
    orientation_result = orientation.OrientationModule.calculate(x, y, z)
    
    return {
        'input_vector': (x, y, z),
        'fermat': fermat_result,
        'joint_state': joint_result,
        'kinematics': kinematics_result,
        'orientation': orientation_result
    }

def solve_fabrik_ik(target_x, target_y, target_z, num_segments=None, tolerance=None):
    """Complete FABRIK inverse kinematics solution."""
    if fabrik is None:
        raise ImportError("FABRIK modules not available. Build them first.")
    
    if num_segments is None:
        num_segments = DEFAULT_ROBOT_SEGMENTS
    if tolerance is None:
        tolerance = FABRIK_TOLERANCE
    
    # Create Eigen Vector3 using numpy array (automatic conversion)
    target = np.array([target_x, target_y, target_z])
    return fabrik.solve_delta_robot(num_segments, target, tolerance)

def calculate_motors(target_x, target_y, target_z):
    """Calculate motor positions using the Motor module."""
    if motor is None:
        raise ImportError("Motor module not available. Build it first.")
    
    return motor.MotorModule.calculate_motors(target_x, target_y, target_z)

def verify_installation():
    """Verify that all modules imported correctly."""
    modules_status = {
        'delta_types': True,
        'kinematics_complete': True,  # New consolidated module
        'fabrik_complete': fabrik is not None,
        'motor_module': motor is not None,
    }
    
    all_good = all(modules_status.values())
    
    if all_good:
        print("✓ All delta robot modules imported successfully!")
        print("✓ Using Eigen for optimized linear algebra operations")
        print("✓ Consolidated modules: kinematics_complete, fabrik_complete")
        return True
    else:
        print("Module status:")
        for module, status in modules_status.items():
            print(f"  {module}: {'✓' if status else '✗'}")
        return False

# Calculate SPHERICAL_JOINT_CONE_ANGLE_DEG for backward compatibility
SPHERICAL_JOINT_CONE_ANGLE_DEG = rad_to_deg(SPHERICAL_JOINT_CONE_ANGLE_RAD)

# Convenience functions for creating Eigen vectors/matrices from Python
def create_vector3(x=0, y=0, z=0):
    """Create Vector3 (Eigen::Vector3d) from coordinates."""
    return np.array([x, y, z])

def create_matrix4_identity():
    """Create 4x4 identity matrix (Eigen::Matrix4d)."""
    return np.eye(4)

# Export the convenience functions
__all__ = [
    'Vector3', 'Matrix4', 'CoordinateFrame',
    'fermat', 'joint_state', 'kinematics', 'orientation',
    'fabrik_init', 'fabrik_back', 'fabrik_fwd', 'fabrik', 'motor',
    'calculate_complete_pipeline', 'solve_fabrik_ik', 'calculate_motors',
    'verify_installation', 'create_vector3', 'create_matrix4_identity',
    'rad_to_deg', 'deg_to_rad',
    'get_base_position_A', 'get_base_position_B', 'get_base_position_C',
    # Constants
    'ROBOT_RADIUS', 'MIN_HEIGHT', 'WORKING_HEIGHT', 'MOTOR_LIMIT',
    'DEFAULT_ROBOT_SEGMENTS', 'SPHERICAL_JOINT_CONE_ANGLE_RAD', 'SPHERICAL_JOINT_CONE_ANGLE_DEG',
    'FABRIK_TOLERANCE', 'FABRIK_MAX_ITERATIONS', 'EPSILON_MATH',
    'BASE_A_ANGLE', 'BASE_B_ANGLE', 'BASE_C_ANGLE'
]