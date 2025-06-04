"""
Delta Robot Python Package
"""

# Import foundation types first
try:
    from . import delta_types
    
    # Make types available at package level
    Vector3 = delta_types.Vector3
    Matrix4x4 = delta_types.Matrix4x4
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

# Import other modules
try:
    from . import fermat_module as fermat
    from . import joint_state_module as joint_state
    from . import kinematics_module as kinematics
    from . import orientation_module as orientation
except ImportError as e:
    raise ImportError(f"Failed to import modules: {e}")

# Import FABRIK modules
try:
    from . import fabrik_initialization as fabrik_init
    from . import fabrik_backward as fabrik_back
    from . import fabrik_forward as fabrik_fwd
    from . import fabrik_solver as fabrik
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
    fermat_result = fermat.FermatModule.calculate(x, y, z)
    direction_vector = Vector3(x, y, z)
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
    
    target = Vector3(target_x, target_y, target_z)
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
        'fermat_module': True,
        'joint_state_module': True,
        'kinematics_module': True,
        'orientation_module': True,
        'fabrik_initialization': fabrik_init is not None,
        'fabrik_backward': fabrik_back is not None,
        'fabrik_forward': fabrik_fwd is not None,
        'fabrik_solver': fabrik is not None,
        'motor_module': motor is not None,
    }
    
    all_good = all(modules_status.values())
    
    if all_good:
        print("✓ All delta robot modules imported successfully!")
        return True
    else:
        print("Module status:")
        for module, status in modules_status.items():
            print(f"  {module}: {'✓' if status else '✗'}")
        return False

# Calculate SPHERICAL_JOINT_CONE_ANGLE_DEG for backward compatibility
SPHERICAL_JOINT_CONE_ANGLE_DEG = rad_to_deg(SPHERICAL_JOINT_CONE_ANGLE_RAD)