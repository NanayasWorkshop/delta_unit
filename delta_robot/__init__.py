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
    WORKSPACE_CONE_ANGLE_RAD = delta_types.WORKSPACE_CONE_ANGLE_RAD
    
    # FABRIK Configuration Constants - NEWLY ADDED
    DEFAULT_ROBOT_SEGMENTS = delta_types.DEFAULT_ROBOT_SEGMENTS
    SPHERICAL_JOINT_CONE_ANGLE_RAD = delta_types.SPHERICAL_JOINT_CONE_ANGLE_RAD
    SPHERICAL_JOINT_CONE_ANGLE_DEG = delta_types.SPHERICAL_JOINT_CONE_ANGLE_DEG
    
    # FABRIK Solver Constants - NEWLY ADDED
    FABRIK_TOLERANCE = delta_types.FABRIK_TOLERANCE
    FABRIK_MAX_ITERATIONS = delta_types.FABRIK_MAX_ITERATIONS
    EPSILON_MATH = delta_types.EPSILON_MATH
    
    # Geometry Constants - NEWLY ADDED
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
except ImportError as e:
    print(f"Warning: FABRIK modules not available: {e}")
    # Don't fail package import if FABRIK modules aren't built yet
    fabrik_init = None
    fabrik_back = None
    fabrik_fwd = None
    fabrik = None

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
    """
    Complete FABRIK inverse kinematics solution.
    
    Args:
        target_x, target_y, target_z: Target position coordinates
        num_segments: Number of robot segments (default: uses DEFAULT_ROBOT_SEGMENTS constant)
        tolerance: Convergence tolerance (default: uses FABRIK_TOLERANCE constant)
    
    Returns:
        FabrikSolutionResult if FABRIK modules available, None otherwise
    """
    if fabrik is None:
        print("Error: FABRIK modules not available. Build them first.")
        return None
    
    # ✅ NOW USES CONSTANTS AS DEFAULTS
    if num_segments is None:
        num_segments = DEFAULT_ROBOT_SEGMENTS
    if tolerance is None:
        tolerance = FABRIK_TOLERANCE
    
    target = Vector3(target_x, target_y, target_z)
    return fabrik.solve_delta_robot(num_segments, target, tolerance)

def verify_installation():
    """Verify that all modules imported correctly."""
    modules_status = {
        'delta_types': True,
        'fermat_module': 'fermat' in globals(),
        'joint_state_module': 'joint_state' in globals(),
        'kinematics_module': 'kinematics' in globals(),
        'orientation_module': 'orientation' in globals(),
        'fabrik_initialization': fabrik_init is not None,
        'fabrik_backward': fabrik_back is not None,
        'fabrik_forward': fabrik_fwd is not None,
        'fabrik_solver': fabrik is not None,
    }
    
    all_good = all(modules_status.values())
    
    if all_good:
        print("✓ All delta robot modules imported successfully!")
        print(f"✓ Using {DEFAULT_ROBOT_SEGMENTS} robot segments (from C++ constants)")
        print(f"✓ FABRIK tolerance: {FABRIK_TOLERANCE} (from C++ constants)")
    else:
        print("Status of delta robot modules:")
        for module, status in modules_status.values():
            print(f"  {module}: {'✓' if status else '✗'}")
    
    return all_good

def show_constants():
    """Show all available constants from C++ headers."""
    print("Delta Robot Constants (from C++ headers):")
    print("=" * 50)
    print("ROBOT PHYSICAL CONSTANTS:")
    print(f"  ROBOT_RADIUS = {ROBOT_RADIUS}")
    print(f"  MIN_HEIGHT = {MIN_HEIGHT}")
    print(f"  WORKING_HEIGHT = {WORKING_HEIGHT}")
    print(f"  MOTOR_LIMIT = {MOTOR_LIMIT}")
    print(f"  WORKSPACE_CONE_ANGLE_RAD = {WORKSPACE_CONE_ANGLE_RAD}")
    print()
    print("FABRIK CONFIGURATION:")
    print(f"  DEFAULT_ROBOT_SEGMENTS = {DEFAULT_ROBOT_SEGMENTS}")
    print(f"  SPHERICAL_JOINT_CONE_ANGLE_RAD = {SPHERICAL_JOINT_CONE_ANGLE_RAD}")
    print(f"  SPHERICAL_JOINT_CONE_ANGLE_DEG = {SPHERICAL_JOINT_CONE_ANGLE_DEG}")
    print()
    print("FABRIK SOLVER:")
    print(f"  FABRIK_TOLERANCE = {FABRIK_TOLERANCE}")
    print(f"  FABRIK_MAX_ITERATIONS = {FABRIK_MAX_ITERATIONS}")
    print(f"  EPSILON_MATH = {EPSILON_MATH}")
    print()
    print("GEOMETRY:")
    print(f"  BASE_A_ANGLE = {BASE_A_ANGLE}")
    print(f"  BASE_B_ANGLE = {BASE_B_ANGLE}")
    print(f"  BASE_C_ANGLE = {BASE_C_ANGLE}")

def show_fabrik_capabilities():
    """Show available FABRIK capabilities."""
    if fabrik is None:
        print("FABRIK modules not available. Build them with:")
        print("  python setup.py build_ext --inplace")
        return
    
    print("Available FABRIK capabilities:")
    print("✓ FABRIK Initialization - Create robot chain structures")
    print("✓ FABRIK Backward - Move end-effector toward target")
    print("✓ FABRIK Forward - Fix base and recalculate lengths")
    print("✓ FABRIK Solver - Complete inverse kinematics solution")
    print()
    print(f"Configuration (from C++ constants):")
    print(f"  Default segments: {DEFAULT_ROBOT_SEGMENTS}")
    print(f"  Tolerance: {FABRIK_TOLERANCE}")
    print(f"  Max iterations: {FABRIK_MAX_ITERATIONS}")
    print()
    print("Quick usage:")
    print("  import delta_robot")
    print("  result = delta_robot.solve_fabrik_ik(100, 100, 300)")
    print("  print(f'Converged: {result.converged}, Error: {result.final_error}')")
    print(f"  # Will use {DEFAULT_ROBOT_SEGMENTS} segments automatically!")