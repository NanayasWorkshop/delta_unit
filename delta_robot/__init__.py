"""
Delta Robot Python Package (Consolidated Module)
"""
import numpy as np

# Import the single consolidated module
try:
    from . import delta_robot_complete as delta_complete
    
    # Create convenient aliases that maintain backward compatibility
    class fermat:
        FermatModule = delta_complete.FermatModule
        FermatResult = delta_complete.FermatResult
    
    class joint_state:
        JointStateModule = delta_complete.JointStateModule
        JointStateResult = delta_complete.JointStateResult
    
    class kinematics:
        KinematicsModule = delta_complete.KinematicsModule
        KinematicsResult = delta_complete.KinematicsResult
    
    class orientation:
        OrientationModule = delta_complete.OrientationModule
        OrientationResult = delta_complete.OrientationResult
    
    class fabrik:
        FabrikSolver = delta_complete.FabrikSolver
        FabrikInitialization = delta_complete.FabrikInitialization
        FabrikChain = delta_complete.FabrikChain
        FabrikJoint = delta_complete.FabrikJoint
        solve_delta_robot = delta_complete.solve_delta_robot
        
    class collision:
        UPointsExtractor = delta_complete.UPointsExtractor
        CollisionDetector = delta_complete.CollisionDetector
        WaypointConverter = delta_complete.WaypointConverter
        Obstacle = delta_complete.Obstacle
        CollisionResult = delta_complete.CollisionResult
        WaypointConversionResult = delta_complete.WaypointConversionResult
        
    class motor:
        MotorModule = delta_complete.MotorModule
        MotorResult = delta_complete.MotorResult
        LevelData = delta_complete.LevelData
        
    # Constants from the consolidated module
    FABRIK_TOLERANCE = delta_complete.FABRIK_TOLERANCE
    DEFAULT_ROBOT_SEGMENTS = delta_complete.DEFAULT_ROBOT_SEGMENTS
    DEFAULT_SPLINE_DIAMETER = delta_complete.DEFAULT_SPLINE_DIAMETER
    
except ImportError as e:
    raise ImportError(f"Failed to import delta_robot_complete module: {e}")

def calculate_motors(target_x, target_y, target_z):
    """Calculate motor positions using the Motor module."""
    return motor.MotorModule.calculate_motors(target_x, target_y, target_z)

def solve_fabrik_ik(target_x, target_y, target_z, num_segments=None, tolerance=None):
    """Complete FABRIK inverse kinematics solution."""
    if num_segments is None:
        num_segments = DEFAULT_ROBOT_SEGMENTS
    if tolerance is None:
        tolerance = FABRIK_TOLERANCE
    
    target = np.array([target_x, target_y, target_z])
    return fabrik.solve_delta_robot(num_segments, target, tolerance)

def verify_installation():
    """Verify that the consolidated module imported correctly."""
    try:
        result = motor.MotorModule.calculate_motors(100, 50, 300)
        print("✓ Delta robot module with collision detection imported successfully!")
        return True
    except Exception as e:
        print(f"✗ Module test failed: {e}")
        return False

# Export main interfaces
__all__ = [
    'fermat', 'joint_state', 'kinematics', 'orientation', 'fabrik', 'collision', 'motor',
    'calculate_motors', 'solve_fabrik_ik', 'verify_installation',
    'FABRIK_TOLERANCE', 'DEFAULT_ROBOT_SEGMENTS', 'DEFAULT_SPLINE_DIAMETER'
]