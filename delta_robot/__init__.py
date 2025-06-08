"""
Delta Robot Python Package (Consolidated Module with Complete Collision Pipeline)
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
        # Individual components
        UPointsExtractor = delta_complete.UPointsExtractor
        CollisionDetector = delta_complete.CollisionDetector
        WaypointConverter = delta_complete.WaypointConverter
        
        # Main collision-aware solver (NEW)
        CollisionAwareSolver = delta_complete.CollisionAwareSolver
        
        # Data structures
        Obstacle = delta_complete.Obstacle
        CollisionResult = delta_complete.CollisionResult
        WaypointConversionResult = delta_complete.WaypointConversionResult
        CollisionAwareSolutionResult = delta_complete.CollisionAwareSolutionResult  # NEW
        CollisionAwareConfig = delta_complete.CollisionAwareConfig  # NEW
        
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

def solve_with_collision_avoidance(target_x, target_y, target_z, obstacles, max_iterations=3):
    """
    Complete collision-aware inverse kinematics solution.
    
    Args:
        target_x, target_y, target_z: Target position coordinates
        obstacles: List of Obstacle objects
        max_iterations: Maximum collision avoidance iterations
    
    Returns:
        CollisionAwareSolutionResult with collision-free solution
    """
    target = np.array([target_x, target_y, target_z])
    return collision.CollisionAwareSolver.solve(target, obstacles, max_iterations)

def create_test_obstacles():
    """Create test obstacles for collision detection."""
    return collision.CollisionDetector.create_test_obstacles()

def create_obstacle(center_x, center_y, center_z, radius):
    """Create a single obstacle."""
    center = np.array([center_x, center_y, center_z])
    return collision.Obstacle(center, radius)

def verify_installation():
    """Verify that the consolidated module with collision detection imported correctly."""
    try:
        # Test basic motor calculation
        result = motor.MotorModule.calculate_motors(100, 50, 300)
        print("✓ Basic motor module working!")
        
        # Test collision detection components
        obstacles = create_test_obstacles()
        print(f"✓ Created {len(obstacles)} test obstacles!")
        
        # Test U points extraction
        joint_positions = [
            np.array([0, 0, 0]),
            np.array([0, 0, 73]),
            np.array([10, 5, 146]),
            np.array([20, 10, 219]),
            np.array([30, 15, 292])
        ]
        u_points = collision.UPointsExtractor.extract_u_points_from_positions(joint_positions)
        print(f"✓ Extracted {len(u_points)} U points!")
        
        # Test collision detection with proper type conversion
        # The C++ function expects the raw return from UPointsExtractor, not converted numpy arrays
        collision_result = collision.CollisionDetector.check_and_avoid(u_points, obstacles, delta_complete.DEFAULT_SPLINE_DIAMETER)
        print(f"✓ Collision detection working! (has_collision: {collision_result.has_collision})")
        
        # Test collision-aware solver
        target = np.array([100, 0, 300])
        collision_aware_result = collision.CollisionAwareSolver.solve(target, obstacles, 1)
        print(f"✓ Collision-aware solver working! (collision_free: {collision_aware_result.collision_free})")
        
        print("✓ Delta robot module with COMPLETE collision detection pipeline imported successfully!")
        return True
        
    except Exception as e:
        print(f"✗ Module test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

# Export main interfaces
__all__ = [
    'fermat', 'joint_state', 'kinematics', 'orientation', 'fabrik', 'collision', 'motor',
    'calculate_motors', 'solve_fabrik_ik', 'solve_with_collision_avoidance', 
    'create_test_obstacles', 'create_obstacle', 'verify_installation',
    'FABRIK_TOLERANCE', 'DEFAULT_ROBOT_SEGMENTS', 'DEFAULT_SPLINE_DIAMETER'
]