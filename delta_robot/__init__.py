"""
Delta Robot Python Package (Consolidated Module with Collision Avoidance)
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
        FabrikSolverConfig = delta_complete.FabrikSolverConfig
        solve_delta_robot = delta_complete.solve_delta_robot
        solve_with_spline = delta_complete.solve_with_spline
        
    # NEW: Collision avoidance classes
    class collision:
        SplineCollisionAvoidance = delta_complete.SplineCollisionAvoidance
        Obstacle = delta_complete.Obstacle
        CollisionAvoidanceResult = delta_complete.CollisionAvoidanceResult
        
    class motor:
        MotorModule = delta_complete.MotorModule
        MotorResult = delta_complete.MotorResult
        LevelData = delta_complete.LevelData
        
    # Constants from the consolidated module
    FABRIK_TOLERANCE = delta_complete.FABRIK_TOLERANCE
    DEFAULT_ROBOT_SEGMENTS = delta_complete.DEFAULT_ROBOT_SEGMENTS
    SPLINE_THICKNESS = delta_complete.SPLINE_THICKNESS
    COLLISION_SAFETY_MARGIN = delta_complete.COLLISION_SAFETY_MARGIN
    COLLISION_AVOIDANCE_TARGET_TIME_MS = delta_complete.COLLISION_AVOIDANCE_TARGET_TIME_MS
    
except ImportError as e:
    raise ImportError(f"Failed to import delta_robot_complete module: {e}")

def calculate_complete_pipeline(x, y, z):
    """Complete delta robot calculation pipeline."""
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
    if num_segments is None:
        num_segments = DEFAULT_ROBOT_SEGMENTS
    if tolerance is None:
        tolerance = FABRIK_TOLERANCE
    
    target = np.array([target_x, target_y, target_z])
    return fabrik.solve_delta_robot(num_segments, target, tolerance)

def solve_fabrik_with_spline(target_x, target_y, target_z, num_segments=None, tolerance=None):
    """FABRIK inverse kinematics solution with spline visualization points."""
    if num_segments is None:
        num_segments = DEFAULT_ROBOT_SEGMENTS
    if tolerance is None:
        tolerance = FABRIK_TOLERANCE
    
    target = np.array([target_x, target_y, target_z])
    return fabrik.solve_with_spline(num_segments, target, tolerance)

def calculate_motors(target_x, target_y, target_z):
    """Calculate motor positions using the Motor module."""
    return motor.MotorModule.calculate_motors(target_x, target_y, target_z)

# =============================================================================
# NEW: COLLISION AVOIDANCE FUNCTIONS
# =============================================================================

def solve_fabrik_with_collision_avoidance(target_x, target_y, target_z, obstacles, num_segments=None, tolerance=None):
    """FABRIK inverse kinematics solution with collision avoidance."""
    if num_segments is None:
        num_segments = DEFAULT_ROBOT_SEGMENTS
    if tolerance is None:
        tolerance = FABRIK_TOLERANCE
    
    target = np.array([target_x, target_y, target_z])
    return delta_complete.solve_delta_robot_safe(num_segments, target, obstacles, tolerance)

def create_sphere_obstacle(x, y, z, radius):
    """Create a spherical obstacle at given position with specified radius."""
    return delta_complete.create_sphere_obstacle(x, y, z, radius)

def create_obstacles_from_positions(positions, radii):
    """Create obstacles from lists of positions and radii."""
    return delta_complete.create_obstacles_from_positions(positions, radii)

def create_obstacles_from_coordinates(obstacle_data):
    """Create obstacles from list of (x, y, z, radius) tuples."""
    return delta_complete.create_obstacles_from_coordinates(obstacle_data)

def check_spline_collision(spline_points, obstacles, thickness=None):
    """Check if spline collides with obstacles."""
    if thickness is None:
        thickness = SPLINE_THICKNESS
    return delta_complete.check_spline_collision(spline_points, obstacles, thickness)

def avoid_spline_collisions(spline_points, obstacles, thickness=None, safety_margin=None):
    """Apply collision avoidance to a spline path."""
    if thickness is None:
        thickness = SPLINE_THICKNESS
    if safety_margin is None:
        safety_margin = COLLISION_SAFETY_MARGIN
    return collision.SplineCollisionAvoidance.avoid_collisions(spline_points, obstacles)

# =============================================================================
# CONVENIENCE FUNCTIONS FOR COLLISION-AWARE SOLVING
# =============================================================================

def solve_delta_robot_safe(num_segments, target, obstacles, tolerance=None):
    """Solve delta robot with collision avoidance."""
    if tolerance is None:
        tolerance = FABRIK_TOLERANCE
    return delta_complete.solve_delta_robot_safe(num_segments, target, obstacles, tolerance)

def verify_installation():
    """Verify that the consolidated module imported correctly."""
    try:
        # Test basic functionality
        result = motor.MotorModule.calculate_motors(100, 50, 300)
        print("✓ Consolidated delta robot module imported successfully!")
        print("✓ All functionality available through single module")
        
        # Test spline functionality
        spline_result = solve_fabrik_with_spline(100, 50, 300)
        if hasattr(spline_result, 'spline_points') and hasattr(spline_result, 'segment_midpoints'):
            print("✓ Spline visualization support available!")
        
        # Test collision avoidance functionality
        try:
            test_obstacle = create_sphere_obstacle(50, 0, 200, 30)
            print("✓ Collision avoidance support available!")
            
            # Test collision detection
            test_spline = [[0, 0, 0], [50, 0, 200], [100, 0, 400]]
            has_collision = check_spline_collision(test_spline, [test_obstacle], SPLINE_THICKNESS)
            print(f"✓ Collision detection working (test collision: {has_collision})")
            
        except Exception as e:
            print(f"⚠️  Collision avoidance test failed: {e}")
        
        return True
    except Exception as e:
        print(f"✗ Module test failed: {e}")
        return False

# Convenience functions for creating vectors
def create_vector3(x=0, y=0, z=0):
    """Create Vector3 (numpy array) from coordinates."""
    return np.array([x, y, z])

# Export main interfaces (updated with collision support)
__all__ = [
    'fermat', 'joint_state', 'kinematics', 'orientation', 'fabrik', 'motor', 'collision',
    'calculate_complete_pipeline', 'solve_fabrik_ik', 'solve_fabrik_with_spline', 
    'solve_fabrik_with_collision_avoidance', 'calculate_motors',
    'create_sphere_obstacle', 'create_obstacles_from_positions', 'create_obstacles_from_coordinates',
    'check_spline_collision', 'avoid_spline_collisions', 'solve_delta_robot_safe',
    'verify_installation', 'create_vector3',
    'FABRIK_TOLERANCE', 'DEFAULT_ROBOT_SEGMENTS', 'SPLINE_THICKNESS', 
    'COLLISION_SAFETY_MARGIN', 'COLLISION_AVOIDANCE_TARGET_TIME_MS'
]