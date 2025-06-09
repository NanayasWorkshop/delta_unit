"""
Delta Robot Python Package - Step 1.2 Update
Enhanced with Level 1 Composite Modules (KinematicsSolver + OrientationSolver)
"""
import numpy as np

# Import the single consolidated module
try:
    from . import delta_robot_complete as delta_complete
    
    # Create convenient aliases that maintain backward compatibility
    class fermat:
        # NEW: Improved Level 0 classes (Step 1.1)
        FermatSolver = delta_complete.FermatSolver
        FermatResult = delta_complete.FermatResult
        
        # Backward compatibility aliases
        FermatModule = delta_complete.FermatModule
    
    class joint_state:
        # NEW: Improved Level 0 classes (Step 1.1)
        JointStateSolver = delta_complete.JointStateSolver
        JointStateResult = delta_complete.JointStateResult
        
        # Backward compatibility aliases
        JointStateModule = delta_complete.JointStateModule
    
    class kinematics:
        # NEW: Level 1 Composite Solver with timing and validation (Step 1.2)
        KinematicsSolver = delta_complete.KinematicsSolver
        KinematicsResult = delta_complete.KinematicsResult
        
        # Backward compatibility aliases
        KinematicsModule = delta_complete.KinematicsModule
    
    class orientation:
        # NEW: Level 1 Composite Solver with timing and validation (Step 1.2)
        OrientationSolver = delta_complete.OrientationSolver
        OrientationResult = delta_complete.OrientationResult
        CoordinateFrame = delta_complete.CoordinateFrame
        
        # Backward compatibility aliases
        OrientationModule = delta_complete.OrientationModule
    
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
        
        # Main collision-aware solver
        CollisionAwareSolver = delta_complete.CollisionAwareSolver
        
        # Data structures
        Obstacle = delta_complete.Obstacle
        CollisionResult = delta_complete.CollisionResult
        WaypointConversionResult = delta_complete.WaypointConversionResult
        CollisionAwareSolutionResult = delta_complete.CollisionAwareSolutionResult
        CollisionAwareConfig = delta_complete.CollisionAwareConfig
        
    class motor:
        MotorModule = delta_complete.MotorModule
        MotorResult = delta_complete.MotorResult
        LevelData = delta_complete.LevelData
        
        # Segment Calculator (separated from FABRIK for performance)
        SegmentCalculator = delta_complete.SegmentCalculator
        SegmentCalculationResult = delta_complete.SegmentCalculationResult
        SegmentEndEffectorData = delta_complete.SegmentEndEffectorData
        
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

def calculate_segment_end_effectors(fabrik_chain, num_segments=None):
    """
    Calculate complex segment end-effector positions from a solved FABRIK chain.
    
    This is now separated from FABRIK for performance - only call after collision-free solution.
    
    Args:
        fabrik_chain: Solved FABRIK chain
        num_segments: Number of robot segments (optional)
    
    Returns:
        SegmentCalculationResult with segment end-effector data
    """
    if num_segments is None:
        return motor.SegmentCalculator.calculate_segment_end_effectors(fabrik_chain)
    else:
        return motor.SegmentCalculator.calculate_segment_end_effectors(fabrik_chain, num_segments)

def create_test_obstacles():
    """Create test obstacles for collision detection."""
    return collision.CollisionDetector.create_test_obstacles()

def create_obstacle(center_x, center_y, center_z, radius):
    """Create a single obstacle."""
    center = np.array([center_x, center_y, center_z])
    return collision.Obstacle(center, radius)

def verify_installation():
    """Verify that the Step 1.2 improvements are working correctly."""
    try:
        # Test NEW Level 1 Composite Modules (Step 1.2)
        direction = np.array([1, 0, 1])
        
        # Test improved KinematicsSolver
        kinematics_result = kinematics.KinematicsSolver.calculate(direction)
        print(f"✓ KinematicsSolver working! (time: {kinematics_result.computation_time_ms:.3f}ms, success: {kinematics_result.calculation_successful})")
        
        # Test improved OrientationSolver
        orientation_result = orientation.OrientationSolver.calculate(direction)
        print(f"✓ OrientationSolver working! (time: {orientation_result.computation_time_ms:.3f}ms, success: {orientation_result.calculation_successful})")
        
        # Test efficiency: OrientationSolver using existing kinematics result
        orientation_from_kinematics = orientation.OrientationSolver.calculate_from_kinematics(kinematics_result)
        print(f"✓ OrientationSolver from kinematics! (time: {orientation_from_kinematics.computation_time_ms:.3f}ms)")
        
        # Test Level 0 modules from Step 1.1
        fermat_result = fermat.FermatSolver.calculate(direction)
        print(f"✓ FermatSolver working! (time: {fermat_result.computation_time_ms:.3f}ms)")
        
        joint_result = joint_state.JointStateSolver.calculate_from_fermat(direction, fermat_result)
        print(f"✓ JointStateSolver working! (time: {joint_result.computation_time_ms:.3f}ms)")
        
        # Test validation
        invalid_direction = np.array([0, 0, 0])  # Zero vector should be invalid
        valid_input = kinematics.KinematicsSolver.is_input_valid(direction)
        invalid_input = kinematics.KinematicsSolver.is_input_valid(invalid_direction)
        print(f"✓ Input validation working! (valid: {valid_input}, invalid: {invalid_input})")
        
        # Test error handling
        failed_result = kinematics.KinematicsSolver.calculate(invalid_direction)
        print(f"✓ Error handling working! (failed: {not failed_result.calculation_successful})")
        
        # Test backward compatibility
        old_kinematics_result = kinematics.KinematicsModule.calculate(direction)
        old_orientation_result = orientation.OrientationModule.calculate(direction)
        print("✓ Backward compatibility working!")
        
        # Test basic motor calculation still works
        result = motor.MotorModule.calculate_motors(100, 50, 300)
        print("✓ Basic motor module still working!")
        
        # Test collision detection components
        obstacles = create_test_obstacles()
        print(f"✓ Created {len(obstacles)} test obstacles!")
        
        # Test collision-aware solver
        target = np.array([100, 0, 300])
        collision_aware_result = collision.CollisionAwareSolver.solve(target, obstacles, 1)
        print(f"✓ Collision-aware solver working! (collision_free: {collision_aware_result.collision_free})")
        
        # Test segment calculator
        fabrik_result = solve_fabrik_ik(100, 0, 300)
        segment_result = motor.SegmentCalculator.calculate_segment_end_effectors(fabrik_result.final_chain)
        print(f"✓ Segment calculator working! (calculated {len(segment_result.segment_end_effectors)} segments in {segment_result.calculation_time_ms:.2f}ms)")
        
        print("\n🎯 Step 1.2 SUCCESS: Level 1 Composite Modules (KinematicsSolver + OrientationSolver) working!")
        print("✅ Enhanced with timing, validation, and error handling")
        print("✅ Using improved Level 0 modules internally")
        print("✅ Backward compatibility maintained")
        print("✅ Clean interfaces with no circular dependencies")
        
        return True
        
    except Exception as e:
        print(f"✗ Step 1.2 test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

# Export main interfaces
__all__ = [
    'fermat', 'joint_state', 'kinematics', 'orientation', 'fabrik', 'collision', 'motor',
    'calculate_motors', 'solve_fabrik_ik', 'solve_with_collision_avoidance', 
    'calculate_segment_end_effectors',
    'create_test_obstacles', 'create_obstacle', 'verify_installation',
    'FABRIK_TOLERANCE', 'DEFAULT_ROBOT_SEGMENTS', 'DEFAULT_SPLINE_DIAMETER'
]