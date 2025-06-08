#!/usr/bin/env python3
"""
Collision-Aware Delta Robot Motor Module Test with Performance Optimization
"""
import sys
import os
import numpy as np

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def parse_coordinates(coord_str):
    """Parse coordinate string like '100,50,300' into x,y,z values"""
    try:
        coords = [float(x.strip()) for x in coord_str.split(',')]
        if len(coords) != 3:
            raise ValueError("Expected 3 coordinates")
        return coords
    except ValueError as e:
        print(f"Error parsing coordinates '{coord_str}': {e}")
        print("Expected format: x,y,z (e.g., 100,50,300)")
        sys.exit(1)

def parse_current_positions(current_str, expected_segments=8):
    """Parse current joint positions string"""
    try:
        if ':' in current_str:
            # Colon-separated format
            position_strs = current_str.split(':')
        else:
            # Comma-separated format with parentheses (support legacy format)
            current_str = current_str.replace('(', '').replace(')', '')
            position_strs = current_str.split(',')
            # Group every 3 values
            position_strs = [','.join(position_strs[i:i+3]) for i in range(0, len(position_strs), 3)]
        
        positions = []
        for pos_str in position_strs:
            coords = [float(x.strip()) for x in pos_str.split(',')]
            if len(coords) != 3:
                raise ValueError(f"Expected 3 coordinates per position, got {len(coords)}")
            positions.append(np.array([coords[0], coords[1], coords[2]]))
        
        expected_joints = expected_segments + 2  # base + segments + end-effector
        if len(positions) != expected_joints:
            print(f"Warning: Expected {expected_joints} joint positions, got {len(positions)}. Using straight-up fallback.")
            return None
            
        return positions
    except Exception as e:
        print(f"Error parsing current positions '{current_str}': {e}")
        print("Using straight-up fallback.")
        return None

def format_joint_positions(joint_positions):
    """Format joint positions as --current parameter"""
    try:
        formatted_positions = []
        for joint_pos in joint_positions:
            if hasattr(joint_pos, 'shape'):  # numpy array
                formatted_positions.append(f"{joint_pos[0]:.3f},{joint_pos[1]:.3f},{joint_pos[2]:.3f}")
            else:
                # Handle Eigen vector case (fallback)
                formatted_positions.append(f"{joint_pos.x():.3f},{joint_pos.y():.3f},{joint_pos.z():.3f}")
        
        return ":".join(formatted_positions)
    
    except Exception as e:
        print(f"Warning: Could not extract joint positions: {e}")
        return "0,0,0:0,0,0:0,0,0:0,0,0"  # fallback

def create_test_scenario_obstacles():
    """Create some test obstacles in the workspace"""
    try:
        import delta_robot
        
        obstacles = []
        
        # Obstacle 1: In middle-left of workspace
        obstacles.append(delta_robot.create_obstacle(80, 80, 250, 30))
        
        # Obstacle 2: On the right side
        obstacles.append(delta_robot.create_obstacle(120, -60, 280, 25))
        
        print(f"Created {len(obstacles)} test obstacles:")
        for i, obs in enumerate(obstacles):
            center = obs.center
            print(f"  Obstacle {i+1}: center=({center[0]:.1f}, {center[1]:.1f}, {center[2]:.1f}), radius={obs.radius}")
        
        return obstacles
        
    except Exception as e:
        print(f"Warning: Could not create test obstacles: {e}")
        return []

def solve_with_collision_detection(target_x, target_y, target_z, current_positions=None, enable_obstacles=True):
    """Solve using collision-aware approach"""
    import delta_robot
    
    target = np.array([target_x, target_y, target_z])
    
    # Create test obstacles if enabled
    obstacles = create_test_scenario_obstacles() if enable_obstacles else []
    
    print(f"\n=== COLLISION-AWARE SOLVING ===")
    print(f"Target: ({target_x}, {target_y}, {target_z})")
    print(f"Obstacles: {len(obstacles)} in workspace")
    print(f"Using {'provided' if current_positions else 'straight-up'} initialization")
    
    # Configure collision-aware solving
    config = delta_robot.collision.CollisionAwareConfig()
    config.max_collision_iterations = 3
    config.spline_diameter = delta_robot.DEFAULT_SPLINE_DIAMETER
    config.enable_collision_detection = len(obstacles) > 0
    config.verbose_logging = True  # Show collision detection details
    
    try:
        # Solve with collision avoidance
        if current_positions is not None:
            # Use provided initial positions
            collision_result = delta_robot.collision.CollisionAwareSolver.solve_with_collision_avoidance(
                target, obstacles, current_positions, delta_robot.DEFAULT_ROBOT_SEGMENTS, config)
        else:
            # Use default initialization
            collision_result = delta_robot.collision.CollisionAwareSolver.solve_with_collision_avoidance(
                target, obstacles, config)
        
        print(f"\n=== COLLISION SOLVING RESULTS ===")
        print(f"✓ Collision-free solution: {collision_result.collision_free}")
        print(f"✓ FABRIK converged: {collision_result.fabrik_result.converged}")
        print(f"✓ Final error: {collision_result.fabrik_result.final_error:.4f}")
        print(f"✓ Collision iterations: {collision_result.collision_iterations}")
        print(f"✓ Total collision time: {collision_result.total_collision_time_ms:.2f}ms")
        print(f"✓ FABRIK solve time: {collision_result.fabrik_result.solve_time_ms:.2f}ms")
        
        if not collision_result.collision_free:
            print("⚠️  Warning: Solution may have collisions!")
        
        # Now calculate segment end-effectors using the optimized SegmentCalculator
        print(f"\n=== SEGMENT CALCULATION (Separated for Performance) ===")
        segment_result = delta_robot.motor.SegmentCalculator.calculate_segment_end_effectors(
            collision_result.fabrik_result.final_chain)
        
        print(f"✓ Calculated {len(segment_result.segment_end_effectors)} segments in {segment_result.calculation_time_ms:.2f}ms")
        
        # Now use the standard motor pipeline for kinematics/orientation calculations
        print(f"\n=== MOTOR CALCULATIONS ===")
        
        # Extract joint positions for motor module
        fabrik_joint_positions = []
        for joint in collision_result.fabrik_result.final_chain.joints:
            # Convert from Eigen to numpy if needed
            if hasattr(joint.position, 'shape'):
                fabrik_joint_positions.append(joint.position)
            else:
                # Fallback for Eigen vectors
                fabrik_joint_positions.append(np.array([joint.position.x(), joint.position.y(), joint.position.z()]))
        
        # Call motor module with the collision-free joint positions
        motor_result = delta_robot.motor.MotorModule.calculate_motors(target, fabrik_joint_positions)
        
        return motor_result, collision_result, segment_result
        
    except Exception as e:
        print(f"Error in collision-aware solving: {e}")
        import traceback
        traceback.print_exc()
        return None, None, None

def solve_without_collision_detection(target_x, target_y, target_z, current_positions=None):
    """Solve using traditional approach (for comparison)"""
    import delta_robot
    
    print(f"\n=== TRADITIONAL SOLVING (No Collision Detection) ===")
    print(f"Target: ({target_x}, {target_y}, {target_z})")
    print(f"Using {'provided' if current_positions else 'straight-up'} initialization")
    
    try:
        if current_positions is not None:
            result = delta_robot.motor.MotorModule.calculate_motors(target_x, target_y, target_z, current_positions)
        else:
            result = delta_robot.calculate_motors(target_x, target_y, target_z)
        
        print(f"✓ FABRIK converged: {result.fabrik_converged}")
        print(f"✓ Final error: {result.fabrik_error:.4f}")
        print(f"✓ Solve time: {result.solve_time_ms:.2f}ms")
        
        return result
        
    except Exception as e:
        print(f"Error in traditional solving: {e}")
        import traceback
        traceback.print_exc()
        return None

def print_motor_results(motor_result, title="MOTOR RESULTS"):
    """Print motor calculation results"""
    if motor_result is None:
        print("No motor results to display.")
        return
    
    print(f"\n=== {title} ===")
    
    if not motor_result.levels:
        print("No transformation levels processed.")
        return
    
    # Print compact segment information
    for level_idx, level_data in enumerate(motor_result.levels):
        if level_idx < len(motor_result.original_segment_numbers):
            base_original_seg_num = motor_result.original_segment_numbers[level_idx]
        else:
            base_original_seg_num = level_idx + 1
        
        print(f"--- Segment {base_original_seg_num} ---")
        
        # Handle position access
        pos = level_data.base_segment_position
        if hasattr(pos, 'shape'):  # numpy array
            print(f"  Position: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
        else:  # Fallback for Eigen
            print(f"  Position: Eigen vector (conversion issue)")
        
        print(f"  Motors: z_A={level_data.z_A:.3f}, z_B={level_data.z_B:.3f}, z_C={level_data.z_C:.3f}")
        print(f"  Joints: prismatic={level_data.prismatic_joint:.3f}, roll={level_data.roll_joint:.1f}°, pitch={level_data.pitch_joint:.1f}°")
    
    # Add FABRIK joint positions in --current format
    if hasattr(motor_result, 'fabrik_joint_positions') and motor_result.fabrik_joint_positions:
        joint_positions_str = format_joint_positions(motor_result.fabrik_joint_positions)
        print(f'\n--current "{joint_positions_str}"')

def main():
    # Parse command line arguments
    if len(sys.argv) < 2:
        target_x, target_y, target_z = 100, 50, 300
        print(f"Usage: python3 {os.path.basename(__file__)} x,y,z [--current \"joint_positions\"] [--no-collision]")
        print("Using default target: (100, 50, 300)")
    else:
        target_x, target_y, target_z = parse_coordinates(sys.argv[1])
    
    # Parse optional arguments
    current_positions = None
    enable_collision = True
    
    for i in range(2, len(sys.argv)):
        if sys.argv[i] == "--current" and i + 1 < len(sys.argv):
            current_positions = parse_current_positions(sys.argv[i + 1])
        elif sys.argv[i] == "--no-collision":
            enable_collision = False
    
    try:
        import delta_robot
        
        if enable_collision:
            # NEW: Collision-aware approach with optimized segment calculation
            motor_result, collision_result, segment_result = solve_with_collision_detection(
                target_x, target_y, target_z, current_positions, enable_obstacles=True)
            
            if motor_result:
                print_motor_results(motor_result, "COLLISION-AWARE MOTOR RESULTS")
                
                print(f"\n=== PERFORMANCE SUMMARY ===")
                if collision_result:
                    print(f"FABRIK solving: {collision_result.fabrik_result.solve_time_ms:.2f}ms")
                    print(f"Collision detection: {collision_result.total_collision_time_ms:.2f}ms")
                if segment_result:
                    print(f"Segment calculation: {segment_result.calculation_time_ms:.2f}ms")
                print(f"Motor calculations: {motor_result.solve_time_ms:.2f}ms (included in FABRIK)")
                
                total_time = (collision_result.fabrik_result.solve_time_ms if collision_result else 0) + \
                           (collision_result.total_collision_time_ms if collision_result else 0) + \
                           (segment_result.calculation_time_ms if segment_result else 0)
                print(f"Total optimized pipeline: {total_time:.2f}ms")
        else:
            # Traditional approach (for comparison)
            motor_result = solve_without_collision_detection(target_x, target_y, target_z, current_positions)
            print_motor_results(motor_result, "TRADITIONAL MOTOR RESULTS")
        
    except ImportError as e:
        print(f"ImportError: {e}")
        print("Please ensure the compiled C++ module is in the Python path.")
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()