#!/usr/bin/env python3
"""
Visual Collision-Aware Delta Robot Motor Module Test
Same functionality as main.py but with detailed 3D visualization using Plotly
"""
import sys
import os
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import time

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
            position_strs = current_str.split(':')
        else:
            current_str = current_str.replace('(', '').replace(')', '')
            position_strs = current_str.split(',')
            position_strs = [','.join(position_strs[i:i+3]) for i in range(0, len(position_strs), 3)]
        
        positions = []
        for pos_str in position_strs:
            coords = [float(x.strip()) for x in pos_str.split(',')]
            if len(coords) != 3:
                raise ValueError(f"Expected 3 coordinates per position, got {len(coords)}")
            positions.append(np.array([coords[0], coords[1], coords[2]]))
        
        expected_joints = expected_segments + 2
        if len(positions) != expected_joints:
            print(f"Warning: Expected {expected_joints} joint positions, got {len(positions)}. Using straight-up fallback.")
            return None
            
        return positions
    except Exception as e:
        print(f"Error parsing current positions '{current_str}': {e}")
        print("Using straight-up fallback.")
        return None

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

def add_obstacle_to_plot(fig, obstacle, index):
    """Add obstacle sphere to the plot"""
    center = obstacle.center
    radius = obstacle.radius
    
    # Create sphere
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0, np.pi, 20)
    x = radius * np.outer(np.cos(u), np.sin(v)) + center[0]
    y = radius * np.outer(np.sin(u), np.sin(v)) + center[1]
    z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + center[2]
    
    fig.add_trace(go.Surface(
        x=x, y=y, z=z, 
        opacity=0.6, 
        colorscale='Reds', 
        showscale=False, 
        name=f'Obstacle {index+1}'
    ))

def add_chain_to_plot(fig, chain, name, color, line_style='solid', marker_size=5):
    """Add a FABRIK chain to the plot"""
    joints = []
    for joint in chain.joints:
        pos = joint.position
        joints.append([pos[0], pos[1], pos[2]])
    joints = np.array(joints)
    
    line_dict = dict(color=color, width=4)
    if line_style == 'dash':
        line_dict['dash'] = 'dash'
    elif line_style == 'dot':
        line_dict['dash'] = 'dot'
    
    fig.add_trace(go.Scatter3d(
        x=joints[:, 0], 
        y=joints[:, 1], 
        z=joints[:, 2],
        mode='markers+lines',
        marker=dict(size=marker_size, color=color),
        line=line_dict,
        name=name
    ))

def add_points_to_plot(fig, points, name, color, marker_symbol='circle', marker_size=6):
    """Add a list of points to the plot"""
    if len(points) == 0:
        return
    
    points_array = []
    for point in points:
        points_array.append([point[0], point[1], point[2]])
    points_array = np.array(points_array)
    
    fig.add_trace(go.Scatter3d(
        x=points_array[:, 0], 
        y=points_array[:, 1], 
        z=points_array[:, 2],
        mode='markers+lines',
        marker=dict(size=marker_size, color=color, symbol=marker_symbol),
        line=dict(color=color, width=3),
        name=name
    ))

def add_tube_to_plot(fig, points, diameter, name, color, opacity=0.7):
    """Add a tube visualization around a path with given diameter"""
    if len(points) == 0:
        return
    
    # Convert points to numpy array
    points_array = []
    for point in points:
        points_array.append([point[0], point[1], point[2]])
    points_array = np.array(points_array)
    
    if len(points_array) < 2:
        return
    
    radius = diameter / 2.0
    
    # Create tube mesh along the path
    tube_x = []
    tube_y = []
    tube_z = []
    
    # Number of segments around the tube circumference
    n_circumference = 16
    
    # Create a consistent reference frame to prevent twisting
    # Use the first direction as reference
    first_direction = points_array[1] - points_array[0] if len(points_array) > 1 else np.array([0, 0, 1])
    first_direction = first_direction / np.linalg.norm(first_direction)
    
    # Create initial perpendicular vectors
    if abs(first_direction[2]) < 0.9:
        ref_v1 = np.cross(first_direction, [0, 0, 1])
    else:
        ref_v1 = np.cross(first_direction, [1, 0, 0])
    ref_v1 = ref_v1 / np.linalg.norm(ref_v1)
    ref_v2 = np.cross(first_direction, ref_v1)
    ref_v2 = ref_v2 / np.linalg.norm(ref_v2)
    
    for i in range(len(points_array)):
        if i == 0:
            # First point - use first direction
            direction = first_direction
            v1, v2 = ref_v1, ref_v2
        elif i == len(points_array) - 1:
            # Last point - use direction from previous to current
            direction = points_array[i] - points_array[i-1]
            direction = direction / np.linalg.norm(direction)
        else:
            # Middle points - use direction from current to next point
            direction = points_array[i+1] - points_array[i]
            direction = direction / np.linalg.norm(direction)
        
        # For points after the first, calculate perpendicular vectors that minimize twisting
        if i > 0:
            # Project previous v1 onto plane perpendicular to current direction
            v1_projected = ref_v1 - np.dot(ref_v1, direction) * direction
            if np.linalg.norm(v1_projected) > 1e-6:
                v1 = v1_projected / np.linalg.norm(v1_projected)
            else:
                # Fallback if projection fails
                if abs(direction[2]) < 0.9:
                    v1 = np.cross(direction, [0, 0, 1])
                else:
                    v1 = np.cross(direction, [1, 0, 0])
                v1 = v1 / np.linalg.norm(v1)
            v2 = np.cross(direction, v1)
            v2 = v2 / np.linalg.norm(v2)
            
            # Update reference for next iteration
            ref_v1, ref_v2 = v1, v2
        
        # Create circle around this point
        circle_x = []
        circle_y = []
        circle_z = []
        
        for j in range(n_circumference + 1):  # +1 to close the circle
            angle = 2 * np.pi * j / n_circumference
            offset = radius * (np.cos(angle) * v1 + np.sin(angle) * v2)
            point = points_array[i] + offset
            circle_x.append(point[0])
            circle_y.append(point[1])
            circle_z.append(point[2])
        
        tube_x.append(circle_x)
        tube_y.append(circle_y)
        tube_z.append(circle_z)
    
    # Convert to numpy arrays for surface plotting
    tube_x = np.array(tube_x)
    tube_y = np.array(tube_y)
    tube_z = np.array(tube_z)
    
    # Add surface with higher opacity and better visual properties
    fig.add_trace(go.Surface(
        x=tube_x, y=tube_y, z=tube_z,
        opacity=opacity,
        colorscale=[[0, color], [1, color]],
        showscale=False,
        name=name,
        lighting=dict(ambient=0.8, diffuse=0.8, specular=0.1),
        hovertemplate=f'{name}<br>Diameter: {diameter}mm<extra></extra>'
    ))

def solve_and_visualize_collision_aware(target_x, target_y, target_z, current_positions=None, enable_obstacles=True):
    """Solve using collision-aware approach with detailed visualization"""
    import delta_robot
    
    target = np.array([target_x, target_y, target_z])
    
    # Create test obstacles if enabled
    obstacles = create_test_scenario_obstacles() if enable_obstacles else []
    
    print(f"\n=== COLLISION-AWARE SOLVING WITH VISUALIZATION ===")
    print(f"Target: ({target_x}, {target_y}, {target_z})")
    print(f"Obstacles: {len(obstacles)} in workspace")
    print(f"Using {'provided' if current_positions else 'straight-up'} initialization")
    
    # Configure collision-aware solving
    config = delta_robot.collision.CollisionAwareConfig()
    config.max_collision_iterations = 3
    config.spline_diameter = delta_robot.DEFAULT_SPLINE_DIAMETER
    config.enable_collision_detection = len(obstacles) > 0
    config.verbose_logging = True
    
    # Create figure for visualization
    fig = go.Figure()
    
    # Add obstacles to plot first
    for i, obs in enumerate(obstacles):
        add_obstacle_to_plot(fig, obs, i)
    
    # Add target point
    fig.add_trace(go.Scatter3d(
        x=[target[0]], y=[target[1]], z=[target[2]],
        mode='markers',
        marker=dict(size=15, color='gold', symbol='diamond'),
        name='Target'
    ))
    
    try:
        # Step 1: Initial FABRIK solution
        print("\nStep 1: Initial FABRIK solution...")
        if current_positions is not None:
            # Use motor module which handles initialization internally
            motor_result_initial = delta_robot.motor.MotorModule.calculate_motors(target, current_positions)
            # Extract the FABRIK result from motor result
            initial_fabrik = type('FabrikResult', (), {
                'final_chain': type('Chain', (), {'joints': [type('Joint', (), {'position': pos})() for pos in motor_result_initial.fabrik_joint_positions]})(),
                'converged': motor_result_initial.fabrik_converged,
                'final_error': motor_result_initial.fabrik_error
            })()
        else:
            # Use simple FABRIK solver
            initial_fabrik = delta_robot.fabrik.solve_delta_robot(delta_robot.DEFAULT_ROBOT_SEGMENTS, target, delta_robot.FABRIK_TOLERANCE)
        
        print(f"   Initial FABRIK converged: {initial_fabrik.converged}")
        print(f"   Initial error: {initial_fabrik.final_error:.3f}mm")
        
        # Add initial solution to plot
        add_chain_to_plot(fig, initial_fabrik.final_chain, '1. Initial FABRIK', 'blue', 'dash')
        
        if len(obstacles) == 0:
            print("No obstacles - showing initial FABRIK solution only")
            
            # Calculate segments for motor results
            segment_result = delta_robot.motor.SegmentCalculator.calculate_segment_end_effectors(
                initial_fabrik.final_chain)
            
            # Update layout and show
            fig.update_layout(
                title=f'No Collision Detection - Initial FABRIK Solution<br>Target: ({target_x}, {target_y}, {target_z})<br>Error: {initial_fabrik.final_error:.3f}mm',
                scene=dict(
                    xaxis_title='X (mm)',
                    yaxis_title='Y (mm)', 
                    zaxis_title='Z (mm)',
                    aspectmode='data'
                ),
                height=800
            )
            fig.show()
            
            return initial_fabrik, None, segment_result
        
        # Step 2: Extract U points from initial solution
        print("Step 2: Extracting U points...")
        u_points = delta_robot.collision.UPointsExtractor.extract_u_points(initial_fabrik.final_chain)
        print(f"   Extracted {len(u_points)} U points")
        
        # Add U points to plot
        add_points_to_plot(fig, u_points, '2. U Points (Collision Detection)', 'red', 'circle', 7)
        
        # Add tube visualization around U points path to show collision envelope
        print("   Adding collision tube visualization...")
        add_tube_to_plot(fig, u_points, config.spline_diameter, 
                        f'2b. Collision Tube (Ø{config.spline_diameter}mm)', 'red', opacity=0.5)
        
        # Step 3: Run collision-aware solving with iteration tracking
        print("Step 3: Running collision-aware solving...")
        
        collision_result = delta_robot.collision.CollisionAwareSolver.solve_with_collision_avoidance(
            target, obstacles, config)
        
        print(f"\n=== COLLISION SOLVING RESULTS ===")
        print(f"✓ Collision-free solution: {collision_result.collision_free}")
        print(f"✓ FABRIK converged: {collision_result.fabrik_result.converged}")
        print(f"✓ Final error: {collision_result.fabrik_result.final_error:.4f}")
        print(f"✓ Collision iterations: {collision_result.collision_iterations}")
        print(f"✓ Total collision time: {collision_result.total_collision_time_ms:.2f}ms")
        
        # Step 4: Show collision detection history
        print("Step 4: Visualizing collision detection iterations...")
        
        # Re-run collision detection to get waypoints for visualization
        collision_check = delta_robot.collision.CollisionDetector.check_and_avoid(
            u_points, obstacles, config.spline_diameter)
        
        if len(collision_check.waypoints) > 0:
            add_points_to_plot(fig, collision_check.waypoints, 
                             f'3. Swarm Waypoints (min_dist: {collision_check.min_distance:.1f}mm)', 
                             'orange', 'circle', 8)
            
            # Add tube around swarm waypoints to show optimized collision envelope
            add_tube_to_plot(fig, collision_check.waypoints, config.spline_diameter,
                           f'3b. Optimized Tube (Ø{config.spline_diameter}mm)', 'orange', opacity=0.4)
            
            # Convert waypoints to joints for visualization
            conversion_result = delta_robot.collision.WaypointConverter.convert_waypoints_to_joints(
                collision_check.waypoints)
            
            if conversion_result.conversion_successful:
                add_points_to_plot(fig, conversion_result.joint_positions, 
                                 '4. Converted Joint Positions', 'purple', 'square', 6)
        
        # Step 5: Add final solution
        print("Step 5: Adding final collision-aware solution...")
        
        # Add final solution
        add_chain_to_plot(fig, collision_result.fabrik_result.final_chain, 
                         '5. Final Collision-Aware Solution', 'green', 'solid', 7)
        
        # Add tube around final solution to show final collision envelope
        final_u_points = delta_robot.collision.UPointsExtractor.extract_u_points(collision_result.fabrik_result.final_chain)
        add_tube_to_plot(fig, final_u_points, config.spline_diameter,
                       f'5b. Final Solution Tube (Ø{config.spline_diameter}mm)', 'green', opacity=0.3)
        
        # Step 6: Calculate segment end-effectors
        print("Step 6: Calculating segment end-effectors...")
        segment_result = delta_robot.motor.SegmentCalculator.calculate_segment_end_effectors(
            collision_result.fabrik_result.final_chain)
        print(f"   Calculated {len(segment_result.segment_end_effectors)} segments in {segment_result.calculation_time_ms:.2f}ms")
        
        # Add segment end-effectors to plot
        if segment_result.calculation_successful:
            segment_positions = [seg.end_effector_position for seg in segment_result.segment_end_effectors]
            add_points_to_plot(fig, segment_positions, '6. Segment End-Effectors', 'cyan', 'diamond', 10)
        
        # Update layout with comprehensive information
        status = "SUCCESS" if collision_result.collision_free else "PARTIAL"
        collision_info = f"Min distance: {collision_check.min_distance:.1f}mm (Tube Ø{config.spline_diameter}mm)" if len(obstacles) > 0 else "No obstacles"
        
        fig.update_layout(
            title=f'Complete Collision-Aware Pipeline with Tube Visualization - {status}<br>'
                  f'Target: ({target_x}, {target_y}, {target_z}) | {collision_info}<br>'
                  f'Iterations: {collision_result.collision_iterations} | Time: {collision_result.total_collision_time_ms:.1f}ms | '
                  f'Final Error: {collision_result.fabrik_result.final_error:.3f}mm',
            scene=dict(
                xaxis_title='X (mm)',
                yaxis_title='Y (mm)',
                zaxis_title='Z (mm)',
                aspectmode='data',
                camera=dict(
                    eye=dict(x=1.3, y=1.3, z=1.2)
                )
            ),
            height=900,
            legend=dict(
                x=0.02,
                y=0.98,
                bgcolor='rgba(255,255,255,0.9)',
                font=dict(size=9)
            )
        )
        
        # Show the plot
        print("Displaying visualization...")
        fig.show()
        
        return collision_result.fabrik_result, collision_result, segment_result
        
    except Exception as e:
        print(f"Error in collision-aware solving: {e}")
        import traceback
        traceback.print_exc()
        return None, None, None

def solve_without_collision_detection_visual(target_x, target_y, target_z, current_positions=None):
    """Solve using traditional approach with basic visualization"""
    import delta_robot
    
    print(f"\n=== TRADITIONAL SOLVING WITH VISUALIZATION ===")
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
        
        # Create simple visualization
        fig = go.Figure()
        
        # Add target
        fig.add_trace(go.Scatter3d(
            x=[target_x], y=[target_y], z=[target_z],
            mode='markers',
            marker=dict(size=15, color='gold', symbol='diamond'),
            name='Target'
        ))
        
        # Add FABRIK joint positions
        if hasattr(result, 'fabrik_joint_positions') and result.fabrik_joint_positions:
            joints = []
            for joint_pos in result.fabrik_joint_positions:
                if hasattr(joint_pos, 'shape'):  # numpy array
                    joints.append([joint_pos[0], joint_pos[1], joint_pos[2]])
                else:
                    joints.append([joint_pos.x(), joint_pos.y(), joint_pos.z()])
            joints = np.array(joints)
            
            fig.add_trace(go.Scatter3d(
                x=joints[:, 0], y=joints[:, 1], z=joints[:, 2],
                mode='markers+lines',
                marker=dict(size=6, color='blue'),
                line=dict(color='blue', width=4),
                name='FABRIK Solution'
            ))
        
        # Add segment end-effectors
        if result.original_segment_positions:
            segments = []
            for seg_pos in result.original_segment_positions:
                if hasattr(seg_pos, 'shape'):  # numpy array
                    segments.append([seg_pos[0], seg_pos[1], seg_pos[2]])
                else:
                    segments.append([seg_pos.x(), seg_pos.y(), seg_pos.z()])
            segments = np.array(segments)
            
            fig.add_trace(go.Scatter3d(
                x=segments[:, 0], y=segments[:, 1], z=segments[:, 2],
                mode='markers',
                marker=dict(size=10, color='green', symbol='diamond'),
                name='Segment End-Effectors'
            ))
        
        fig.update_layout(
            title=f'Traditional Motor Solution (No Collision)<br>Target: ({target_x}, {target_y}, {target_z}) | Error: {result.fabrik_error:.3f}mm',
            scene=dict(
                xaxis_title='X (mm)',
                yaxis_title='Y (mm)',
                zaxis_title='Z (mm)',
                aspectmode='data'
            ),
            height=800
        )
        
        fig.show()
        
        return result
        
    except Exception as e:
        print(f"Error in traditional solving: {e}")
        import traceback
        traceback.print_exc()
        return None

def print_motor_results(motor_result, collision_result=None, segment_result=None, title="MOTOR RESULTS"):
    """Print motor calculation results with performance breakdown"""
    if motor_result is None:
        print("No motor results to display.")
        return
    
    print(f"\n=== {title} ===")
    
    if hasattr(motor_result, 'levels') and motor_result.levels:
        # Print segment information
        for level_idx, level_data in enumerate(motor_result.levels):
            if hasattr(motor_result, 'original_segment_numbers') and level_idx < len(motor_result.original_segment_numbers):
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
    
    # Performance breakdown
    print(f"\n=== PERFORMANCE BREAKDOWN ===")
    if hasattr(motor_result, 'solve_time_ms'):
        print(f"FABRIK solving: {motor_result.solve_time_ms:.2f}ms")
    if collision_result:
        print(f"Collision detection: {collision_result.total_collision_time_ms:.2f}ms")
    if segment_result:
        print(f"Segment calculation: {segment_result.calculation_time_ms:.2f}ms")
    elif hasattr(motor_result, 'segment_calculation_time_ms'):
        print(f"Segment calculation: {motor_result.segment_calculation_time_ms:.2f}ms")

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
            # Collision-aware approach with detailed visualization
            fabrik_result, collision_result, segment_result = solve_and_visualize_collision_aware(
                target_x, target_y, target_z, current_positions, enable_obstacles=True)
            
            if fabrik_result:
                # Calculate motor results using collision-free joint positions
                fabrik_joint_positions = []
                for joint in fabrik_result.final_chain.joints:
                    if hasattr(joint.position, 'shape'):
                        fabrik_joint_positions.append(joint.position)
                    else:
                        fabrik_joint_positions.append(np.array([joint.position.x(), joint.position.y(), joint.position.z()]))
                
                motor_result = delta_robot.motor.MotorModule.calculate_motors(
                    target_x, target_y, target_z, fabrik_joint_positions)
                
                print_motor_results(motor_result, collision_result, segment_result, "COLLISION-AWARE MOTOR RESULTS")
        else:
            # Traditional approach with basic visualization
            motor_result = solve_without_collision_detection_visual(target_x, target_y, target_z, current_positions)
            print_motor_results(motor_result, title="TRADITIONAL MOTOR RESULTS")
        
    except ImportError as e:
        print(f"ImportError: {e}")
        print("Please ensure the compiled C++ module and plotly are installed.")
        print("Install plotly with: pip install plotly")
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()