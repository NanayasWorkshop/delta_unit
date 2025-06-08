#!/usr/bin/env python3
"""
Visual Collision-Aware Delta Robot Motor Module Test
Based on the clean visualization approach from test_collision_aware_solver.py
"""
import sys
import os
import numpy as np
import plotly.graph_objects as go
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

def create_spline_from_u_points(u_points, num_samples=100):
    """
    Create the same spline that collision detection uses (Catmull-Rom spline)
    This replicates the logic from ConicalSwarmSplineAvoider::createSpline
    """
    if len(u_points) < 2:
        return np.array([])
    
    # Convert u_points to numpy array for easier manipulation
    points = np.array([[pt[0], pt[1], pt[2]] if hasattr(pt, '__getitem__') 
                      else [pt.x(), pt.y(), pt.z()] for pt in u_points])
    
    spline_points = []
    
    for i in range(num_samples):
        t = i / (num_samples - 1)
        scaled_t = t * (len(points) - 1)
        segment = int(scaled_t)
        local_t = scaled_t - segment
        
        if segment >= len(points) - 1:
            spline_points.append(points[-1])
            continue
        
        # Get control points for Catmull-Rom spline
        p0 = points[segment - 1] if segment > 0 else points[segment]
        p1 = points[segment]
        p2 = points[segment + 1]
        p3 = points[segment + 2] if segment < len(points) - 2 else points[segment + 1]
        
        # Catmull-Rom interpolation
        interpolated = 0.5 * (
            2.0 * p1 +
            (-p0 + p2) * local_t +
            (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * local_t * local_t +
            (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * local_t * local_t * local_t
        )
        
        spline_points.append(interpolated)
    
    return np.array(spline_points)

def add_spline_tube_to_plot(fig, spline_points, diameter, name, color, opacity=0.3):
    """
    Add a tube visualization around a SPLINE PATH with given diameter
    This shows the actual collision envelope that the collision detection uses
    """
    if len(spline_points) == 0:
        return
    
    # Ensure spline_points is a numpy array
    if not isinstance(spline_points, np.ndarray):
        spline_array = []
        for point in spline_points:
            if hasattr(point, '__getitem__'):  # numpy array or list
                spline_array.append([point[0], point[1], point[2]])
            else:  # Eigen vector
                spline_array.append([point.x(), point.y(), point.z()])
        spline_points = np.array(spline_array)
    
    if len(spline_points) < 2:
        return
    
    radius = diameter / 2.0
    
    # Create tube mesh along the spline path
    tube_x = []
    tube_y = []
    tube_z = []
    
    # Number of segments around the tube circumference
    n_circumference = 12  # Reduced for performance
    
    # Create a consistent reference frame to prevent twisting
    first_direction = spline_points[1] - spline_points[0] if len(spline_points) > 1 else np.array([0, 0, 1])
    first_direction = first_direction / np.linalg.norm(first_direction)
    
    # Create initial perpendicular vectors
    if abs(first_direction[2]) < 0.9:
        ref_v1 = np.cross(first_direction, [0, 0, 1])
    else:
        ref_v1 = np.cross(first_direction, [1, 0, 0])
    ref_v1 = ref_v1 / np.linalg.norm(ref_v1)
    ref_v2 = np.cross(first_direction, ref_v1)
    ref_v2 = ref_v2 / np.linalg.norm(ref_v2)
    
    for i in range(len(spline_points)):
        if i == 0:
            direction = first_direction
            v1, v2 = ref_v1, ref_v2
        elif i == len(spline_points) - 1:
            direction = spline_points[i] - spline_points[i-1]
            direction = direction / np.linalg.norm(direction)
        else:
            direction = spline_points[i+1] - spline_points[i]
            direction = direction / np.linalg.norm(direction)
        
        # For points after the first, calculate perpendicular vectors that minimize twisting
        if i > 0:
            v1_projected = ref_v1 - np.dot(ref_v1, direction) * direction
            if np.linalg.norm(v1_projected) > 1e-6:
                v1 = v1_projected / np.linalg.norm(v1_projected)
            else:
                if abs(direction[2]) < 0.9:
                    v1 = np.cross(direction, [0, 0, 1])
                else:
                    v1 = np.cross(direction, [1, 0, 0])
                v1 = v1 / np.linalg.norm(v1)
            v2 = np.cross(direction, v1)
            v2 = v2 / np.linalg.norm(v2)
            ref_v1, ref_v2 = v1, v2
        
        # Create circle around this point
        circle_x = []
        circle_y = []
        circle_z = []
        
        for j in range(n_circumference + 1):  # +1 to close the circle
            angle = 2 * np.pi * j / n_circumference
            offset = radius * (np.cos(angle) * v1 + np.sin(angle) * v2)
            point = spline_points[i] + offset
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
    
    # Add surface
    fig.add_trace(go.Surface(
        x=tube_x, y=tube_y, z=tube_z,
        opacity=opacity,
        colorscale=[[0, color], [1, color]],
        showscale=False,
        name=name,
        lighting=dict(ambient=0.8, diffuse=0.8, specular=0.1),
        hovertemplate=f'{name}<br>Diameter: {diameter}mm<extra></extra>'
    ))

def create_detailed_pipeline_visualization(target_x, target_y, target_z, current_positions=None, enable_obstacles=True):
    """Create detailed visualization of the complete collision-aware pipeline - based on test file approach"""
    try:
        import delta_robot
        import time
        
        print("\n=== CREATING DETAILED PIPELINE VISUALIZATION ===")
        
        # Start total timing
        total_start_time = time.time()
        
        # Setup test scenario
        target = np.array([target_x, target_y, target_z])
        obstacles = delta_robot.collision.CollisionDetector.create_test_obstacles() if enable_obstacles else []
        
        print(f"Target: ({target_x}, {target_y}, {target_z})")
        print(f"Obstacles: {len(obstacles)} in workspace")
        print("Running complete collision-aware pipeline...")
        
        # Step 1: Initial FABRIK solution
        step1_start = time.time()
        print("Step 1: Initial FABRIK solution")
        if current_positions is not None:
            # Use motor module for initialization
            motor_result_initial = delta_robot.motor.MotorModule.calculate_motors(target, current_positions)
            # Create FABRIK-like structure from motor result
            initial_joints = []
            for joint_pos in motor_result_initial.fabrik_joint_positions:
                if hasattr(joint_pos, 'shape'):
                    initial_joints.append(joint_pos)
                else:
                    initial_joints.append(np.array([joint_pos.x(), joint_pos.y(), joint_pos.z()]))
            initial_fabrik_error = motor_result_initial.fabrik_error
            initial_solve_time = motor_result_initial.solve_time_ms
        else:
            initial_fabrik = delta_robot.fabrik.solve_delta_robot(7, target, delta_robot.FABRIK_TOLERANCE)
            initial_joints = []
            for joint in initial_fabrik.final_chain.joints:
                pos = joint.position
                initial_joints.append([pos[0], pos[1], pos[2]])
            initial_fabrik_error = initial_fabrik.final_error
            initial_solve_time = initial_fabrik.solve_time_ms
        step1_time = (time.time() - step1_start) * 1000
        
        # Step 2: Extract U points from initial FABRIK
        step2_start = time.time()
        print("Step 2: Extracting U points")
        if current_positions is not None:
            u_points = delta_robot.collision.UPointsExtractor.extract_u_points_from_positions(initial_joints)
        else:
            u_points = delta_robot.collision.UPointsExtractor.extract_u_points(initial_fabrik.final_chain)
        step2_time = (time.time() - step2_start) * 1000
        
        # Step 3: Run collision detection and get swarm-optimized waypoints
        step3_start = time.time()
        print("Step 3: Collision detection and swarm optimization")
        collision_result = delta_robot.collision.CollisionDetector.check_and_avoid(
            u_points, obstacles, delta_robot.DEFAULT_SPLINE_DIAMETER
        )
        step3_time = collision_result.computation_time
        
        # Step 4: Convert waypoints back to joint positions
        step4_start = time.time()
        print("Step 4: Converting waypoints to joint positions")
        conversion_result = delta_robot.collision.WaypointConverter.convert_waypoints_to_joints(
            collision_result.waypoints
        )
        step4_time = (time.time() - step4_start) * 1000
        
        # Step 5: Final FABRIK solution with collision-free initial positions
        step5_start = time.time()
        print("Step 5: Final FABRIK solution")
        if conversion_result.conversion_successful and len(collision_result.waypoints) > 0:
            final_fabrik_chain = delta_robot.collision.WaypointConverter.create_fabrik_chain_from_waypoints(
                collision_result.waypoints, 7
            )
            final_fabrik = delta_robot.fabrik.FabrikSolver.solve(final_fabrik_chain, target)
            final_joints = []
            for joint in final_fabrik.final_chain.joints:
                pos = joint.position
                final_joints.append([pos[0], pos[1], pos[2]])
            final_fabrik_solve_time = final_fabrik.solve_time_ms
        else:
            final_joints = initial_joints
            final_fabrik = type('FabrikResult', (), {'final_error': initial_fabrik_error, 'solve_time_ms': 0})()
            final_fabrik_solve_time = 0
        step5_time = (time.time() - step5_start) * 1000
        
        # Step 6: Final motor calculation (like test file approach)
        step6_start = time.time()
        print("Step 6: Final motor calculation with collision-free solution")
        
        # Convert final joints to proper format for motor module
        final_joint_positions = []
        for joint_pos in final_joints:
            if hasattr(joint_pos, '__len__') and len(joint_pos) == 3:
                final_joint_positions.append(np.array(joint_pos))
            else:
                final_joint_positions.append(joint_pos)
        
        # Calculate final motor result
        final_motor_result = delta_robot.motor.MotorModule.calculate_motors(
            target_x, target_y, target_z, final_joint_positions)
        step6_time = final_motor_result.solve_time_ms
        
        # Step 7: Verify final solution is collision-free
        step7_start = time.time()
        print("Step 7: Verifying final motor solution")
        final_motor_joints = []
        for joint_pos in final_motor_result.fabrik_joint_positions:
            if hasattr(joint_pos, 'shape'):
                final_motor_joints.append(joint_pos)
            else:
                final_motor_joints.append(np.array([joint_pos.x(), joint_pos.y(), joint_pos.z()]))
        
        final_u_points = delta_robot.collision.UPointsExtractor.extract_u_points_from_positions(final_motor_joints)
        final_collision_check = delta_robot.collision.CollisionDetector.check_and_avoid(
            final_u_points, obstacles, delta_robot.DEFAULT_SPLINE_DIAMETER)
        step7_time = final_collision_check.computation_time
        
        print(f"   Final solution collision-free: {not final_collision_check.has_collision}")
        print(f"   Final min distance: {final_collision_check.min_distance:.2f}mm")
        
        # CRITICAL FIX: If Step 7 found collision, we need to use the corrected waypoints!
        if final_collision_check.has_collision and len(final_collision_check.waypoints) > 0:
            print("   Step 7 detected collision - using corrected waypoints for visualization")
            
            # Convert the corrected waypoints to joint positions
            step7_conversion = delta_robot.collision.WaypointConverter.convert_waypoints_to_joints(
                final_collision_check.waypoints)
            
            if step7_conversion.conversion_successful:
                # Use the collision-corrected joint positions for visualization
                final_motor_joints = []
                for joint_pos in step7_conversion.joint_positions:
                    if hasattr(joint_pos, '__len__'):
                        final_motor_joints.append(np.array(joint_pos))
                    else:
                        final_motor_joints.append(joint_pos)
                
                # Recalculate U points from corrected positions
                final_u_points = delta_robot.collision.UPointsExtractor.extract_u_points_from_positions(final_motor_joints)
                
                print(f"   Using collision-corrected solution for visualization")
        
        # Calculate total time
        total_time = (time.time() - total_start_time) * 1000
        
        # Create visualization (exactly like test file)
        print("Creating visualization...")
        fig = go.Figure()
        
        # Add obstacles
        for i, obs in enumerate(obstacles):
            center = obs.center
            radius = obs.radius
            
            # Create sphere
            u = np.linspace(0, 2 * np.pi, 15)
            v = np.linspace(0, np.pi, 15)
            x = radius * np.outer(np.cos(u), np.sin(v)) + center[0]
            y = radius * np.outer(np.sin(u), np.sin(v)) + center[1]
            z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + center[2]
            
            fig.add_trace(go.Surface(
                x=x, y=y, z=z, 
                opacity=0.5, 
                colorscale='Reds', 
                showscale=False, 
                name=f'Obstacle {i+1}'
            ))
        
        # Step 1: Initial FABRIK solution (blue dashed)
        initial_joints_array = np.array(initial_joints)
        fig.add_trace(go.Scatter3d(
            x=initial_joints_array[:, 0], 
            y=initial_joints_array[:, 1], 
            z=initial_joints_array[:, 2],
            mode='markers+lines',
            marker=dict(size=5, color='blue'),
            line=dict(color='blue', width=3, dash='dash'),
            name='1. Initial FABRIK'
        ))
        
        # Step 2: U points (red)
        u_points_array = []
        for u_point in u_points:
            if hasattr(u_point, '__len__'):
                u_points_array.append([u_point[0], u_point[1], u_point[2]])
            else:
                u_points_array.append([u_point.x(), u_point.y(), u_point.z()])
        u_points_array = np.array(u_points_array)
        
        fig.add_trace(go.Scatter3d(
            x=u_points_array[:, 0], 
            y=u_points_array[:, 1], 
            z=u_points_array[:, 2],
            mode='markers+lines',
            marker=dict(size=7, color='red'),
            line=dict(color='red', width=4),
            name='2. U Points'
        ))
        
        # Step 3: Swarm-optimized waypoints (orange)
        if len(collision_result.waypoints) > 0:
            waypoints_array = []
            for waypoint in collision_result.waypoints:
                if hasattr(waypoint, '__len__'):
                    waypoints_array.append([waypoint[0], waypoint[1], waypoint[2]])
                else:
                    waypoints_array.append([waypoint.x(), waypoint.y(), waypoint.z()])
            waypoints_array = np.array(waypoints_array)
            
            fig.add_trace(go.Scatter3d(
                x=waypoints_array[:, 0], 
                y=waypoints_array[:, 1], 
                z=waypoints_array[:, 2],
                mode='markers+lines',
                marker=dict(size=8, color='orange'),
                line=dict(color='orange', width=5),
                name='3. Swarm Waypoints'
            ))
        
        # Step 4: Converted joint positions (purple) 
        if conversion_result.conversion_successful:
            converted_joints = []
            for joint_pos in conversion_result.joint_positions:
                if hasattr(joint_pos, '__len__'):
                    converted_joints.append([joint_pos[0], joint_pos[1], joint_pos[2]])
                else:
                    converted_joints.append([joint_pos.x(), joint_pos.y(), joint_pos.z()])
            converted_joints = np.array(converted_joints)
            
            fig.add_trace(go.Scatter3d(
                x=converted_joints[:, 0], 
                y=converted_joints[:, 1], 
                z=converted_joints[:, 2],
                mode='markers+lines',
                marker=dict(size=6, color='purple'),
                line=dict(color='purple', width=4, dash='dot'),
                name='4. Converted Joints'
            ))
        
        # Step 5: Final FABRIK solution (green)
        final_joints_array = np.array(final_joints)
        fig.add_trace(go.Scatter3d(
            x=final_joints_array[:, 0], 
            y=final_joints_array[:, 1], 
            z=final_joints_array[:, 2],
            mode='markers+lines',
            marker=dict(size=7, color='green'),
            line=dict(color='green', width=6),
            name='5. Final FABRIK'
        ))
        
        # Step 6: Final MOTOR solution (dark blue - most important)
        final_motor_joints_array = np.array(final_motor_joints)
        final_color = 'red' if final_collision_check.has_collision else 'darkblue'
        final_name = '6. Final MOTOR Solution ❌' if final_collision_check.has_collision else '6. Final MOTOR Solution ✅'
        
        fig.add_trace(go.Scatter3d(
            x=final_motor_joints_array[:, 0], 
            y=final_motor_joints_array[:, 1], 
            z=final_motor_joints_array[:, 2],
            mode='markers+lines',
            marker=dict(size=8, color=final_color),
            line=dict(color=final_color, width=7),
            name=final_name
        ))
        
        # NEW: Add tube visualization around the FINAL MOTOR solution
        print("   Adding collision tube around final motor solution...")
        final_motor_spline = create_spline_from_u_points(final_u_points, num_samples=80)
        
        if len(final_motor_spline) > 0:
            # Add the spline path first
            fig.add_trace(go.Scatter3d(
                x=final_motor_spline[:, 0], 
                y=final_motor_spline[:, 1], 
                z=final_motor_spline[:, 2],
                mode='lines',
                line=dict(color=final_color, width=2, dash='dot'),
                name='6b. Final Motor Spline Path',
                opacity=0.8
            ))
            
            # Add tube around the final motor solution
            tube_color = 'red' if final_collision_check.has_collision else 'darkblue'
            tube_name = f'6c. Final Motor Tube (Ø{delta_robot.DEFAULT_SPLINE_DIAMETER}mm) ❌' if final_collision_check.has_collision else f'6c. Final Motor Tube (Ø{delta_robot.DEFAULT_SPLINE_DIAMETER}mm) ✅'
            tube_opacity = 0.4 if final_collision_check.has_collision else 0.2
            
            add_spline_tube_to_plot(fig, final_motor_spline, delta_robot.DEFAULT_SPLINE_DIAMETER,
                           tube_name, tube_color, opacity=tube_opacity)
        
        # Add target (gold diamond)
        fig.add_trace(go.Scatter3d(
            x=[target[0]], y=[target[1]], z=[target[2]],
            mode='markers',
            marker=dict(size=15, color='gold', symbol='diamond'),
            name='Target'
        ))
        
        # Update layout (like test file)
        status = "SUCCESS ✅" if not final_collision_check.has_collision else "COLLISION ❌"
        collision_info = f"Final min distance: {final_collision_check.min_distance:.1f}mm"
        
        fig.update_layout(
            title=f'Complete Collision-Aware Pipeline - {status}<br>'
                  f'Steps: FABRIK → U Points → Swarm → Convert → Final FABRIK → MOTOR<br>'
                  f'{collision_info} | Total Time: {total_time:.1f}ms | Swarm: {step3_time:.1f}ms',
            scene=dict(
                xaxis_title='X (mm)',
                yaxis_title='Y (mm)',
                zaxis_title='Z (mm)',
                aspectmode='data',
                camera=dict(
                    eye=dict(x=1.2, y=1.2, z=1.2)
                )
            ),
            height=800,
            legend=dict(
                x=0.02,
                y=0.98,
                bgcolor='rgba(255,255,255,0.8)'
            )
        )
        
        fig.show()
        
        # Print detailed analysis with timing (enhanced)
        print(f"\nPipeline Analysis:")
        print(f"Initial FABRIK error: {initial_fabrik_error:.3f}mm")
        print(f"Final FABRIK error: {final_fabrik.final_error:.3f}mm")
        print(f"Final MOTOR error: {final_motor_result.fabrik_error:.3f}mm")
        print(f"Collision detected initially: {collision_result.has_collision}")
        print(f"Min distance to obstacles: {collision_result.min_distance:.2f}mm")
        print(f"Final solution collision-free: {not final_collision_check.has_collision}")
        print(f"Final min distance: {final_collision_check.min_distance:.2f}mm")
        print(f"Waypoint conversion successful: {conversion_result.conversion_successful}")
        if conversion_result.conversion_successful:
            print(f"Total reach after conversion: {conversion_result.total_reach:.1f}mm")
        
        # Detailed timing breakdown
        print(f"\n=== PERFORMANCE BREAKDOWN ===")
        print(f"Step 1 - Initial FABRIK: {step1_time:.2f}ms (FABRIK: {initial_solve_time:.2f}ms)")
        print(f"Step 2 - U Points extraction: {step2_time:.2f}ms")
        print(f"Step 3 - Collision detection + Swarm: {step3_time:.2f}ms")
        print(f"Step 4 - Waypoint conversion: {step4_time:.2f}ms")
        print(f"Step 5 - Final FABRIK: {step5_time:.2f}ms (FABRIK: {final_fabrik_solve_time:.2f}ms)")
        print(f"Step 6 - Final motor calculation: {step6_time:.2f}ms")
        print(f"Step 7 - Final collision verification: {step7_time:.2f}ms")
        print(f"TOTAL PIPELINE TIME: {total_time:.2f}ms")
        
        # Segment calculation timing
        if hasattr(final_motor_result, 'segment_calculation_time_ms'):
            print(f"Segment calculation (included): {final_motor_result.segment_calculation_time_ms:.2f}ms")
        
        return final_motor_result
        
    except Exception as e:
        print(f"Pipeline visualization failed: {e}")
        import traceback
        traceback.print_exc()
        return None

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

def print_motor_results(motor_result, title="MOTOR RESULTS"):
    """Print motor calculation results"""
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
            # Collision-aware approach with detailed visualization (like test file)
            motor_result = create_detailed_pipeline_visualization(
                target_x, target_y, target_z, current_positions, enable_obstacles=True)
            
            if motor_result:
                print_motor_results(motor_result, "FINAL COLLISION-AWARE MOTOR RESULTS")
        else:
            # Traditional approach with basic visualization
            motor_result = solve_without_collision_detection_visual(target_x, target_y, target_z, current_positions)
            print_motor_results(motor_result, "TRADITIONAL MOTOR RESULTS")
        
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