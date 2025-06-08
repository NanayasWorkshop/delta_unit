#!/usr/bin/env python3
"""
Test Complete Collision Pipeline - U Points → Collision Avoidance → Waypoint Conversion
"""
import sys
import os
import numpy as np
import plotly.graph_objects as go

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def test_complete_collision_pipeline():
    try:
        import delta_robot
        
        target = np.array([100, 0, 300])  # Use target that causes collision
        print(f"Target: {target}")
        
        # 1. FABRIK solver first
        print("\n=== FABRIK SOLVER ===")
        fabrik_result = delta_robot.fabrik.solve_delta_robot(7, target, delta_robot.FABRIK_TOLERANCE)
        print(f"FABRIK converged: {fabrik_result.converged}")
        
        # 2. Extract joint positions and U points
        print("\n=== U POINTS EXTRACTION ===")
        joint_positions = [np.array([j.position[0], j.position[1], j.position[2]]) for j in fabrik_result.final_chain.joints]
        u_points_raw = delta_robot.delta_robot_complete.UPointsExtractor.extract_u_points_from_positions(joint_positions)
        u_points = [np.array([u[0], u[1], u[2]]) for u in u_points_raw]
        print(f"Extracted {len(u_points)} U points")
        
        # 3. Create obstacles
        print("\n=== OBSTACLES ===")
        obstacles = delta_robot.delta_robot_complete.CollisionDetector.create_test_obstacles()
        print(f"Created {len(obstacles)} obstacles")
        
        # 4. Run collision detection
        print("\n=== COLLISION DETECTION (C++ output) ===")
        collision_result = delta_robot.delta_robot_complete.CollisionDetector.check_and_avoid(
            u_points_raw, obstacles, delta_robot.DEFAULT_SPLINE_DIAMETER
        )
        
        print(f"\n=== COLLISION RESULTS ===")
        print(f"Collision detected: {collision_result.has_collision}")
        print(f"Min distance: {collision_result.min_distance:.2f}")
        print(f"Computation time: {collision_result.computation_time:.2f}ms")
        print(f"Got {len(collision_result.waypoints)} waypoints back")
        
        # 5. Convert waypoints back to joint positions
        print("\n=== WAYPOINT CONVERSION ===")
        waypoints = [np.array([w[0], w[1], w[2]]) for w in collision_result.waypoints]
        
        conversion_result = delta_robot.delta_robot_complete.WaypointConverter.convert_waypoints_to_joints(
            collision_result.waypoints
        )
        
        print(f"Conversion successful: {conversion_result.conversion_successful}")
        if conversion_result.conversion_successful:
            converted_joints = [np.array([j[0], j[1], j[2]]) for j in conversion_result.joint_positions]
            print(f"Converted to {len(converted_joints)} joint positions")
            print(f"Total reach: {conversion_result.total_reach:.1f}mm")
            print(f"Calculated {len(conversion_result.joint_angles_deg)} joint angles")
        
        # 6. Movement analysis
        print(f"\n=== MOVEMENT ANALYSIS ===")
        total_movement = 0
        max_movement = 0
        for i, (u_point, waypoint) in enumerate(zip(u_points, waypoints)):
            diff = np.linalg.norm(waypoint - u_point)
            total_movement += diff
            max_movement = max(max_movement, diff)
            if diff > 5.0:
                print(f"U{i+1}: moved {diff:.1f}mm")
        
        avg_movement = total_movement / len(u_points)
        print(f"Average movement: {avg_movement:.1f}mm, Max movement: {max_movement:.1f}mm")
        
        # 7. Visualization
        print("\n=== VISUALIZATION ===")
        fig = go.Figure()
        
        # Plot obstacles
        for i, obs in enumerate(obstacles):
            center = obs.center
            radius = obs.radius
            
            u = np.linspace(0, 2 * np.pi, 20)
            v = np.linspace(0, np.pi, 20)
            x = radius * np.outer(np.cos(u), np.sin(v)) + center[0]
            y = radius * np.outer(np.sin(u), np.sin(v)) + center[1]
            z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + center[2]
            
            fig.add_trace(go.Surface(x=x, y=y, z=z, opacity=0.6, 
                                   colorscale='Reds', showscale=False, name=f'Obstacle {i+1}'))
        
        # Original FABRIK joint positions
        fabrik_x = [j[0] for j in joint_positions]
        fabrik_y = [j[1] for j in joint_positions]
        fabrik_z = [j[2] for j in joint_positions]
        
        fig.add_trace(go.Scatter3d(
            x=fabrik_x, y=fabrik_y, z=fabrik_z,
            mode='markers+lines',
            marker=dict(size=5, color='blue'),
            line=dict(color='blue', width=3, dash='dot'),
            name='Original FABRIK Joints'
        ))
        
        # Original U points
        u_x = [p[0] for p in u_points]
        u_y = [p[1] for p in u_points]
        u_z = [p[2] for p in u_points]
        
        fig.add_trace(go.Scatter3d(
            x=u_x, y=u_y, z=u_z,
            mode='markers+lines',
            marker=dict(size=6, color='red'),
            line=dict(color='red', width=4, dash='dash'),
            name='Original U Points'
        ))
        
        # Collision-free waypoints
        if waypoints:
            wp_x = [p[0] for p in waypoints]
            wp_y = [p[1] for p in waypoints]
            wp_z = [p[2] for p in waypoints]
            
            color = 'lime' if not collision_result.has_collision else 'orange'
            name = 'Collision-Free Waypoints' if not collision_result.has_collision else 'Optimized Waypoints'
            
            fig.add_trace(go.Scatter3d(
                x=wp_x, y=wp_y, z=wp_z,
                mode='markers+lines',
                marker=dict(size=8, color=color),
                line=dict(color=color, width=6),
                name=name
            ))
        
        # Converted joint positions
        if conversion_result.conversion_successful:
            conv_x = [j[0] for j in converted_joints]
            conv_y = [j[1] for j in converted_joints]
            conv_z = [j[2] for j in converted_joints]
            
            fig.add_trace(go.Scatter3d(
                x=conv_x, y=conv_y, z=conv_z,
                mode='markers+lines',
                marker=dict(size=7, color='purple'),
                line=dict(color='purple', width=5, dash='solid'),
                name='Converted FABRIK Joints'
            ))
        
        # Target point
        fig.add_trace(go.Scatter3d(
            x=[target[0]], y=[target[1]], z=[target[2]],
            mode='markers',
            marker=dict(size=12, color='green'),
            name='Target'
        ))
        
        status = "COLLISION AVOIDED" if not collision_result.has_collision else "COLLISION DETECTED"
        
        fig.update_layout(
            title=f'Complete Collision Pipeline Test - {status} (Time: {collision_result.computation_time:.1f}ms)',
            scene=dict(
                xaxis_title='X (mm)',
                yaxis_title='Y (mm)',
                zaxis_title='Z (mm)',
                aspectmode='data'
            ),
            height=800
        )
        
        fig.show()
        
        # 8. Summary
        print(f"\n=== PIPELINE SUMMARY ===")
        print(f"Original FABRIK: {len(joint_positions)} joints → {len(u_points)} U points")
        print(f"Collision detection: {collision_result.computation_time:.2f}ms")
        print(f"Waypoint conversion: {'SUCCESS' if conversion_result.conversion_successful else 'FAILED'}")
        if conversion_result.conversion_successful:
            print(f"Final result: {len(converted_joints)} joints with {conversion_result.total_reach:.1f}mm reach")
        
        return True
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    test_complete_collision_pipeline()