#!/usr/bin/env python3
"""
Test Complete Collision Pipeline - U Points → Collision Avoidance → Waypoint Conversion
WITH SPLINE CURVATURE ANALYSIS
"""
import sys
import os
import numpy as np
import plotly.graph_objects as go

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def calculate_curvature_radius(p1, p2, p3):
    """Calculate radius of curvature at point p2 using three consecutive points"""
    v1 = p1 - p2
    v2 = p3 - p2
    
    v1_norm = np.linalg.norm(v1)
    v2_norm = np.linalg.norm(v2)
    
    if v1_norm < 1e-6 or v2_norm < 1e-6:
        return float('inf')
    
    v1_unit = v1 / v1_norm
    v2_unit = v2 / v2_norm
    
    dot_product = np.clip(np.dot(v1_unit, v2_unit), -1.0, 1.0)
    angle = np.arccos(dot_product)
    
    if angle < 1e-6:
        return float('inf')
    
    chord_length = np.linalg.norm(p3 - p1)
    half_angle = angle / 2.0
    
    if half_angle < 1e-6:
        return float('inf')
    
    return chord_length / (2.0 * np.sin(half_angle))

def analyze_spline_curvature(spline_points, label, min_radius=75.0):
    """Analyze curvature along a spline and print results"""
    print(f"\n=== {label} CURVATURE ANALYSIS ===")
    
    if len(spline_points) < 3:
        print("Not enough points for curvature analysis")
        return [], []
    
    radii = []
    violations = []
    
    for i in range(1, len(spline_points) - 1):
        radius = calculate_curvature_radius(spline_points[i-1], spline_points[i], spline_points[i+1])
        radii.append(radius)
        
        if radius < min_radius:
            violations.append((i, radius))
    
    # Statistics
    finite_radii = [r for r in radii if r != float('inf')]
    if finite_radii:
        min_r = min(finite_radii)
        avg_r = sum(finite_radii) / len(finite_radii)
        print(f"Curvature points analyzed: {len(radii)}")
        print(f"Minimum radius: {min_r:.1f}mm")
        print(f"Average radius: {avg_r:.1f}mm")
        print(f"Violations (< {min_radius}mm): {len(violations)}")
        
        if violations:
            print("Violation details:")
            for idx, radius in violations[:5]:  # Show first 5 violations
                print(f"  Point {idx}: {radius:.1f}mm")
            if len(violations) > 5:
                print(f"  ... and {len(violations) - 5} more")
    else:
        print("All segments are straight lines (infinite radius)")
    
    return radii, violations

def create_spline_from_points(points, num_samples=100):
    """Create Catmull-Rom spline from control points (mimics C++ implementation)"""
    if len(points) < 2:
        return np.array([])
    
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
        
        # 2.5. Generate original spline for comparison
        print("\n=== ORIGINAL SPLINE GENERATION ===")
        original_spline = create_spline_from_points(u_points, 100)
        print(f"Generated original spline with {len(original_spline)} points")
        
        # 2.6. Analyze original spline curvature
        original_radii, original_violations = analyze_spline_curvature(original_spline, "ORIGINAL SPLINE")
        
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
        
        # 4.5. Generate optimized spline for comparison
        print("\n=== OPTIMIZED SPLINE GENERATION ===")
        waypoints = [np.array([w[0], w[1], w[2]]) for w in collision_result.waypoints]
        optimized_spline = create_spline_from_points(waypoints, 100)
        print(f"Generated optimized spline with {len(optimized_spline)} points")
        
        # 4.6. Analyze optimized spline curvature
        optimized_radii, optimized_violations = analyze_spline_curvature(optimized_spline, "OPTIMIZED SPLINE")
        
        # 5. Convert waypoints back to joint positions
        print("\n=== WAYPOINT CONVERSION ===")
        
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
        
        # 7. Curvature improvement analysis
        print(f"\n=== CURVATURE IMPROVEMENT ===")
        print(f"Original violations: {len(original_violations)}")
        print(f"Optimized violations: {len(optimized_violations)}")
        if len(original_violations) > len(optimized_violations):
            print(f"✓ Reduced violations by {len(original_violations) - len(optimized_violations)}")
        elif len(optimized_violations) > len(original_violations):
            print(f"⚠️  Increased violations by {len(optimized_violations) - len(original_violations)}")
        else:
            print("Same number of violations")
        
        # 8. Visualization
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
        
        # Original spline (dense)
        if len(original_spline) > 0:
            fig.add_trace(go.Scatter3d(
                x=original_spline[:, 0], y=original_spline[:, 1], z=original_spline[:, 2],
                mode='lines',
                line=dict(color='red', width=2, dash='solid'),
                name='Original Spline',
                opacity=0.7
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
        
        # Optimized spline (dense)
        if len(optimized_spline) > 0:
            spline_color = 'lime' if not collision_result.has_collision else 'orange'
            fig.add_trace(go.Scatter3d(
                x=optimized_spline[:, 0], y=optimized_spline[:, 1], z=optimized_spline[:, 2],
                mode='lines',
                line=dict(color=spline_color, width=3, dash='solid'),
                name='Optimized Spline',
                opacity=0.8
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
        curvature_status = f"Violations: {len(original_violations)}→{len(optimized_violations)}"
        
        fig.update_layout(
            title=f'Complete Collision Pipeline - {status} - {curvature_status} (Time: {collision_result.computation_time:.1f}ms)',
            scene=dict(
                xaxis_title='X (mm)',
                yaxis_title='Y (mm)',
                zaxis_title='Z (mm)',
                aspectmode='data'
            ),
            height=800
        )
        
        fig.show()
        
        # 9. Summary
        print(f"\n=== PIPELINE SUMMARY ===")
        print(f"Original FABRIK: {len(joint_positions)} joints → {len(u_points)} U points")
        print(f"Original spline: {len(original_violations)} curvature violations")
        print(f"Collision detection: {collision_result.computation_time:.2f}ms")
        print(f"Optimized spline: {len(optimized_violations)} curvature violations")
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