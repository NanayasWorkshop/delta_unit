#!/usr/bin/env python3
"""
Debug Solutions - Visualize all 5 solution candidates
"""
import sys
import os
import numpy as np
import plotly.graph_objects as go
from scipy.interpolate import CubicSpline

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def create_sphere_mesh(center, radius, num_points=20):
    """Create a 3D sphere mesh for obstacle visualization"""
    u = np.linspace(0, 2 * np.pi, num_points)
    v = np.linspace(0, np.pi, num_points)
    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
    return x, y, z

def create_smooth_spline(points, num_interpolation_points=50):
    """Create smooth cubic spline through the given points."""
    if len(points) < 2:
        return points, points
    
    points = np.array(points)
    
    # Parameter t for the spline (0 to 1)
    t = np.linspace(0, 1, len(points))
    t_smooth = np.linspace(0, 1, num_interpolation_points)
    
    # Create cubic splines for each dimension
    try:
        spline_x = CubicSpline(t, points[:, 0], bc_type='natural')
        spline_y = CubicSpline(t, points[:, 1], bc_type='natural') 
        spline_z = CubicSpline(t, points[:, 2], bc_type='natural')
        
        # Evaluate splines at smooth parameter values
        smooth_points = np.column_stack([
            spline_x(t_smooth),
            spline_y(t_smooth), 
            spline_z(t_smooth)
        ])
        
        return points, smooth_points
        
    except Exception as e:
        print(f"Warning: Spline interpolation failed: {e}")
        return points, points

def analyze_bending_directions():
    """Analyze what bending directions are being generated"""
    
    try:
        import delta_robot
        
        print("Analyzing Collision Avoidance Algorithm...")
        print("=" * 50)
        
        # Test parameters
        target = np.array([100, 50, 300])
        obstacle_pos = np.array([53, 28, 500])  # Put obstacle in middle of path
        obstacle_radius = 60
        
        print(f"Target: {target}")
        print(f"Obstacle: {obstacle_pos}, radius={obstacle_radius}")
        
        # Get normal FABRIK result
        normal_result = delta_robot.solve_fabrik_with_spline(target[0], target[1], target[2], 7)
        original_spline = np.array(normal_result.spline_points)
        
        print(f"\nOriginal spline points:")
        for i, point in enumerate(original_spline):
            print(f"  {i}: ({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f})")
        
        # Create obstacle
        obstacle = delta_robot.create_sphere_obstacle(obstacle_pos[0], obstacle_pos[1], obstacle_pos[2], obstacle_radius)
        obstacles = [obstacle]
        
        # Check collision
        has_collision = delta_robot.check_spline_collision(original_spline, obstacles, delta_robot.SPLINE_THICKNESS)
        print(f"\nHas collision: {has_collision}")
        
        if not has_collision:
            print("No collision detected - adjusting obstacle position...")
            # Find closest point on spline to put obstacle there
            closest_point = delta_robot.collision.SplineCollisionAvoidance.find_closest_point_on_spline(
                obstacle_pos, original_spline.tolist())
            print(f"Closest point on spline: ({closest_point[0]:.2f}, {closest_point[1]:.2f}, {closest_point[2]:.2f})")
            
            # Put obstacle directly on spline
            obstacle_pos = np.array([closest_point[0], closest_point[1], closest_point[2]])
            obstacle = delta_robot.create_sphere_obstacle(obstacle_pos[0], obstacle_pos[1], obstacle_pos[2], obstacle_radius)
            obstacles = [obstacle]
            
            has_collision = delta_robot.check_spline_collision(original_spline, obstacles, delta_robot.SPLINE_THICKNESS)
            print(f"Collision with adjusted obstacle: {has_collision}")
        
        if not has_collision:
            print("Still no collision - algorithm might not be working as expected")
            return
        
        # Try collision avoidance
        collision_result = delta_robot.collision.SplineCollisionAvoidance.avoid_collisions(
            original_spline, obstacles)
        
        print(f"\nCollision Avoidance Results:")
        print(f"  Collision free: {collision_result.collision_free}")
        print(f"  Solutions evaluated: {collision_result.solutions_evaluated}")
        print(f"  Total adjustment: {collision_result.total_adjustment:.6f} mm")
        print(f"  Execution time: {collision_result.execution_time_ms:.3f} ms")
        
        # Analyze the safe spline
        safe_spline = np.array(collision_result.safe_control_points)
        
        print(f"\nSafe spline points:")
        for i, point in enumerate(safe_spline):
            print(f"  {i}: ({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f})")
        
        print(f"\nPoint-by-point differences:")
        max_diff = 0.0
        for i, (orig, safe) in enumerate(zip(original_spline, safe_spline)):
            diff = np.linalg.norm(safe - orig)
            max_diff = max(max_diff, diff)
            print(f"  Point {i}: {diff:.6f} mm")
        
        print(f"\nMaximum point movement: {max_diff:.6f} mm")
        
        # Check if safe spline still has collision
        safe_has_collision = delta_robot.check_spline_collision(safe_spline, obstacles, delta_robot.SPLINE_THICKNESS)
        print(f"Safe spline still has collision: {safe_has_collision}")
        
        # Try manual bending test
        print(f"\nTesting manual displacement...")
        manual_spline = original_spline.copy()
        
        # Move middle points away from obstacle by 100mm in X direction
        for i in range(2, 6):  # Move middle points
            manual_spline[i][0] += 100  # Move 100mm in X direction
        
        manual_has_collision = delta_robot.check_spline_collision(manual_spline, obstacles, delta_robot.SPLINE_THICKNESS)
        print(f"Manual 100mm X displacement collision: {manual_has_collision}")
        
        # Visualize everything
        visualize_analysis(original_spline, safe_spline, manual_spline, target, obstacle_pos, obstacle_radius, 
                          has_collision, safe_has_collision, manual_has_collision)
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

def visualize_analysis(original_spline, safe_spline, manual_spline, target, obstacle_pos, obstacle_radius,
                      has_collision, safe_has_collision, manual_has_collision):
    """Visualize the analysis results"""
    
    print("\nCreating analysis visualization...")
    
    # Create smooth splines
    orig_control, orig_smooth = create_smooth_spline(original_spline, 100)
    safe_control, safe_smooth = create_smooth_spline(safe_spline, 100)
    manual_control, manual_smooth = create_smooth_spline(manual_spline, 100)
    
    # Calculate ranges
    all_points = np.vstack([original_spline, safe_spline, manual_spline, [target], [obstacle_pos]])
    
    x_range_size = np.max(all_points[:, 0]) - np.min(all_points[:, 0])
    y_range_size = np.max(all_points[:, 1]) - np.min(all_points[:, 1])
    z_range_size = np.max(all_points[:, 2]) - np.min(all_points[:, 2])
    max_range = max(x_range_size, y_range_size, z_range_size)
    
    margin = 0.2 * max_range
    max_range_with_margin = max_range + 2 * margin
    
    x_center = (np.max(all_points[:, 0]) + np.min(all_points[:, 0])) / 2
    y_center = (np.max(all_points[:, 1]) + np.min(all_points[:, 1])) / 2
    z_center = (np.max(all_points[:, 2]) + np.min(all_points[:, 2])) / 2
    
    half_range = max_range_with_margin / 2
    x_range = [x_center - half_range, x_center + half_range]
    y_range = [y_center - half_range, y_center + half_range]
    z_range = [z_center - half_range, z_center + half_range]
    
    # Create plot
    fig = go.Figure()
    
    # Obstacle
    x_sphere, y_sphere, z_sphere = create_sphere_mesh(obstacle_pos, obstacle_radius, 15)
    fig.add_trace(go.Surface(
        x=x_sphere, y=y_sphere, z=z_sphere,
        colorscale=[[0, 'red'], [1, 'red']],
        opacity=0.7,
        showscale=False,
        name='Obstacle',
        hovertemplate=f'Obstacle<br>Center: ({obstacle_pos[0]:.1f}, {obstacle_pos[1]:.1f}, {obstacle_pos[2]:.1f})<br>Radius: {obstacle_radius}mm<extra></extra>'
    ))
    
    # Original spline
    collision_color = 'red' if has_collision else 'green'
    fig.add_trace(go.Scatter3d(
        x=orig_smooth[:, 0], y=orig_smooth[:, 1], z=orig_smooth[:, 2],
        mode='lines',
        line=dict(color=collision_color, width=8),
        name=f'Original Spline (collision: {has_collision})',
        hovertemplate='Original<br>(%{x:.2f}, %{y:.2f}, %{z:.2f})<extra></extra>'
    ))
    
    # Safe spline (from collision avoidance)
    safe_collision_color = 'red' if safe_has_collision else 'lime'
    fig.add_trace(go.Scatter3d(
        x=safe_smooth[:, 0], y=safe_smooth[:, 1], z=safe_smooth[:, 2],
        mode='lines',
        line=dict(color=safe_collision_color, width=10, dash='dash'),
        name=f'CA Safe Spline (collision: {safe_has_collision})',
        hovertemplate='CA Safe<br>(%{x:.2f}, %{y:.2f}, %{z:.2f})<extra></extra>'
    ))
    
    # Manual displacement spline
    manual_collision_color = 'red' if manual_has_collision else 'cyan'
    fig.add_trace(go.Scatter3d(
        x=manual_smooth[:, 0], y=manual_smooth[:, 1], z=manual_smooth[:, 2],
        mode='lines',
        line=dict(color=manual_collision_color, width=6, dash='dot'),
        name=f'Manual +100mm X (collision: {manual_has_collision})',
        hovertemplate='Manual<br>(%{x:.2f}, %{y:.2f}, %{z:.2f})<extra></extra>'
    ))
    
    # Control points
    fig.add_trace(go.Scatter3d(
        x=orig_control[:, 0], y=orig_control[:, 1], z=orig_control[:, 2],
        mode='markers',
        marker=dict(size=6, color='orange', symbol='circle'),
        name='Original Control Points',
        hovertemplate='Control %{pointNumber}<br>(%{x:.2f}, %{y:.2f}, %{z:.2f})<extra></extra>'
    ))
    
    # Target and base
    fig.add_trace(go.Scatter3d(
        x=[target[0]], y=[target[1]], z=[target[2]],
        mode='markers',
        marker=dict(size=12, color='gold', symbol='diamond'),
        name='Target',
        hovertemplate='Target<br>(%{x:.2f}, %{y:.2f}, %{z:.2f})<extra></extra>'
    ))
    
    fig.add_trace(go.Scatter3d(
        x=[0], y=[0], z=[0],
        mode='markers',
        marker=dict(size=8, color='black', symbol='square'),
        name='Base',
        hovertemplate='Base<br>(0, 0, 0)<extra></extra>'
    ))
    
    # Layout
    fig.update_layout(
        title=dict(
            text='Collision Avoidance Algorithm Analysis<br>' +
                 f'Original: {collision_color} | CA Safe: {safe_collision_color} | Manual: {manual_collision_color}',
            x=0.5,
            font=dict(size=16)
        ),
        scene=dict(
            xaxis=dict(title='X (mm)', range=x_range),
            yaxis=dict(title='Y (mm)', range=y_range),
            zaxis=dict(title='Z (mm)', range=z_range),
            camera=dict(eye=dict(x=1.5, y=1.5, z=1.2)),
            aspectmode='cube'
        ),
        width=1400,
        height=900,
        legend=dict(yanchor="top", y=0.99, xanchor="left", x=0.01)
    )
    
    fig.show()
    print("Analysis visualization complete!")

if __name__ == "__main__":
    analyze_bending_directions()