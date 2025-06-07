#!/usr/bin/env python3
"""
Collision Avoidance Test and Visualization
Interactive 3D visualization showing FABRIK solving with collision avoidance
"""
import sys
import os
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from scipy.interpolate import CubicSpline

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

def create_sphere_mesh(center, radius, num_points=20):
    """Create a 3D sphere mesh for obstacle visualization"""
    u = np.linspace(0, 2 * np.pi, num_points)
    v = np.linspace(0, np.pi, num_points)
    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
    return x, y, z

def create_smooth_spline(points, num_interpolation_points=100):
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

def test_collision_avoidance_scenario(target_x, target_y, target_z, obstacles, num_segments=7):
    """Test collision avoidance for a specific scenario"""
    
    try:
        import delta_robot
        
        print(f"\n=== Testing Collision Avoidance ===")
        print(f"Target: ({target_x}, {target_y}, {target_z})")
        print(f"Obstacles: {len(obstacles)} spheres")
        
        # Create obstacle objects
        obstacle_objects = []
        for i, (x, y, z, radius) in enumerate(obstacles):
            obstacle_objects.append(delta_robot.create_sphere_obstacle(x, y, z, radius))
            print(f"  Obstacle {i+1}: Center=({x}, {y}, {z}), Radius={radius}mm")
        
        target = np.array([target_x, target_y, target_z])
        
        # Solve without collision avoidance (original)
        print("\n1. Solving without collision avoidance...")
        original_result = delta_robot.solve_fabrik_with_spline(target_x, target_y, target_z, num_segments)
        print(f"   FABRIK converged: {original_result.converged}")
        print(f"   Error: {original_result.final_error:.4f} mm")
        
        # Check if original path has collisions
        has_collision = delta_robot.check_spline_collision(
            original_result.spline_points, obstacle_objects, delta_robot.SPLINE_THICKNESS)
        print(f"   Original path has collisions: {has_collision}")
        
        # Solve with collision avoidance
        print("\n2. Solving with collision avoidance...")
        safe_result = delta_robot.solve_delta_robot_safe(num_segments, target, obstacle_objects)
        print(f"   FABRIK converged: {safe_result.converged}")
        print(f"   Error: {safe_result.final_error:.4f} mm")
        print(f"   Collision avoidance applied: {safe_result.collision_avoidance_applied}")
        
        if safe_result.collision_avoidance_applied:
            collision_info = safe_result.collision_result
            print(f"   Collision avoidance time: {collision_info.execution_time_ms:.3f} ms")
            print(f"   Solutions evaluated: {collision_info.solutions_evaluated}")
            print(f"   Total adjustment: {collision_info.total_adjustment:.2f} mm")
            
            # Verify safe path is collision-free
            safe_has_collision = delta_robot.check_spline_collision(
                safe_result.safe_spline_points, obstacle_objects, delta_robot.SPLINE_THICKNESS)
            print(f"   Safe path has collisions: {safe_has_collision}")
        
        return original_result, safe_result, obstacle_objects
        
    except ImportError as e:
        print(f"ImportError: {e}")
        print("Please ensure the compiled C++ module is in the Python path.")
        return None, None, None
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()
        return None, None, None

def visualize_collision_avoidance(original_result, safe_result, obstacle_objects, target):
    """Create interactive 3D visualization comparing original and safe paths"""
    
    if original_result is None or safe_result is None:
        print("Cannot visualize: Invalid results")
        return
    
    print(f"\n=== Creating Visualization ===")
    
    # Extract data
    original_spline = np.array(original_result.spline_points)
    joint_positions = np.array([np.array(joint.position) for joint in original_result.final_chain.joints])
    
    # Use safe spline if collision avoidance was applied, otherwise original
    if safe_result.collision_avoidance_applied:
        safe_spline = np.array(safe_result.safe_spline_points)
        print("Using collision-free spline path")
    else:
        safe_spline = original_spline
        print("No collision avoidance needed")
    
    # Create smooth splines
    original_control, original_smooth = create_smooth_spline(original_spline, 150)
    safe_control, safe_smooth = create_smooth_spline(safe_spline, 150)
    
    # Calculate axis ranges with equal scaling
    all_points = np.vstack([joint_positions, original_spline, safe_spline, [target]])
    
    # Add obstacle centers to range calculation
    for obs in obstacle_objects:
        obs_center = np.array([obs.center[0], obs.center[1], obs.center[2]])
        all_points = np.vstack([all_points, [obs_center]])
    
    # Find the maximum range across all dimensions
    x_range_size = np.max(all_points[:, 0]) - np.min(all_points[:, 0])
    y_range_size = np.max(all_points[:, 1]) - np.min(all_points[:, 1])
    z_range_size = np.max(all_points[:, 2]) - np.min(all_points[:, 2])
    max_range = max(x_range_size, y_range_size, z_range_size)
    
    # Add margin
    margin = 0.15 * max_range
    max_range_with_margin = max_range + 2 * margin
    
    # Calculate centers
    x_center = (np.max(all_points[:, 0]) + np.min(all_points[:, 0])) / 2
    y_center = (np.max(all_points[:, 1]) + np.min(all_points[:, 1])) / 2
    z_center = (np.max(all_points[:, 2]) + np.min(all_points[:, 2])) / 2
    
    # Set equal ranges around centers
    half_range = max_range_with_margin / 2
    x_range = [x_center - half_range, x_center + half_range]
    y_range = [y_center - half_range, y_center + half_range]
    z_range = [z_center - half_range, z_center + half_range]
    
    # Create 3D plot
    fig = go.Figure()
    
    # 1. Obstacles (add first so they appear behind)
    for i, obs in enumerate(obstacle_objects):
        x_sphere, y_sphere, z_sphere = create_sphere_mesh(
            [obs.center[0], obs.center[1], obs.center[2]], obs.radius, 15)
        
        fig.add_trace(go.Surface(
            x=x_sphere, y=y_sphere, z=z_sphere,
            colorscale=[[0, 'red'], [1, 'red']],
            opacity=0.6,
            showscale=False,
            name=f'Obstacle {i+1}',
            hovertemplate=f'Obstacle {i+1}<br>Center: ({obs.center[0]:.1f}, {obs.center[1]:.1f}, {obs.center[2]:.1f})<br>Radius: {obs.radius:.1f}mm<extra></extra>'
        ))
    
    # 2. FABRIK joint chain
    fig.add_trace(go.Scatter3d(
        x=joint_positions[:, 0],
        y=joint_positions[:, 1], 
        z=joint_positions[:, 2],
        mode='lines+markers',
        line=dict(color='blue', width=6),
        marker=dict(size=5, color='darkblue'),
        name='FABRIK Joint Chain',
        hovertemplate='Joint %{pointNumber}<br>Position: (%{x:.2f}, %{y:.2f}, %{z:.2f})<extra></extra>'
    ))
    
    # 3. Original spline path (potentially with collisions)
    spline_color = 'orange' if safe_result.collision_avoidance_applied else 'lime'
    spline_name = 'Original Spline (with collisions)' if safe_result.collision_avoidance_applied else 'Spline Path (collision-free)'
    
    fig.add_trace(go.Scatter3d(
        x=original_smooth[:, 0],
        y=original_smooth[:, 1],
        z=original_smooth[:, 2],
        mode='lines',
        line=dict(color=spline_color, width=8, dash='solid'),
        name=spline_name,
        hovertemplate='Original Spline<br>Position: (%{x:.2f}, %{y:.2f}, %{z:.2f})<extra></extra>'
    ))
    
    # 4. Safe spline path (if different from original)
    if safe_result.collision_avoidance_applied:
        fig.add_trace(go.Scatter3d(
            x=safe_smooth[:, 0],
            y=safe_smooth[:, 1],
            z=safe_smooth[:, 2],
            mode='lines',
            line=dict(color='lime', width=10, dash='solid'),
            name='Safe Spline (collision-free)',
            hovertemplate='Safe Spline<br>Position: (%{x:.2f}, %{y:.2f}, %{z:.2f})<extra></extra>'
        ))
        
        # 5. Original spline control points
        fig.add_trace(go.Scatter3d(
            x=original_control[:, 0],
            y=original_control[:, 1],
            z=original_control[:, 2],
            mode='markers',
            marker=dict(size=6, color='orange', symbol='circle'),
            name='Original Control Points',
            hovertemplate='Original Control %{pointNumber}<br>Position: (%{x:.2f}, %{y:.2f}, %{z:.2f})<extra></extra>'
        ))
        
        # 6. Safe spline control points
        fig.add_trace(go.Scatter3d(
            x=safe_control[:, 0],
            y=safe_control[:, 1],
            z=safe_control[:, 2],
            mode='markers',
            marker=dict(size=7, color='green', symbol='diamond'),
            name='Safe Control Points',
            hovertemplate='Safe Control %{pointNumber}<br>Position: (%{x:.2f}, %{y:.2f}, %{z:.2f})<extra></extra>'
        ))
    else:
        # 5. Spline control points (no collision avoidance needed)
        fig.add_trace(go.Scatter3d(
            x=original_control[:, 0],
            y=original_control[:, 1],
            z=original_control[:, 2],
            mode='markers',
            marker=dict(size=7, color='green', symbol='circle'),
            name='Spline Control Points',
            hovertemplate='Control Point %{pointNumber}<br>Position: (%{x:.2f}, %{y:.2f}, %{z:.2f})<extra></extra>'
        ))
    
    # 7. Target position
    fig.add_trace(go.Scatter3d(
        x=[target[0]],
        y=[target[1]],
        z=[target[2]],
        mode='markers',
        marker=dict(size=8, color='gold', symbol='star'),
        name='Target Position',
        hovertemplate='Target<br>Position: (%{x:.2f}, %{y:.2f}, %{z:.2f})<extra></extra>'
    ))
    
    # 8. Base position (origin)
    fig.add_trace(go.Scatter3d(
        x=[0],
        y=[0], 
        z=[0],
        mode='markers',
        marker=dict(size=6, color='black', symbol='square'),
        name='Base (Origin)',
        hovertemplate='Base<br>Position: (0.00, 0.00, 0.00)<extra></extra>'
    ))
    
    # Create title with collision avoidance info
    title_text = f'FABRIK with Collision Avoidance<br>'
    title_text += f'Target: ({target[0]}, {target[1]}, {target[2]}) | '
    title_text += f'Error: {safe_result.final_error:.4f}mm'
    
    if safe_result.collision_avoidance_applied:
        collision_info = safe_result.collision_result
        title_text += f'<br>Collision Avoidance: {collision_info.execution_time_ms:.3f}ms | '
        title_text += f'Adjustment: {collision_info.total_adjustment:.2f}mm'
    
    # Update layout
    fig.update_layout(
        title=dict(
            text=title_text,
            x=0.5,
            font=dict(size=16)
        ),
        scene=dict(
            xaxis=dict(title='X (mm)', range=x_range),
            yaxis=dict(title='Y (mm)', range=y_range),
            zaxis=dict(title='Z (mm)', range=z_range),
            camera=dict(
                eye=dict(x=1.5, y=1.5, z=1.2),
                center=dict(x=0, y=0, z=0)
            ),
            aspectmode='cube'
        ),
        width=1400,
        height=900,
        legend=dict(
            yanchor="top",
            y=0.99,
            xanchor="left", 
            x=0.01
        )
    )
    
    # Show interactive plot
    fig.show()
    
    print(f"✓ Visualization complete!")
    print(f"✓ Interactive 3D plot opened in browser")

def run_collision_avoidance_tests():
    """Run a series of collision avoidance test scenarios"""
    
    print("🚀 FABRIK Collision Avoidance Test Suite")
    print("=" * 50)
    
    # Test scenarios
    scenarios = [
        {
            "name": "Simple Single Obstacle",
            "target": (100, 50, 300),
            "obstacles": [(50, 25, 200, 40)],  # (x, y, z, radius)
            "description": "Single obstacle in the path"
        },
        {
            "name": "Multiple Obstacles",
            "target": (80, 80, 350),
            "obstacles": [(40, 20, 180, 25), (20, 60, 250, 30), (60, 40, 320, 20)],
            "description": "Multiple obstacles requiring complex avoidance"
        },
        {
            "name": "Narrow Passage",
            "target": (0, 0, 400),
            "obstacles": [(-30, 0, 200, 25), (30, 0, 200, 25)],
            "description": "Narrow passage between two obstacles"
        },
        {
            "name": "No Collision (Control)",
            "target": (150, 0, 250),
            "obstacles": [(0, 150, 200, 30)],
            "description": "Obstacle not in path (should not trigger avoidance)"
        }
    ]
    
    print(f"Available test scenarios:")
    for i, scenario in enumerate(scenarios):
        print(f"  {i+1}. {scenario['name']}: {scenario['description']}")
    
    print(f"\nRunning all scenarios automatically...")
    
    for i, scenario in enumerate(scenarios):
        print(f"\n{'='*60}")
        print(f"SCENARIO {i+1}: {scenario['name']}")
        print(f"Description: {scenario['description']}")
        print(f"{'='*60}")
        
        target_x, target_y, target_z = scenario['target']
        obstacles = scenario['obstacles']
        
        # Test the scenario
        original_result, safe_result, obstacle_objects = test_collision_avoidance_scenario(
            target_x, target_y, target_z, obstacles)
        
        if original_result and safe_result:
            # Visualize the result
            visualize_collision_avoidance(original_result, safe_result, obstacle_objects, 
                                         np.array([target_x, target_y, target_z]))
            
            input(f"\nPress Enter to continue to next scenario...")

def main():
    """Main function with command line argument parsing"""
    
    if len(sys.argv) >= 2 and sys.argv[1] == "test":
        # Run test suite
        run_collision_avoidance_tests()
        return
    
    # Parse command line arguments for custom scenario
    if len(sys.argv) < 2:
        target_x, target_y, target_z = 100, 50, 300
        obstacles = [(50, 25, 200, 40)]  # Default: single obstacle
        print(f"Usage: python3 {os.path.basename(__file__)} x,y,z [obstacle1_x,y,z,radius] [obstacle2_x,y,z,radius] ...")
        print(f"       python3 {os.path.basename(__file__)} test  # Run test suite")
        print(f"Using default: target=({target_x}, {target_y}, {target_z}), obstacle at (50,25,200) r=40")
    else:
        target_x, target_y, target_z = parse_coordinates(sys.argv[1])
        obstacles = []
        
        # Parse obstacle arguments
        for i in range(2, len(sys.argv)):
            try:
                obs_coords = [float(x.strip()) for x in sys.argv[i].split(',')]
                if len(obs_coords) != 4:
                    print(f"Warning: Obstacle {i-1} needs 4 values (x,y,z,radius), got {len(obs_coords)}")
                    continue
                obstacles.append(tuple(obs_coords))
            except ValueError as e:
                print(f"Warning: Could not parse obstacle {i-1}: {e}")
        
        if not obstacles:
            obstacles = [(50, 25, 200, 40)]  # Default obstacle
            print("No valid obstacles provided, using default obstacle at (50,25,200) r=40")
    
    print("Starting FABRIK Collision Avoidance Test...")
    print("Required packages: plotly, scipy, numpy")
    
    try:
        import plotly
        import scipy
        print("✓ All visualization packages available")
    except ImportError as e:
        print(f"✗ Missing package: {e}")
        print("Install with: pip install plotly scipy")
        sys.exit(1)
    
    # Test the scenario
    original_result, safe_result, obstacle_objects = test_collision_avoidance_scenario(
        target_x, target_y, target_z, obstacles)
    
    if original_result and safe_result:
        # Visualize the result
        visualize_collision_avoidance(original_result, safe_result, obstacle_objects, 
                                     np.array([target_x, target_y, target_z]))
        print("\n✓ Test complete! Check your browser for the interactive 3D visualization.")
    else:
        print("\n✗ Test failed. Check error messages above.")

if __name__ == "__main__":
    main()