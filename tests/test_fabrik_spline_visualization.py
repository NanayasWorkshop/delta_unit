#!/usr/bin/env python3
"""
FABRIK Spline Visualization Test with Plotly
Interactive 3D visualization of FABRIK solution with spline interpolation
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

def create_spindle_geometry(spline_points, center_radius=300, end_radius=32, num_circumference=36, num_length=120):
    """
    Create a capsule/pill-shaped 3D geometry around the spline.
    Thick in center, substantial rounded ends (not pointed).
    """
    if len(spline_points) < 2:
        return None, None, None, None, None, None
    
    spline_points = np.array(spline_points)
    
    # Create smooth parameter along spline (0 to 1)
    t_spline = np.linspace(0, 1, len(spline_points))
    t_smooth = np.linspace(0, 1, num_length)
    
    # Interpolate smooth spline path
    smooth_spline_x = np.interp(t_smooth, t_spline, spline_points[:, 0])
    smooth_spline_y = np.interp(t_smooth, t_spline, spline_points[:, 1])
    smooth_spline_z = np.interp(t_smooth, t_spline, spline_points[:, 2])
    
    smooth_spline = np.column_stack([smooth_spline_x, smooth_spline_y, smooth_spline_z])
    
    # Calculate radius function: capsule/pill shape with substantial ends
    # end_radius at start/end, center_radius at middle
    radius_function = end_radius + (center_radius - end_radius) * np.sin(np.pi * t_smooth)
    
    # Calculate tangent vectors along the spline
    tangents = np.zeros_like(smooth_spline)
    for i in range(len(smooth_spline)):
        if i == 0:
            tangents[i] = smooth_spline[i+1] - smooth_spline[i]
        elif i == len(smooth_spline) - 1:
            tangents[i] = smooth_spline[i] - smooth_spline[i-1]
        else:
            tangents[i] = smooth_spline[i+1] - smooth_spline[i-1]
    
    # Normalize tangents
    tangent_norms = np.linalg.norm(tangents, axis=1)
    tangent_norms[tangent_norms == 0] = 1  # Avoid division by zero
    tangents = tangents / tangent_norms[:, np.newaxis]
    
    # Create consistent perpendicular frame using Frenet-Serret approach
    # This prevents twisting by maintaining a consistent orientation
    perp_vectors = np.zeros_like(smooth_spline)
    binormal_vectors = np.zeros_like(smooth_spline)
    
    # Initialize first perpendicular vector
    if abs(tangents[0][2]) < 0.9:
        initial_perp = np.array([0, 0, 1])
    else:
        initial_perp = np.array([1, 0, 0])
    
    # Remove component parallel to tangent
    perp_vectors[0] = initial_perp - np.dot(initial_perp, tangents[0]) * tangents[0]
    perp_vectors[0] = perp_vectors[0] / np.linalg.norm(perp_vectors[0])
    binormal_vectors[0] = np.cross(tangents[0], perp_vectors[0])
    
    # Propagate frame along the curve to avoid twisting
    for i in range(1, len(smooth_spline)):
        # Project previous perpendicular onto plane perpendicular to current tangent
        perp_proj = perp_vectors[i-1] - np.dot(perp_vectors[i-1], tangents[i]) * tangents[i]
        
        # Check if projection is too small (parallel case)
        if np.linalg.norm(perp_proj) < 0.1:
            # Use cross product with a different vector
            if abs(tangents[i][2]) < 0.9:
                temp_vec = np.array([0, 0, 1])
            else:
                temp_vec = np.array([1, 0, 0])
            perp_proj = np.cross(tangents[i], temp_vec)
        
        perp_vectors[i] = perp_proj / np.linalg.norm(perp_proj)
        binormal_vectors[i] = np.cross(tangents[i], perp_vectors[i])
    
    # Generate mesh vertices
    vertices = []
    faces = []
    
    for i, (center, perp1, perp2, radius) in enumerate(zip(smooth_spline, perp_vectors, binormal_vectors, radius_function)):
        if radius < 0.01:  # Skip nearly zero radius sections
            continue
        
        # Create circular cross-section using consistent frame
        theta = np.linspace(0, 2*np.pi, num_circumference, endpoint=False)
        for angle in theta:
            point = center + radius * (np.cos(angle) * perp1 + np.sin(angle) * perp2)
            vertices.append(point)
    
    vertices = np.array(vertices)
    
    if len(vertices) == 0:
        return None, None, None, None, None, None
    
    # Create triangular faces for the mesh
    faces = []
    sections_with_vertices = len(vertices) // num_circumference
    
    for i in range(sections_with_vertices - 1):
        for j in range(num_circumference):
            # Current ring
            curr_base = i * num_circumference
            next_base = (i + 1) * num_circumference
            
            # Current and next vertices
            v1 = curr_base + j
            v2 = curr_base + (j + 1) % num_circumference
            v3 = next_base + j
            v4 = next_base + (j + 1) % num_circumference
            
            # Two triangles per quad
            faces.append([v1, v2, v3])
            faces.append([v2, v4, v3])
    
    faces = np.array(faces)
    
    return vertices[:, 0], vertices[:, 1], vertices[:, 2], faces[:, 0], faces[:, 1], faces[:, 2]

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

def plot_fabrik_spline_visualization(target_x, target_y, target_z, num_segments=None, tolerance=None):
    """Create interactive 3D visualization of FABRIK solution with spline."""
    
    try:
        import delta_robot
        
        print(f"Solving FABRIK for target: ({target_x}, {target_y}, {target_z})")
        
        # Solve FABRIK with spline points
        result = delta_robot.solve_fabrik_with_spline(target_x, target_y, target_z, num_segments, tolerance)
        
        if not result.converged:
            print("Warning: FABRIK did not converge! Visualization may not be accurate.")
        else:
            print(f"FABRIK converged with error: {result.final_error:.4f}")
        
        # Extract data from result
        joint_positions = np.array([np.array(joint.position) for joint in result.final_chain.joints])
        spline_points = np.array(result.spline_points)
        segment_midpoints = np.array(result.segment_midpoints)
        
        print(f"Joints: {len(joint_positions)}, Spline points: {len(spline_points)}, Midpoints: {len(segment_midpoints)}")
        
        # Create smooth spline interpolation
        control_points, smooth_spline = create_smooth_spline(spline_points, 150)
        
        # Create capsule geometry around the spline
        spindle_x, spindle_y, spindle_z, faces_i, faces_j, faces_k = create_spindle_geometry(
            control_points, center_radius=60, end_radius=22, num_circumference=24, num_length=80
        )
        
        # Calculate reasonable axis ranges with equal scaling
        all_points = np.vstack([joint_positions, spline_points, [[target_x, target_y, target_z]]])
        
        # Find the maximum range across all dimensions
        x_range_size = np.max(all_points[:, 0]) - np.min(all_points[:, 0])
        y_range_size = np.max(all_points[:, 1]) - np.min(all_points[:, 1])
        z_range_size = np.max(all_points[:, 2]) - np.min(all_points[:, 2])
        max_range = max(x_range_size, y_range_size, z_range_size)
        
        # Add margin
        margin = 0.1 * max_range
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
        
        # 0. Capsule geometry (add first so it appears behind other elements)
        if spindle_x is not None and len(spindle_x) > 0:
            fig.add_trace(go.Mesh3d(
                x=spindle_x,
                y=spindle_y,
                z=spindle_z,
                i=faces_i,
                j=faces_j,
                k=faces_k,
                color='lightblue',
                opacity=0.3,
                name='Kinematic Envelope',
                showlegend=True,
                hovertemplate='Kinematic Envelope<extra></extra>'
            ))
        
        # 1. Original joint chain (FABRIK solution)
        fig.add_trace(go.Scatter3d(
            x=joint_positions[:, 0],
            y=joint_positions[:, 1], 
            z=joint_positions[:, 2],
            mode='lines+markers',
            line=dict(color='blue', width=6),
            marker=dict(size=6, color='darkblue'),  # 30% smaller: 8 -> 6
            name='FABRIK Joint Chain',
            hovertemplate='Joint %{pointNumber}<br>Position: (%{x:.2f}, %{y:.2f}, %{z:.2f})<extra></extra>'
        ))
        
        # 2. Segment midpoints (50% points)
        fig.add_trace(go.Scatter3d(
            x=segment_midpoints[:, 0],
            y=segment_midpoints[:, 1],
            z=segment_midpoints[:, 2],
            mode='markers',
            marker=dict(size=4, color='red', symbol='diamond'),  # 30% smaller: 6 -> 4
            name='Segment Midpoints (50%)',
            hovertemplate='Midpoint %{pointNumber}<br>Position: (%{x:.2f}, %{y:.2f}, %{z:.2f})<extra></extra>'
        ))
        
        # 3. Spline control points
        fig.add_trace(go.Scatter3d(
            x=control_points[:, 0],
            y=control_points[:, 1],
            z=control_points[:, 2],
            mode='markers',
            marker=dict(size=7, color='green', symbol='circle'),  # 30% smaller: 10 -> 7
            name='Spline Control Points',
            hovertemplate='Control Point %{pointNumber}<br>Position: (%{x:.2f}, %{y:.2f}, %{z:.2f})<extra></extra>'
        ))
        
        # 4. Smooth cubic spline curve
        fig.add_trace(go.Scatter3d(
            x=smooth_spline[:, 0],
            y=smooth_spline[:, 1],
            z=smooth_spline[:, 2],
            mode='lines',
            line=dict(color='lime', width=8, dash='solid'),
            name='Cubic Spline Curve',
            hovertemplate='Spline Point<br>Position: (%{x:.2f}, %{y:.2f}, %{z:.2f})<extra></extra>'
        ))
        
        # 5. Target position
        fig.add_trace(go.Scatter3d(
            x=[target_x],
            y=[target_y],
            z=[target_z],
            mode='markers',
            marker=dict(size=6, color='orange', symbol='diamond'),  # 60% smaller: 15 -> 6
            name='Target Position',
            hovertemplate='Target<br>Position: (%{x:.2f}, %{y:.2f}, %{z:.2f})<extra></extra>'
        ))
        
        # 6. Base position (origin)
        fig.add_trace(go.Scatter3d(
            x=[0],
            y=[0], 
            z=[0],
            mode='markers',
            marker=dict(size=5, color='black', symbol='square'),  # 60% smaller: 12 -> 5
            name='Base (Origin)',
            hovertemplate='Base<br>Position: (0.00, 0.00, 0.00)<extra></extra>'
        ))
        
        # Update layout
        fig.update_layout(
            title=dict(
                text=f'FABRIK Spline Visualization<br>Target: ({target_x}, {target_y}, {target_z}) | Error: {result.final_error:.4f}',
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
            width=1200,
            height=800,
            legend=dict(
                yanchor="top",
                y=0.99,
                xanchor="left", 
                x=0.01
            )
        )
        
        # Show interactive plot
        fig.show()
        
        # Print summary statistics
        print(f"\nVisualization Summary:")
        print(f"   Total joints: {len(joint_positions)}")
        print(f"   Spline control points: {len(control_points)}")
        print(f"   Smooth spline points: {len(smooth_spline)}")
        print(f"   Target error: {result.final_error:.4f} mm")
        print(f"   FABRIK converged: {'Yes' if result.converged else 'No'}")
        print(f"   Capsule envelope: Center radius {60}mm, End radius {22}mm")
        
        if hasattr(result, 'solve_time_ms'):
            print(f"   Solve time: {result.solve_time_ms:.2f} ms")
            
        return result
        
    except ImportError as e:
        print(f"ImportError: {e}")
        print("Please ensure the compiled C++ module is in the Python path.")
        return None
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()
        return None

def main():
    """Main function with command line argument parsing."""
    
    # Parse command line arguments
    if len(sys.argv) < 2:
        target_x, target_y, target_z = 100, 50, 300
        print(f"Usage: python3 {os.path.basename(__file__)} x,y,z")
        print(f"Using default target: ({target_x}, {target_y}, {target_z})")
    else:
        target_x, target_y, target_z = parse_coordinates(sys.argv[1])
    
    # Optional parameters
    num_segments = None
    tolerance = None
    if len(sys.argv) >= 3:
        try:
            num_segments = int(sys.argv[2])
        except ValueError:
            print(f"Warning: Invalid number of segments '{sys.argv[2]}', using default")
    
    if len(sys.argv) >= 4:
        try:
            tolerance = float(sys.argv[3])
        except ValueError:
            print(f"Warning: Invalid tolerance '{sys.argv[3]}', using default")
    
    print("Starting FABRIK Spline Visualization...")
    print("Required packages: plotly, scipy, numpy")
    
    try:
        import plotly
        import scipy
        print("All visualization packages available")
    except ImportError as e:
        print(f"Missing package: {e}")
        print("Install with: pip install plotly scipy")
        sys.exit(1)
    
    # Create visualization
    result = plot_fabrik_spline_visualization(target_x, target_y, target_z, num_segments, tolerance)
    
    if result is not None:
        print("\nVisualization complete! Check your browser for the interactive 3D plot.")
        print("Tip: Use mouse to rotate, zoom, and pan the 3D scene.")
    else:
        print("\nVisualization failed. Check error messages above.")

if __name__ == "__main__":
    main()