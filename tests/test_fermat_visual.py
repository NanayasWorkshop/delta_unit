#!/usr/bin/env python3
"""
Visual test for the Fermat module - 3D visualization of base points, plane intersection, and Fermat point
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def parse_coordinates(coord_str):
    """Parse coordinate string like '5,4,7' into x,y,z values"""
    try:
        coords = [float(x.strip()) for x in coord_str.split(',')]
        if len(coords) != 3:
            raise ValueError("Expected 3 coordinates")
        return coords
    except ValueError as e:
        print(f"Error parsing coordinates '{coord_str}': {e}")
        print("Expected format: x,y,z (e.g., 5,4,7)")
        sys.exit(1)

def create_plane_mesh(normal, size=50, resolution=20):
    """Create mesh points for plane visualization"""
    # Find two perpendicular vectors to the normal
    if abs(normal[2]) < 0.9:
        v1 = np.cross(normal, [0, 0, 1])
    else:
        v1 = np.cross(normal, [1, 0, 0])
    v1 = v1 / np.linalg.norm(v1)
    v2 = np.cross(normal, v1)
    v2 = v2 / np.linalg.norm(v2)
    
    # Create grid
    u = np.linspace(-size, size, resolution)
    v = np.linspace(-size, size, resolution)
    U, V = np.meshgrid(u, v)
    
    # Calculate points on plane (plane passes through origin)
    X = U * v1[0] + V * v2[0]
    Y = U * v1[1] + V * v2[1]
    Z = U * v1[2] + V * v2[2]
    
    return X, Y, Z

def visualize_fermat(direction_x, direction_y, direction_z):
    """Create 3D visualization of Fermat calculation"""
    
    try:
        import delta_robot
        fm = delta_robot.fermat
    except ImportError:
        print("Error: delta_robot package not found. Build first.")
        return False
    
    # Calculate Fermat result
    result = fm.FermatModule.calculate(direction_x, direction_y, direction_z)
    
    # Get base positions
    base_A = fm.FermatModule.get_base_A()
    base_B = fm.FermatModule.get_base_B()
    base_C = fm.FermatModule.get_base_C()
    
    # Setup figure
    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot base positions (in XY plane)
    ax.scatter([base_A.x], [base_A.y], [0], color='red', s=100, label='Base A (XY)')
    ax.scatter([base_B.x], [base_B.y], [0], color='green', s=100, label='Base B (XY)')
    ax.scatter([base_C.x], [base_C.y], [0], color='blue', s=100, label='Base C (XY)')
    
    # Plot base triangle in XY plane
    triangle_x = [base_A.x, base_B.x, base_C.x, base_A.x]
    triangle_y = [base_A.y, base_B.y, base_C.y, base_A.y]
    triangle_z = [0, 0, 0, 0]
    ax.plot(triangle_x, triangle_y, triangle_z, 'k--', alpha=0.5, label='Base Triangle')
    
    # Plot 3D points after plane intersection
    ax.scatter([base_A.x], [base_A.y], [result.z_A], color='red', s=150, marker='^', label=f'Point A (z={result.z_A:.2f})')
    ax.scatter([base_B.x], [base_B.y], [result.z_B], color='green', s=150, marker='^', label=f'Point B (z={result.z_B:.2f})')
    ax.scatter([base_C.x], [base_C.y], [result.z_C], color='blue', s=150, marker='^', label=f'Point C (z={result.z_C:.2f})')
    
    # Plot 3D triangle
    triangle_3d_x = [base_A.x, base_B.x, base_C.x, base_A.x]
    triangle_3d_y = [base_A.y, base_B.y, base_C.y, base_A.y]
    triangle_3d_z = [result.z_A, result.z_B, result.z_C, result.z_A]
    ax.plot(triangle_3d_x, triangle_3d_y, triangle_3d_z, 'k-', linewidth=2, label='3D Triangle ABC')
    
    # Plot Fermat point
    ax.scatter([result.fermat_point.x], [result.fermat_point.y], [result.fermat_point.z], 
               color='gold', s=200, marker='*', label='Fermat Point', edgecolor='orange', linewidth=2)
    
    # Plot lines from Fermat point to vertices
    ax.plot([result.fermat_point.x, base_A.x], [result.fermat_point.y, base_A.y], 
            [result.fermat_point.z, result.z_A], 'r:', alpha=0.7)
    ax.plot([result.fermat_point.x, base_B.x], [result.fermat_point.y, base_B.y], 
            [result.fermat_point.z, result.z_B], 'g:', alpha=0.7)
    ax.plot([result.fermat_point.x, base_C.x], [result.fermat_point.y, base_C.y], 
            [result.fermat_point.z, result.z_C], 'b:', alpha=0.7)
    
    # Plot direction vector from origin
    direction_norm = np.sqrt(direction_x**2 + direction_y**2 + direction_z**2)
    direction_normalized = np.array([direction_x, direction_y, direction_z]) / direction_norm
    arrow_length = 30
    ax.quiver(0, 0, 0, direction_normalized[0]*arrow_length, direction_normalized[1]*arrow_length, direction_normalized[2]*arrow_length,
              color='purple', arrow_length_ratio=0.1, linewidth=3, label='Direction Vector')
    
    # Plot plane (transparent)
    try:
        X, Y, Z = create_plane_mesh(direction_normalized, size=40, resolution=15)
        ax.plot_surface(X, Y, Z, alpha=0.2, color='purple', label='Intersection Plane')
    except:
        print("Could not plot plane surface")
    
    # Plot vertical lines from base to 3D points
    ax.plot([base_A.x, base_A.x], [base_A.y, base_A.y], [0, result.z_A], 'r:', alpha=0.3)
    ax.plot([base_B.x, base_B.x], [base_B.y, base_B.y], [0, result.z_B], 'g:', alpha=0.3)
    ax.plot([base_C.x, base_C.x], [base_C.y, base_C.y], [0, result.z_C], 'b:', alpha=0.3)
    
    # Formatting
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Fermat Point Calculation\nDirection: ({direction_x}, {direction_y}, {direction_z})')
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    # Set equal aspect ratio
    max_range = 40
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    ax.set_zlim([-max_range, max_range])
    
    # Add text info
    info_text = f"""
Fermat Point: ({result.fermat_point.x:.3f}, {result.fermat_point.y:.3f}, {result.fermat_point.z:.3f})
Z Intersections:
  A: {result.z_A:.3f}
  B: {result.z_B:.3f}  
  C: {result.z_C:.3f}
Direction: ({direction_x}, {direction_y}, {direction_z})
Normalized: ({direction_normalized[0]:.3f}, {direction_normalized[1]:.3f}, {direction_normalized[2]:.3f})
    """
    
    plt.figtext(0.02, 0.02, info_text, fontsize=10, verticalalignment='bottom', 
                bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))
    
    plt.tight_layout()
    plt.show()
    
    return True

def main():
    # Parse command line arguments for direction vector
    if len(sys.argv) == 2:
        direction_x, direction_y, direction_z = parse_coordinates(sys.argv[1])
        print(f"Using command line direction: ({direction_x}, {direction_y}, {direction_z})")
    else:
        direction_x, direction_y, direction_z = 5, 4, 7
        print("Using default direction: (5, 4, 7)")
        print("Usage: python3 test_fermat_visual.py x,y,z")
    
    print("Creating Fermat visualization...")
    success = visualize_fermat(direction_x, direction_y, direction_z)
    
    if success:
        print("Visualization complete!")
        print("\nVisualization shows:")
        print("- Red/Green/Blue points: Base positions and their Z intersections")
        print("- Gold star: Fermat point (minimizes sum of distances to A,B,C)")
        print("- Purple arrow: Input direction vector")
        print("- Purple plane: Intersection plane through origin")
        print("- Black triangle: 3D triangle ABC on the plane")
    else:
        print("Visualization failed - check module installation")

if __name__ == "__main__":
    main()