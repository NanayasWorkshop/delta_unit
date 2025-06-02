#!/usr/bin/env python3
"""
Visual test for the Kinematics module - 3D visualization of half-angle transformation and end-effector calculation
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

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

def create_cone_surface(apex, axis, angle, height, resolution=20):
    """Create cone surface for workspace visualization"""
    # Normalize axis
    axis = axis / np.linalg.norm(axis)
    
    # Create two perpendicular vectors to axis
    if abs(axis[2]) < 0.9:
        v1 = np.cross(axis, [0, 0, 1])
    else:
        v1 = np.cross(axis, [1, 0, 0])
    v1 = v1 / np.linalg.norm(v1)
    v2 = np.cross(axis, v1)
    
    # Create cone
    theta = np.linspace(0, 2*np.pi, resolution)
    z = np.linspace(0, height, resolution//2)
    THETA, Z = np.meshgrid(theta, z)
    
    # Radius increases linearly with height
    R = Z * np.tan(angle)
    
    # Convert to Cartesian coordinates
    X_local = R * np.cos(THETA)
    Y_local = R * np.sin(THETA)
    Z_local = Z
    
    # Transform to world coordinates
    X = apex[0] + X_local * v1[0] + Y_local * v2[0] + Z_local * axis[0]
    Y = apex[1] + X_local * v1[1] + Y_local * v2[1] + Z_local * axis[1]
    Z = apex[2] + X_local * v1[2] + Y_local * v2[2] + Z_local * axis[2]
    
    return X, Y, Z

def visualize_kinematics(input_x, input_y, input_z):
    """Create 3D visualization of kinematics calculation"""
    
    try:
        import delta_robot
        km = delta_robot.kinematics
    except ImportError as e:
        print(f"Error: delta_robot package not found: {e}")
        return False
    
    # Calculate kinematics result
    result = km.KinematicsModule.calculate(input_x, input_y, input_z)
    
    # Get constants from module
    try:
        import delta_robot
        WORKING_HEIGHT = delta_robot.WORKING_HEIGHT
        MIN_HEIGHT = delta_robot.MIN_HEIGHT
        MOTOR_LIMIT = delta_robot.MOTOR_LIMIT
        WORKSPACE_CONE_ANGLE = delta_robot.WORKSPACE_CONE_ANGLE_RAD
    except:
        # Fallback only if module not available
        WORKING_HEIGHT = 11.5
        MIN_HEIGHT = 101.0
        MOTOR_LIMIT = 11.0
        WORKSPACE_CONE_ANGLE = math.pi / 6
    
    # Setup figure with subplots
    fig = plt.figure(figsize=(20, 15))
    
    # Main 3D kinematics view
    ax1 = fig.add_subplot(231, projection='3d')
    
    # Plot robot structure
    # Base center
    ax1.scatter([0], [0], [0], color='black', s=200, marker='s', label='Base (0,0,0)')
    
    # Joint H (working height)
    joint_H = [0, 0, WORKING_HEIGHT]
    ax1.scatter([joint_H[0]], [joint_H[1]], [joint_H[2]], color='purple', s=150, marker='o', 
               label=f'Joint H (0,0,{WORKING_HEIGHT})')
    
    # Original input vector
    input_norm = math.sqrt(input_x**2 + input_y**2 + input_z**2)
    input_unit = np.array([input_x, input_y, input_z]) / input_norm
    ax1.quiver(0, 0, 0, input_x, input_y, input_z, 
               color='red', arrow_length_ratio=0.05, linewidth=3, label='Original Input')
    
    # Transformed vector (half-angle)
    tv = result.transformed_vector
    transform_scale = input_norm  # Scale for visibility
    ax1.quiver(0, 0, 0, tv.x * transform_scale, tv.y * transform_scale, tv.z * transform_scale,
               color='blue', arrow_length_ratio=0.05, linewidth=3, label='Transformed (Half-Angle)')
    
    # Z-axis reference
    ax1.quiver(0, 0, 0, 0, 0, input_norm, 
               color='green', arrow_length_ratio=0.05, linewidth=2, alpha=0.7, label='+Z Axis')
    
    # Point G calculation
    vector_length = MIN_HEIGHT + 2*MOTOR_LIMIT + result.prismatic_joint_length
    point_G = np.array(joint_H) + input_unit * vector_length
    ax1.scatter([point_G[0]], [point_G[1]], [point_G[2]], color='orange', s=150, marker='^', 
               label=f'Point G')
    
    # Line H to G
    ax1.plot([joint_H[0], point_G[0]], [joint_H[1], point_G[1]], [joint_H[2], point_G[2]], 
             'orange', linewidth=4, alpha=0.8, label=f'H→G ({vector_length:.1f})')
    
    # End-effector position
    ee = result.end_effector_position
    ax1.scatter([ee.x], [ee.y], [ee.z], color='red', s=200, marker='*', 
               label='End-Effector', edgecolor='darkred', linewidth=2)
    
    # Mirror validation: line from G to end-effector
    ax1.plot([point_G[0], ee.x], [point_G[1], ee.y], [point_G[2], ee.z], 
             'purple', linewidth=3, alpha=0.8, linestyle='--', label='G→End-Eff')
    
    # Midpoint of H-G
    midpoint = (np.array(joint_H) + point_G) / 2
    ax1.scatter([midpoint[0]], [midpoint[1]], [midpoint[2]], color='cyan', s=100, marker='o', 
               alpha=0.7, label='H-G Midpoint')
    
    # Workspace cone visualization
    try:
        cone_X, cone_Y, cone_Z = create_cone_surface([0, 0, 0], [0, 0, 1], WORKSPACE_CONE_ANGLE, 50, 15)
        ax1.plot_surface(cone_X, cone_Y, cone_Z, alpha=0.1, color='gray', label='Workspace Cone')
    except:
        pass
    
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title('Kinematics 3D Overview')
    ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    # Set limits
    max_range = max(50, abs(point_G[0]), abs(point_G[1]), abs(point_G[2]), abs(ee.x), abs(ee.y), abs(ee.z))
    ax1.set_xlim([-max_range*0.5, max_range*0.5])
    ax1.set_ylim([-max_range*0.5, max_range*0.5])
    ax1.set_zlim([0, max_range])
    
    # Angle visualization
    ax2 = fig.add_subplot(232)
    
    # Show angle between input and Z-axis, and half-angle
    z_vec = [0, 1]  # Z-axis in 2D
    input_angle = result.input_angle_from_z
    half_angle = input_angle / 2
    
    # Draw vectors
    ax2.arrow(0, 0, 0, 1, head_width=0.05, head_length=0.05, fc='green', ec='green', 
              linewidth=2, label='+Z Axis')
    
    input_2d = [np.sin(input_angle), np.cos(input_angle)]
    ax2.arrow(0, 0, input_2d[0], input_2d[1], head_width=0.05, head_length=0.05, 
              fc='red', ec='red', linewidth=2, label='Original Input')
    
    half_2d = [np.sin(half_angle), np.cos(half_angle)]
    ax2.arrow(0, 0, half_2d[0], half_2d[1], head_width=0.05, head_length=0.05, 
              fc='blue', ec='blue', linewidth=2, label='Half-Angle Vector')
    
    # Draw angle arcs
    theta_full = np.linspace(0, input_angle, 30)
    arc_full_x = 0.7 * np.sin(theta_full)
    arc_full_y = 0.7 * np.cos(theta_full)
    ax2.plot(arc_full_x, arc_full_y, 'red', linewidth=3, label=f'Full Angle ({math.degrees(input_angle):.1f}°)')
    
    theta_half = np.linspace(0, half_angle, 30)
    arc_half_x = 0.5 * np.sin(theta_half)
    arc_half_y = 0.5 * np.cos(theta_half)
    ax2.plot(arc_half_x, arc_half_y, 'blue', linewidth=3, label=f'Half Angle ({math.degrees(half_angle):.1f}°)')
    
    ax2.set_xlim([-1.2, 1.2])
    ax2.set_ylim([0, 1.2])
    ax2.set_xlabel('X (projected)')
    ax2.set_ylabel('Z')
    ax2.set_title('Half-Angle Transformation')
    ax2.grid(True)
    ax2.legend()
    ax2.set_aspect('equal')
    
    # Mirror geometry visualization
    ax3 = fig.add_subplot(233)
    
    # 2D side view showing mirror operation
    # H, G, midpoint, and end-effector in XZ plane (project to major axis)
    major_axis = 0 if abs(input_x) > abs(input_y) else 1
    if major_axis == 0:  # Project to XZ
        h_2d = [joint_H[0], joint_H[2]]
        g_2d = [point_G[0], point_G[2]]
        mid_2d = [midpoint[0], midpoint[2]]
        ee_2d = [ee.x, ee.z]
        base_2d = [0, 0]
        ax3.set_xlabel('X')
    else:  # Project to YZ
        h_2d = [joint_H[1], joint_H[2]]
        g_2d = [point_G[1], point_G[2]]
        mid_2d = [midpoint[1], midpoint[2]]
        ee_2d = [ee.y, ee.z]
        base_2d = [0, 0]
        ax3.set_xlabel('Y')
    
    # Plot points
    ax3.scatter(*base_2d, color='black', s=100, marker='s', label='Base')
    ax3.scatter(*h_2d, color='purple', s=100, marker='o', label='Joint H')
    ax3.scatter(*g_2d, color='orange', s=100, marker='^', label='Point G')
    ax3.scatter(*mid_2d, color='cyan', s=100, marker='o', label='Midpoint')
    ax3.scatter(*ee_2d, color='red', s=100, marker='*', label='End-Effector')
    
    # Plot lines
    ax3.plot([h_2d[0], g_2d[0]], [h_2d[1], g_2d[1]], 'orange', linewidth=3, label='H→G')
    ax3.plot([g_2d[0], ee_2d[0]], [g_2d[1], ee_2d[1]], 'purple', linewidth=2, linestyle='--', label='G→EE')
    ax3.plot([base_2d[0], h_2d[0]], [base_2d[1], h_2d[1]], 'gray', linewidth=2, linestyle=':', label='Base→H')
    
    # Mirror plane (perpendicular to input direction through midpoint)
    plane_normal = input_unit[:2] if major_axis == 0 else input_unit[1:]
    plane_tangent = [-plane_normal[1], plane_normal[0]]
    plane_length = 20
    plane_start = np.array(mid_2d) - np.array(plane_tangent) * plane_length
    plane_end = np.array(mid_2d) + np.array(plane_tangent) * plane_length
    ax3.plot([plane_start[0], plane_end[0]], [plane_start[1], plane_end[1]], 
             'green', linewidth=2, alpha=0.7, label='Mirror Plane')
    
    ax3.set_ylabel('Z')
    ax3.set_title('Mirror Geometry (2D Projection)')
    ax3.grid(True)
    ax3.legend()
    ax3.set_aspect('equal')
    
    # Distance validation plot
    ax4 = fig.add_subplot(234)
    
    # Calculate distances
    base_to_H = np.linalg.norm(np.array(joint_H) - np.array([0, 0, 0]))
    G_to_EE = np.linalg.norm(point_G - np.array([ee.x, ee.y, ee.z]))
    H_to_G = vector_length
    
    distances = ['Base→H', 'G→End-Eff', 'H→G']
    values = [base_to_H, G_to_EE, H_to_G]
    colors = ['gray', 'purple', 'orange']
    
    bars = ax4.bar(distances, values, color=colors, alpha=0.7)
    ax4.set_ylabel('Distance')
    ax4.set_title('Distance Validation')
    ax4.grid(True, axis='y')
    
    # Add value labels on bars
    for bar, value in zip(bars, values):
        height = bar.get_height()
        ax4.text(bar.get_x() + bar.get_width()/2., height + 0.5,
                f'{value:.2f}', ha='center', va='bottom', fontweight='bold')
    
    # Mirror validation
    mirror_error = abs(base_to_H - G_to_EE)
    if mirror_error < 0.001:
        validation_text = f"✓ Mirror Validation PASSED\nBase→H = G→EE ({mirror_error:.6f})"
        validation_color = 'green'
    else:
        validation_text = f"✗ Mirror Validation FAILED\nDifference: {mirror_error:.6f}"
        validation_color = 'red'
    
    ax4.text(0.5, 0.95, validation_text, transform=ax4.transAxes, 
             bbox=dict(boxstyle='round', facecolor=validation_color, alpha=0.3),
             ha='center', va='top', fontweight='bold')
    
    # Fermat data visualization
    ax5 = fig.add_subplot(235, projection='3d')
    
    fp = result.fermat_data.fermat_point
    ax5.scatter([fp.x], [fp.y], [fp.z], color='gold', s=150, marker='*', label='Fermat Point')
    
    # Get base positions for context
    try:
        import delta_robot
        base_A = delta_robot.get_base_position_A()
        base_B = delta_robot.get_base_position_B()
        base_C = delta_robot.get_base_position_C()
        
        # Plot 3D triangle using actual data
        triangle_x = [base_A.x, base_B.x, base_C.x, base_A.x]
        triangle_y = [base_A.y, base_B.y, base_C.y, base_A.y]
        triangle_z = [result.fermat_data.z_A, result.fermat_data.z_B, result.fermat_data.z_C, result.fermat_data.z_A]
        ax5.plot(triangle_x, triangle_y, triangle_z, 'k-', linewidth=2, label='3D Triangle ABC')
        
        ax5.scatter([base_A.x], [base_A.y], [result.fermat_data.z_A], color='red', s=100, marker='^')
        ax5.scatter([base_B.x], [base_B.y], [result.fermat_data.z_B], color='green', s=100, marker='^')
        ax5.scatter([base_C.x], [base_C.y], [result.fermat_data.z_C], color='blue', s=100, marker='^')
    except:
        pass
    
    # Transformed vector
    ax5.quiver(0, 0, 0, tv.x * 20, tv.y * 20, tv.z * 20,
               color='blue', arrow_length_ratio=0.1, linewidth=2, label='Transformed Vector')
    
    ax5.set_xlabel('X')
    ax5.set_ylabel('Y')
    ax5.set_zlabel('Z')
    ax5.set_title('Fermat Calculation Context')
    ax5.legend()
    
    # Summary text
    ax6 = fig.add_subplot(236)
    ax6.axis('off')
    
    summary_text = f"""
KINEMATICS RESULTS SUMMARY

Input Vector: ({input_x}, {input_y}, {input_z})
  Magnitude: {input_norm:.3f}
  Angle from +Z: {result.input_angle_from_z:.3f} rad = {math.degrees(result.input_angle_from_z):.1f}°

Half-Angle Transformation:
  Half angle: {result.input_angle_from_z/2:.3f} rad = {math.degrees(result.input_angle_from_z/2):.1f}°
  Transformed vector: ({tv.x:.3f}, {tv.y:.3f}, {tv.z:.3f})

Fermat Calculation (using transformed vector):
  Fermat point: ({fp.x:.3f}, {fp.y:.3f}, {fp.z:.3f})
  Prismatic joint: {result.prismatic_joint_length:.3f}

End-Effector Calculation:
  Joint H position: (0, 0, {WORKING_HEIGHT})
  Vector length H→G: {vector_length:.3f}
  Point G: ({point_G[0]:.3f}, {point_G[1]:.3f}, {point_G[2]:.3f})
  End-effector: ({ee.x:.3f}, {ee.y:.3f}, {ee.z:.3f})

Mirror Validation:
  Distance Base→H: {base_to_H:.3f}
  Distance G→End-Eff: {G_to_EE:.3f}
  Difference: {mirror_error:.6f}
  Status: {"PASSED" if mirror_error < 0.001 else "FAILED"}

Key Insight:
  The end-effector is the mirror of the base point across 
  the plane through the H-G midpoint with normal = input direction.
    """
    
    ax6.text(0.05, 0.95, summary_text, transform=ax6.transAxes, fontsize=9,
             verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))
    
    plt.tight_layout()
    plt.show()
    
    return True

def main():
    # Parse command line arguments for input vector
    if len(sys.argv) == 2:
        input_x, input_y, input_z = parse_coordinates(sys.argv[1])
        print(f"Using command line input: ({input_x}, {input_y}, {input_z})")
    else:
        input_x, input_y, input_z = 0.612, 0.612, 0.5
        print("Using default input: (5, 4, 7)")
        print("Usage: python3 test_kinematics_visual.py x,y,z")
    
    print("Creating Kinematics visualization...")
    success = visualize_kinematics(input_x, input_y, input_z)
    
    if success:
        print("Visualization complete!")
        print("\nVisualization shows:")
        print("- 3D Overview: Complete robot structure and calculated points")
        print("- Half-Angle: Transformation from input to Fermat calculation vector")
        print("- Mirror Geometry: How end-effector is calculated using mirror operation")
        print("- Distance Validation: Verification of mirror property")
        print("- Fermat Context: Underlying Fermat point calculation")
        print("- Summary: All calculated values and validation status")
    else:
        print("Visualization failed - check module installation")

if __name__ == "__main__":
    main()