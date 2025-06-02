#!/usr/bin/env python3
"""
Visual test for the Joint State module - 3D visualization of joint rotations and prismatic extension
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

def create_rotation_visualization(ax, center, axis, angle, radius=5, color='blue', label='Rotation'):
    """Create visualization of rotation around an axis"""
    # Create circle around rotation axis
    if np.allclose(axis, [1, 0, 0]):  # X-axis
        theta = np.linspace(0, 2*np.pi, 50)
        circle_y = radius * np.cos(theta)
        circle_z = radius * np.sin(theta)
        circle_x = np.full_like(theta, center[0])
        circle = np.array([circle_x, circle_y + center[1], circle_z + center[2]])
    elif np.allclose(axis, [0, 1, 0]):  # Y-axis
        theta = np.linspace(0, 2*np.pi, 50)
        circle_x = radius * np.cos(theta)
        circle_z = radius * np.sin(theta)
        circle_y = np.full_like(theta, center[1])
        circle = np.array([circle_x + center[0], circle_y, circle_z + center[2]])
    else:  # Z-axis or other
        theta = np.linspace(0, 2*np.pi, 50)
        circle_x = radius * np.cos(theta)
        circle_y = radius * np.sin(theta)
        circle_z = np.full_like(theta, center[2])
        circle = np.array([circle_x + center[0], circle_y + center[1], circle_z])
    
    ax.plot(circle[0], circle[1], circle[2], '--', color=color, alpha=0.5, label=f'{label} ({math.degrees(angle):.1f}°)')
    
    # Show angle arc
    if angle != 0:
        theta_arc = np.linspace(0, angle, 20)
        if np.allclose(axis, [1, 0, 0]):  # Roll (X-axis)
            arc_y = radius * 0.8 * np.cos(theta_arc)
            arc_z = radius * 0.8 * np.sin(theta_arc)
            arc_x = np.full_like(theta_arc, center[0])
            ax.plot(arc_x, arc_y + center[1], arc_z + center[2], color=color, linewidth=3)
        elif np.allclose(axis, [0, 1, 0]):  # Pitch (Y-axis)
            arc_x = radius * 0.8 * np.cos(theta_arc)
            arc_z = radius * 0.8 * np.sin(theta_arc)
            arc_y = np.full_like(theta_arc, center[1])
            ax.plot(arc_x + center[0], arc_y, arc_z + center[2], color=color, linewidth=3)

def create_coordinate_frame(ax, origin, roll=0, pitch=0, yaw=0, size=10, label='Frame'):
    """Create coordinate frame visualization with rotations"""
    # Create rotation matrices
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])
    
    # Combined rotation
    R = R_z @ R_y @ R_x
    
    # Original axes
    x_axis = np.array([size, 0, 0])
    y_axis = np.array([0, size, 0])
    z_axis = np.array([0, 0, size])
    
    # Rotated axes
    x_rot = R @ x_axis
    y_rot = R @ y_axis
    z_rot = R @ z_axis
    
    # Plot axes
    ax.quiver(origin[0], origin[1], origin[2], x_rot[0], x_rot[1], x_rot[2], 
              color='red', arrow_length_ratio=0.1, label=f'{label} X')
    ax.quiver(origin[0], origin[1], origin[2], y_rot[0], y_rot[1], y_rot[2], 
              color='green', arrow_length_ratio=0.1, label=f'{label} Y')
    ax.quiver(origin[0], origin[1], origin[2], z_rot[0], z_rot[1], z_rot[2], 
              color='blue', arrow_length_ratio=0.1, label=f'{label} Z')
    
    return R

def visualize_joint_state(direction_x, direction_y, direction_z):
    """Create 3D visualization of joint state calculation"""
    
    try:
        # FIXED: Use proper delta_robot package import
        import delta_robot
        fm = delta_robot.fermat
        jsm = delta_robot.joint_state
    except ImportError as e:
        print(f"Error: delta_robot package not found: {e}")
        return False
    
    # Calculate results
    direction_vector = delta_robot.Vector3(direction_x, direction_y, direction_z)
    fermat_result = fm.FermatModule.calculate(direction_x, direction_y, direction_z)
    joint_result = jsm.JointStateModule.calculate_from_fermat(direction_vector, fermat_result)
    
    # Setup figure with subplots
    fig = plt.figure(figsize=(18, 12))
    
    # Main 3D view
    ax1 = fig.add_subplot(221, projection='3d')
    
    # Plot robot structure
    # Base center
    ax1.scatter([0], [0], [0], color='black', s=200, marker='s', label='Base Center')
    
    # Joint H (working height)
    working_height = delta_robot.WORKING_HEIGHT
    ax1.scatter([0], [0], [working_height], color='purple', s=150, marker='o', label=f'Joint H (z={working_height})')
    
    # Fermat point
    fp = fermat_result.fermat_point
    ax1.scatter([fp.x], [fp.y], [fp.z], color='gold', s=150, marker='*', label='Fermat Point')
    
    # Original direction vector
    direction_norm = np.sqrt(direction_x**2 + direction_y**2 + direction_z**2)
    direction_unit = np.array([direction_x, direction_y, direction_z]) / direction_norm
    ax1.quiver(0, 0, 0, direction_x, direction_y, direction_z, 
               color='orange', arrow_length_ratio=0.1, linewidth=3, label='Input Direction')
    
    # Prismatic joint visualization (cylinder from fermat point)
    prismatic_length = joint_result.prismatic_joint
    cylinder_end = np.array([fp.x, fp.y, fp.z]) + direction_unit * (prismatic_length/2)
    ax1.plot([fp.x, cylinder_end[0]], [fp.y, cylinder_end[1]], [fp.z, cylinder_end[2]], 
             'purple', linewidth=8, alpha=0.7, label=f'Prismatic Joint ({prismatic_length:.2f})')
    
    # Coordinate frame at base (showing roll and pitch rotations)
    base_frame_R = create_coordinate_frame(ax1, [0, 0, 0], 
                                         roll=joint_result.roll_joint, 
                                         pitch=joint_result.pitch_joint, 
                                         size=8, label='Base Frame')
    
    # Show rotation visualizations
    create_rotation_visualization(ax1, [0, 0, 0], [1, 0, 0], joint_result.roll_joint, 
                                radius=6, color='red', label='Roll')
    create_rotation_visualization(ax1, [0, 0, 0], [0, 1, 0], joint_result.pitch_joint, 
                                radius=7, color='green', label='Pitch')
    
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title('Joint State 3D Visualization')
    ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    # Set limits
    max_range = max(20, abs(prismatic_length))
    ax1.set_xlim([-max_range, max_range])
    ax1.set_ylim([-max_range, max_range])
    ax1.set_zlim([0, max_range])
    
    # Roll visualization (YZ plane view)
    ax2 = fig.add_subplot(222)
    
    # Original Y and Z axes
    ax2.arrow(0, 0, 0, 5, head_width=0.3, head_length=0.3, fc='green', ec='green', alpha=0.5, label='Original Y')
    ax2.arrow(0, 0, 0, 5, head_width=0.3, head_length=0.3, fc='blue', ec='blue', alpha=0.5, label='Original Z')
    
    # Rotated axes after roll
    roll_angle = joint_result.roll_joint
    y_rotated = [0, 5 * np.cos(roll_angle)]
    z_rotated = [0, 5 * np.sin(roll_angle)]
    ax2.arrow(0, 0, y_rotated[1], z_rotated[1], head_width=0.3, head_length=0.3, 
              fc='darkgreen', ec='darkgreen', linewidth=2, label=f'Y after Roll ({math.degrees(roll_angle):.1f}°)')
    
    # Show angle arc
    if roll_angle != 0:
        theta = np.linspace(0, roll_angle, 20)
        arc_y = 3 * np.cos(theta)
        arc_z = 3 * np.sin(theta)
        ax2.plot(arc_y, arc_z, 'red', linewidth=3, label=f'Roll Angle')
    
    ax2.set_xlim([-6, 6])
    ax2.set_ylim([-6, 6])
    ax2.set_xlabel('Y')
    ax2.set_ylabel('Z')
    ax2.set_title('Roll Rotation (around X-axis)')
    ax2.grid(True)
    ax2.legend()
    ax2.set_aspect('equal')
    
    # Pitch visualization (XZ plane view)
    ax3 = fig.add_subplot(223)
    
    # Original X and Z axes
    ax3.arrow(0, 0, 5, 0, head_width=0.3, head_length=0.3, fc='red', ec='red', alpha=0.5, label='Original X')
    ax3.arrow(0, 0, 0, 5, head_width=0.3, head_length=0.3, fc='blue', ec='blue', alpha=0.5, label='Original Z')
    
    # Rotated axes after pitch
    pitch_angle = joint_result.pitch_joint
    x_rotated = [5 * np.cos(pitch_angle), 0]
    z_rotated = [5 * np.sin(pitch_angle), 0]
    ax3.arrow(0, 0, x_rotated[0], z_rotated[0], head_width=0.3, head_length=0.3, 
              fc='darkred', ec='darkred', linewidth=2, label=f'X after Pitch ({math.degrees(pitch_angle):.1f}°)')
    
    # Show angle arc
    if pitch_angle != 0:
        theta = np.linspace(0, pitch_angle, 20)
        arc_x = 3 * np.cos(theta)
        arc_z = 3 * np.sin(theta)
        ax3.plot(arc_x, arc_z, 'green', linewidth=3, label=f'Pitch Angle')
    
    ax3.set_xlim([-6, 6])
    ax3.set_ylim([-6, 6])
    ax3.set_xlabel('X')
    ax3.set_ylabel('Z')
    ax3.set_title('Pitch Rotation (around Y-axis)')
    ax3.grid(True)
    ax3.legend()
    ax3.set_aspect('equal')
    
    # Joint values summary
    ax4 = fig.add_subplot(224)
    ax4.axis('off')
    
    # Create summary text
    summary_text = f"""
JOINT STATE RESULTS

Input Direction Vector:
  ({direction_x}, {direction_y}, {direction_z})
  Magnitude: {direction_norm:.3f}
  Normalized: ({direction_unit[0]:.3f}, {direction_unit[1]:.3f}, {direction_unit[2]:.3f})

Fermat Point:
  ({fp.x:.3f}, {fp.y:.3f}, {fp.z:.3f})

Joint Values:
  Prismatic: {prismatic_length:.3f} units
  Roll:      {roll_angle:.3f} rad = {math.degrees(roll_angle):.1f}°
  Pitch:     {pitch_angle:.3f} rad = {math.degrees(pitch_angle):.1f}°

Calculations:
  Prismatic = 2 × fermat_z = 2 × {fp.z:.3f} = {prismatic_length:.3f}
  Roll = -atan2(y/|v|, z/|v|) = -atan2({direction_unit[1]:.3f}, {direction_unit[2]:.3f})
  Pitch = atan2(x/|v|, z/|v|) = atan2({direction_unit[0]:.3f}, {direction_unit[2]:.3f})

Interpretation:
  • Prismatic extends/retracts along robot axis
  • Roll rotates end-effector around X-axis (tilt forward/back)
  • Pitch rotates end-effector around Y-axis (tilt left/right)
    """
    
    ax4.text(0.05, 0.95, summary_text, transform=ax4.transAxes, fontsize=10,
             verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
    
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
        print("Usage: python3 test_joint_state_visual.py x,y,z")
    
    print("Creating Joint State visualization...")
    success = visualize_joint_state(direction_x, direction_y, direction_z)
    
    if success:
        print("Visualization complete!")
        print("\nVisualization shows:")
        print("- 3D view: Robot structure with joint rotations and prismatic extension")
        print("- Roll view: Rotation around X-axis (YZ plane)")
        print("- Pitch view: Rotation around Y-axis (XZ plane)")
        print("- Summary: Calculated joint values and formulas")
    else:
        print("Visualization failed - check module installation")

if __name__ == "__main__":
    main()