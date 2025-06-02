#!/usr/bin/env python3
"""
Visual test for the Orientation module - 3D visualization of coordinate frame transformations and final transformation matrix
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

def draw_coordinate_frame(ax, origin, u_axis, v_axis, w_axis, scale=10, label_prefix="", alpha=1.0):
    """Draw coordinate frame with U, V, W axes"""
    # Draw axes
    ax.quiver(origin[0], origin[1], origin[2], u_axis[0]*scale, u_axis[1]*scale, u_axis[2]*scale,
              color='red', arrow_length_ratio=0.1, alpha=alpha, linewidth=2, label=f'{label_prefix}U')
    ax.quiver(origin[0], origin[1], origin[2], v_axis[0]*scale, v_axis[1]*scale, v_axis[2]*scale,
              color='green', arrow_length_ratio=0.1, alpha=alpha, linewidth=2, label=f'{label_prefix}V')
    ax.quiver(origin[0], origin[1], origin[2], w_axis[0]*scale, w_axis[1]*scale, w_axis[2]*scale,
              color='blue', arrow_length_ratio=0.1, alpha=alpha, linewidth=2, label=f'{label_prefix}W')
    
    # Draw origin point
    ax.scatter([origin[0]], [origin[1]], [origin[2]], color='black', s=100, alpha=alpha)
    
    return ax

def draw_transformation_steps(ax, frames, labels, colors, scales=None):
    """Draw multiple coordinate frames to show transformation steps"""
    if scales is None:
        scales = [8] * len(frames)
    
    for i, (frame, label, color, scale) in enumerate(zip(frames, labels, colors, scales)):
        origin = [frame.origin.x, frame.origin.y, frame.origin.z]
        u_axis = [frame.u_axis.x, frame.u_axis.y, frame.u_axis.z]
        v_axis = [frame.v_axis.x, frame.v_axis.y, frame.v_axis.z]
        w_axis = [frame.w_axis.x, frame.w_axis.y, frame.w_axis.z]
        
        alpha = 0.7 if i < len(frames)-1 else 1.0  # Final frame more opaque
        
        # Draw frame
        ax.quiver(origin[0], origin[1], origin[2], u_axis[0]*scale, u_axis[1]*scale, u_axis[2]*scale,
                  color='red', arrow_length_ratio=0.1, alpha=alpha, linewidth=2)
        ax.quiver(origin[0], origin[1], origin[2], v_axis[0]*scale, v_axis[1]*scale, v_axis[2]*scale,
                  color='green', arrow_length_ratio=0.1, alpha=alpha, linewidth=2)
        ax.quiver(origin[0], origin[1], origin[2], w_axis[0]*scale, w_axis[1]*scale, w_axis[2]*scale,
                  color='blue', arrow_length_ratio=0.1, alpha=alpha, linewidth=2)
        
        # Draw origin
        ax.scatter([origin[0]], [origin[1]], [origin[2]], color=color, s=150, alpha=alpha, 
                  marker='o', edgecolor='black', linewidth=1, label=label)

def create_matrix_heatmap(ax, matrix, title):
    """Create heatmap visualization of 4x4 transformation matrix"""
    # Extract matrix data
    data = np.zeros((4, 4))
    for i in range(4):
        for j in range(4):
            data[i][j] = matrix[i, j]
    
    # Create heatmap
    im = ax.imshow(data, cmap='RdBu_r', aspect='equal')
    
    # Add text annotations
    for i in range(4):
        for j in range(4):
            text = ax.text(j, i, f'{data[i, j]:.3f}', ha="center", va="center", 
                          color="white" if abs(data[i, j]) > 0.5 else "black", fontweight='bold')
    
    ax.set_title(title)
    ax.set_xticks(range(4))
    ax.set_yticks(range(4))
    ax.set_xticklabels(['X', 'Y', 'Z', 'T'])
    ax.set_yticklabels(['X', 'Y', 'Z', '1'])
    
    # Add colorbar
    plt.colorbar(im, ax=ax, shrink=0.6)
    
    return im

def visualize_orientation(input_x, input_y, input_z):
    """Create comprehensive visualization of orientation calculation"""
    
    try:
        import delta_robot
        om = delta_robot.orientation
        km = delta_robot.kinematics
    except ImportError as e:
        print(f"Error: delta_robot package not found: {e}")
        return False
    
    # Calculate orientation result
    result = om.OrientationModule.calculate(input_x, input_y, input_z)
    
    # Also get kinematics for context
    kinematics_result = km.KinematicsModule.calculate(input_x, input_y, input_z)
    
    # Setup figure with subplots
    fig = plt.figure(figsize=(20, 16))
    
    # 1. UVW Frame at Fermat Point
    ax1 = fig.add_subplot(321, projection='3d')
    
    # Get ABC points from fermat data
    fp = result.fermat_point
    fermat_data = kinematics_result.fermat_data
    
    # Plot ABC triangle using actual module data
    try:
        # Get actual base positions from module
        import delta_robot
        base_A = delta_robot.get_base_position_A()
        base_B = delta_robot.get_base_position_B()
        base_C = delta_robot.get_base_position_C()
        
        # 3D points using actual Z intersections
        A_point = [base_A.x, base_A.y, fermat_data.z_A]
        B_point = [base_B.x, base_B.y, fermat_data.z_B]
        C_point = [base_C.x, base_C.y, fermat_data.z_C]
        
        # Plot triangle
        triangle_x = [A_point[0], B_point[0], C_point[0], A_point[0]]
        triangle_y = [A_point[1], B_point[1], C_point[1], A_point[1]]
        triangle_z = [A_point[2], B_point[2], C_point[2], A_point[2]]
        ax1.plot(triangle_x, triangle_y, triangle_z, 'k-', linewidth=2, label='Triangle ABC')
        
        ax1.scatter([A_point[0]], [A_point[1]], [A_point[2]], color='red', s=100, marker='^', label='Point A')
        ax1.scatter([B_point[0]], [B_point[1]], [B_point[2]], color='green', s=100, marker='^', label='Point B')
        ax1.scatter([C_point[0]], [C_point[1]], [C_point[2]], color='blue', s=100, marker='^', label='Point C')
    except Exception as e:
        print(f"Could not plot ABC triangle: {e}")
    
    # Plot Fermat point
    ax1.scatter([fp.x], [fp.y], [fp.z], color='gold', s=200, marker='*', label='Fermat Point')
    
    # Draw UVW coordinate frame at Fermat point
    uvw_frame = result.UVW_at_fermat
    draw_coordinate_frame(ax1, [fp.x, fp.y, fp.z], 
                         [uvw_frame.u_axis.x, uvw_frame.u_axis.y, uvw_frame.u_axis.z],
                         [uvw_frame.v_axis.x, uvw_frame.v_axis.y, uvw_frame.v_axis.z],
                         [uvw_frame.w_axis.x, uvw_frame.w_axis.y, uvw_frame.w_axis.z],
                         scale=8, label_prefix="UVW-")
    
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title('UVW Frame at Fermat Point')
    ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    # 2. Transformation Steps Overview
    ax2 = fig.add_subplot(322, projection='3d')
    
    # Show transformation sequence: UVW → IJK → U'V'W' → U''V''W''
    # We can't get intermediate frames directly, but we can show the progression
    
    # UVW at Fermat (step 1)
    ax2.scatter([fp.x], [fp.y], [fp.z], color='gold', s=150, marker='*', alpha=0.7, label='Fermat Point')
    draw_coordinate_frame(ax2, [fp.x, fp.y, fp.z], 
                         [uvw_frame.u_axis.x, uvw_frame.u_axis.y, uvw_frame.u_axis.z],
                         [uvw_frame.v_axis.x, uvw_frame.v_axis.y, uvw_frame.v_axis.z],
                         [uvw_frame.w_axis.x, uvw_frame.w_axis.y, uvw_frame.w_axis.z],
                         scale=6, alpha=0.5)
    
    # Final frame at end-effector (step 4)
    final_frame = result.final_frame
    ee_pos = result.end_effector_position
    ax2.scatter([ee_pos.x], [ee_pos.y], [ee_pos.z], color='red', s=200, marker='*', label='End-Effector')
    draw_coordinate_frame(ax2, [ee_pos.x, ee_pos.y, ee_pos.z], 
                         [final_frame.u_axis.x, final_frame.u_axis.y, final_frame.u_axis.z],
                         [final_frame.v_axis.x, final_frame.v_axis.y, final_frame.v_axis.z],
                         [final_frame.w_axis.x, final_frame.w_axis.y, final_frame.w_axis.z],
                         scale=8, alpha=1.0)
    
    # Show transformation path
    ax2.plot([fp.x, ee_pos.x], [fp.y, ee_pos.y], [fp.z, ee_pos.z], 
             'purple', linewidth=3, alpha=0.7, linestyle='--', label='Transformation Path')
    
    # Reference frames
    ax2.quiver(0, 0, 0, 10, 0, 0, color='red', alpha=0.3, linewidth=1, label='World X')
    ax2.quiver(0, 0, 0, 0, 10, 0, color='green', alpha=0.3, linewidth=1, label='World Y')
    ax2.quiver(0, 0, 0, 0, 0, 10, color='blue', alpha=0.3, linewidth=1, label='World Z')
    
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.set_title('Coordinate Frame Transformation')
    ax2.legend()
    
    # 3. Transformation Matrix Heatmap
    ax3 = fig.add_subplot(323)
    create_matrix_heatmap(ax3, result.transformation_matrix, 'Transformation Matrix')
    
    # 4. Frame Component Analysis
    ax4 = fig.add_subplot(324)
    
    # Analyze frame orientations
    frames = ['UVW U-axis', 'UVW V-axis', 'UVW W-axis', 'Final U-axis', 'Final V-axis', 'Final W-axis']
    
    # X components
    x_components = [uvw_frame.u_axis.x, uvw_frame.v_axis.x, uvw_frame.w_axis.x,
                   final_frame.u_axis.x, final_frame.v_axis.x, final_frame.w_axis.x]
    
    # Y components  
    y_components = [uvw_frame.u_axis.y, uvw_frame.v_axis.y, uvw_frame.w_axis.y,
                   final_frame.u_axis.y, final_frame.v_axis.y, final_frame.w_axis.y]
    
    # Z components
    z_components = [uvw_frame.u_axis.z, uvw_frame.v_axis.z, uvw_frame.w_axis.z,
                   final_frame.u_axis.z, final_frame.v_axis.z, final_frame.w_axis.z]
    
    x_pos = np.arange(len(frames))
    width = 0.25
    
    bars1 = ax4.bar(x_pos - width, x_components, width, label='X Component', color='red', alpha=0.7)
    bars2 = ax4.bar(x_pos, y_components, width, label='Y Component', color='green', alpha=0.7)
    bars3 = ax4.bar(x_pos + width, z_components, width, label='Z Component', color='blue', alpha=0.7)
    
    ax4.set_xlabel('Axis')
    ax4.set_ylabel('Component Value')
    ax4.set_title('Frame Axis Components')
    ax4.set_xticks(x_pos)
    ax4.set_xticklabels(frames, rotation=45, ha='right')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # 5. Orthonormality Verification
    ax5 = fig.add_subplot(325)
    
    # Check orthonormality of both frames
    def check_orthonormality(frame, name):
        u = np.array([frame.u_axis.x, frame.u_axis.y, frame.u_axis.z])
        v = np.array([frame.v_axis.x, frame.v_axis.y, frame.v_axis.z])
        w = np.array([frame.w_axis.x, frame.w_axis.y, frame.w_axis.z])
        
        # Magnitudes
        u_mag = np.linalg.norm(u)
        v_mag = np.linalg.norm(v)
        w_mag = np.linalg.norm(w)
        
        # Dot products (should be 0 for orthogonality)
        uv_dot = np.dot(u, v)
        uw_dot = np.dot(u, w)
        vw_dot = np.dot(v, w)
        
        return {
            'name': name,
            'u_mag': u_mag, 'v_mag': v_mag, 'w_mag': w_mag,
            'uv_dot': uv_dot, 'uw_dot': uw_dot, 'vw_dot': vw_dot
        }
    
    uvw_check = check_orthonormality(uvw_frame, 'UVW')
    final_check = check_orthonormality(final_frame, 'Final')
    
    # Plot orthonormality metrics
    metrics = ['U magnitude', 'V magnitude', 'W magnitude', 'U·V dot', 'U·W dot', 'V·W dot']
    uvw_values = [uvw_check['u_mag'], uvw_check['v_mag'], uvw_check['w_mag'],
                  abs(uvw_check['uv_dot']), abs(uvw_check['uw_dot']), abs(uvw_check['vw_dot'])]
    final_values = [final_check['u_mag'], final_check['v_mag'], final_check['w_mag'],
                   abs(final_check['uv_dot']), abs(final_check['uw_dot']), abs(final_check['vw_dot'])]
    
    x_pos = np.arange(len(metrics))
    width = 0.35
    
    bars1 = ax5.bar(x_pos - width/2, uvw_values, width, label='UVW Frame', alpha=0.7)
    bars2 = ax5.bar(x_pos + width/2, final_values, width, label='Final Frame', alpha=0.7)
    
    # Add reference lines
    ax5.axhline(y=1.0, color='green', linestyle='--', alpha=0.5, label='Target (1.0 for magnitudes)')
    ax5.axhline(y=0.0, color='red', linestyle='--', alpha=0.5, label='Target (0.0 for dot products)')
    
    ax5.set_xlabel('Metric')
    ax5.set_ylabel('Value')
    ax5.set_title('Orthonormality Verification')
    ax5.set_xticks(x_pos)
    ax5.set_xticklabels(metrics, rotation=45, ha='right')
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    ax5.set_yscale('log')  # Log scale to see small values
    
    # 6. Summary and Validation
    ax6 = fig.add_subplot(326)
    ax6.axis('off')
    
    # Matrix validation
    matrix = result.transformation_matrix
    
    # Check bottom row
    bottom_row_correct = (abs(matrix[3,0]) < 1e-6 and abs(matrix[3,1]) < 1e-6 and 
                         abs(matrix[3,2]) < 1e-6 and abs(matrix[3,3] - 1.0) < 1e-6)
    
    # Check translation part
    translation_correct = (abs(matrix[0,3] - ee_pos.x) < 1e-6 and 
                          abs(matrix[1,3] - ee_pos.y) < 1e-6 and 
                          abs(matrix[2,3] - ee_pos.z) < 1e-6)
    
    # Check rotation orthonormality
    rot_cols = []
    for j in range(3):
        col = np.array([matrix[i,j] for i in range(3)])
        rot_cols.append(col)
    
    rot_mags = [np.linalg.norm(col) for col in rot_cols]
    rot_dots = [abs(np.dot(rot_cols[i], rot_cols[j])) for i in range(3) for j in range(i+1, 3)]
    
    rotation_orthonormal = (all(abs(mag - 1.0) < 1e-6 for mag in rot_mags) and 
                           all(dot < 1e-6 for dot in rot_dots))
    
    summary_text = f"""
ORIENTATION MODULE RESULTS

Input Vector: ({input_x}, {input_y}, {input_z})

Coordinate Frames:
  UVW at Fermat: ({fp.x:.3f}, {fp.y:.3f}, {fp.z:.3f})
  Final at End-Effector: ({ee_pos.x:.3f}, {ee_pos.y:.3f}, {ee_pos.z:.3f})

Transformation Matrix Validation:
  ✓ Bottom row [0,0,0,1]: {"PASS" if bottom_row_correct else "FAIL"}
  ✓ Translation matches end-effector: {"PASS" if translation_correct else "FAIL"}
  ✓ Rotation part orthonormal: {"PASS" if rotation_orthonormal else "FAIL"}

Frame Orthonormality:
  UVW Frame:
    Magnitudes: U={uvw_check['u_mag']:.6f}, V={uvw_check['v_mag']:.6f}, W={uvw_check['w_mag']:.6f}
    Dot Products: U·V={uvw_check['uv_dot']:.6f}, U·W={uvw_check['uw_dot']:.6f}, V·W={uvw_check['vw_dot']:.6f}
  
  Final Frame:
    Magnitudes: U={final_check['u_mag']:.6f}, V={final_check['v_mag']:.6f}, W={final_check['w_mag']:.6f}
    Dot Products: U·V={final_check['uv_dot']:.6f}, U·W={final_check['uw_dot']:.6f}, V·W={final_check['vw_dot']:.6f}

Transformation Process:
  1. UVW frame created at Fermat point
     • U-axis: Fermat → A point
     • W-axis: Normal to ABC plane  
     • V-axis: W × U (right-hand rule)
  
  2. Mirror UVW across XY plane → IJK
  
  3. Translate to align IJK origin with world origin → U'V'W'
  
  4. Translate to end-effector position → U''V''W''

Status: {"✓ ALL VALIDATIONS PASSED" if all([bottom_row_correct, translation_correct, rotation_orthonormal]) else "✗ SOME VALIDATIONS FAILED"}
    """
    
    ax6.text(0.05, 0.95, summary_text, transform=ax6.transAxes, fontsize=9,
             verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='lightcyan', alpha=0.8))
    
    plt.tight_layout()
    plt.show()
    
    return True

def main():
    # Parse command line arguments for input vector
    if len(sys.argv) == 2:
        input_x, input_y, input_z = parse_coordinates(sys.argv[1])
        print(f"Using command line input: ({input_x}, {input_y}, {input_z})")
    else:
        input_x, input_y, input_z = 5, 4, 7
        print("Using default input: (5, 4, 7)")
        print("Usage: python3 test_orientation_visual.py x,y,z")
    
    print("Creating Orientation visualization...")
    success = visualize_orientation(input_x, input_y, input_z)
    
    if success:
        print("Visualization complete!")
        print("\nVisualization shows:")
        print("- UVW Frame: Coordinate system at Fermat point")
        print("- Transformation: Complete UVW→IJK→U'V'W'→U''V''W'' process")
        print("- Matrix Heatmap: Final 4x4 transformation matrix")
        print("- Component Analysis: Breakdown of frame axis components")
        print("- Orthonormality: Verification that frames are properly normalized")
        print("- Summary: Complete validation and transformation process")
    else:
        print("Visualization failed - check module installation")

if __name__ == "__main__":
    main()