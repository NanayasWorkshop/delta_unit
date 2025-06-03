#!/usr/bin/env python3
"""
Simple Visual Orientation Test - Just one 3D plot with coordinate frames
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

def draw_coordinate_frame(ax, origin, u_axis, v_axis, w_axis, scale=50, colors=['red', 'green', 'blue'], label=""):
    """Draw a coordinate frame with colored arrows"""
    # Draw axes
    ax.quiver(origin[0], origin[1], origin[2], 
              u_axis[0]*scale, u_axis[1]*scale, u_axis[2]*scale, 
              color=colors[0], arrow_length_ratio=0.1, linewidth=2, label=f'{label}U' if label else 'U')
    
    ax.quiver(origin[0], origin[1], origin[2], 
              v_axis[0]*scale, v_axis[1]*scale, v_axis[2]*scale, 
              color=colors[1], arrow_length_ratio=0.1, linewidth=2, label=f'{label}V' if label else 'V')
    
    ax.quiver(origin[0], origin[1], origin[2], 
              w_axis[0]*scale, w_axis[1]*scale, w_axis[2]*scale, 
              color=colors[2], arrow_length_ratio=0.1, linewidth=2, label=f'{label}W' if label else 'W')

def main():
    # Parse command line arguments for input vector
    if len(sys.argv) == 2:
        input_x, input_y, input_z = parse_coordinates(sys.argv[1])
        print(f"Input: ({input_x}, {input_y}, {input_z})")
    else:
        input_x, input_y, input_z = 5, 4, 7
        print(f"Default input: ({input_x}, {input_y}, {input_z})")
        print("Usage: python3 test_orientation_visual_simple.py x,y,z")
    
    try:
        import delta_robot.orientation_module as orientation
        import delta_robot.kinematics_module as kinematics
        import delta_robot.delta_types as dt
    except ImportError as e:
        print(f"Error: {e}")
        print("Build the modules first: python setup.py build_ext --inplace")
        return
    
    print("=" * 50)
    
    # Calculate orientation and kinematics
    orientation_result = orientation.OrientationModule.calculate(input_x, input_y, input_z)
    kinematics_result = kinematics.KinematicsModule.calculate(input_x, input_y, input_z)
    
    print("ORIENTATION RESULTS:")
    print(f"End-Effector: ({orientation_result.end_effector_position.x:.3f}, {orientation_result.end_effector_position.y:.3f}, {orientation_result.end_effector_position.z:.3f})")
    print(f"Fermat Point: ({orientation_result.fermat_point.x:.3f}, {orientation_result.fermat_point.y:.3f}, {orientation_result.fermat_point.z:.3f})")
    
    final_frame = orientation_result.final_frame
    print(f"UVW Orientation:")
    print(f"  U-axis: ({final_frame.u_axis.x:.6f}, {final_frame.u_axis.y:.6f}, {final_frame.u_axis.z:.6f})")
    print(f"  V-axis: ({final_frame.v_axis.x:.6f}, {final_frame.v_axis.y:.6f}, {final_frame.w_axis.z:.6f})")
    print(f"  W-axis: ({final_frame.w_axis.x:.6f}, {final_frame.w_axis.y:.6f}, {final_frame.w_axis.z:.6f})")
    
    # === VISUALIZATION ===
    print("=" * 50)
    print("Creating 3D visualization...")
    
    # Create 3D plot
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # World XYZ coordinate frame at origin (RED, GREEN, BLUE)
    world_origin = [0, 0, 0]
    world_x = [1, 0, 0]
    world_y = [0, 1, 0] 
    world_z = [0, 0, 1]
    draw_coordinate_frame(ax, world_origin, world_x, world_y, world_z, 
                        scale=80, colors=['red', 'green', 'blue'], label="World ")
    
    # End-effector position (BLACK point)
    end_eff_pos = [orientation_result.end_effector_position.x, 
                   orientation_result.end_effector_position.y, 
                   orientation_result.end_effector_position.z]
    ax.scatter(*end_eff_pos, color='black', s=100, label='End-Effector', marker='o')
    
    # End-effector UVW frame (MAGENTA, YELLOW, CYAN)
    uvw_u = [final_frame.u_axis.x, final_frame.u_axis.y, final_frame.u_axis.z]
    uvw_v = [final_frame.v_axis.x, final_frame.v_axis.y, final_frame.v_axis.z]
    uvw_w = [final_frame.w_axis.x, final_frame.w_axis.y, final_frame.w_axis.z]
    draw_coordinate_frame(ax, end_eff_pos, uvw_u, uvw_v, uvw_w,
                        scale=60, colors=['magenta', 'yellow', 'cyan'], label="UVW ")
    
    # Fermat point (ORANGE point)
    fermat_pos = [orientation_result.fermat_point.x, 
                  orientation_result.fermat_point.y, 
                  orientation_result.fermat_point.z]
    ax.scatter(*fermat_pos, color='orange', s=80, label='Fermat Point', marker='s')
    
    # UVW frame at Fermat point (same colors but smaller, alpha)
    uvw_frame_fermat = orientation_result.UVW_at_fermat
    uvw_u_fermat = [uvw_frame_fermat.u_axis.x, uvw_frame_fermat.u_axis.y, uvw_frame_fermat.u_axis.z]
    uvw_v_fermat = [uvw_frame_fermat.v_axis.x, uvw_frame_fermat.v_axis.y, uvw_frame_fermat.v_axis.z]
    uvw_w_fermat = [uvw_frame_fermat.w_axis.x, uvw_frame_fermat.w_axis.y, uvw_frame_fermat.w_axis.z]
    
    # Draw smaller UVW frame at Fermat (with alpha for transparency)
    ax.quiver(fermat_pos[0], fermat_pos[1], fermat_pos[2], 
              uvw_u_fermat[0]*40, uvw_u_fermat[1]*40, uvw_u_fermat[2]*40, 
              color='magenta', arrow_length_ratio=0.1, linewidth=1, alpha=0.6)
    ax.quiver(fermat_pos[0], fermat_pos[1], fermat_pos[2], 
              uvw_v_fermat[0]*40, uvw_v_fermat[1]*40, uvw_v_fermat[2]*40, 
              color='yellow', arrow_length_ratio=0.1, linewidth=1, alpha=0.6)
    ax.quiver(fermat_pos[0], fermat_pos[1], fermat_pos[2], 
              uvw_w_fermat[0]*40, uvw_w_fermat[1]*40, uvw_w_fermat[2]*40, 
              color='cyan', arrow_length_ratio=0.1, linewidth=1, alpha=0.6)
    
    # Calculate and show H and G points (from kinematics)
    # H point: Joint H at (0, 0, WORKING_HEIGHT)
    WORKING_HEIGHT = 11.5  # From constants
    MIN_HEIGHT = 101.0     # From constants  
    MOTOR_LIMIT = 11.0     # From constants
    
    joint_H = [0, 0, WORKING_HEIGHT]
    ax.scatter(*joint_H, color='gold', s=80, label='Joint H', marker='^')
    
    # G point: Calculate from kinematics
    prismatic_length = kinematics_result.joint_state_data.prismatic_joint
    vector_length = MIN_HEIGHT + 2 * MOTOR_LIMIT + prismatic_length
    
    # Direction from original input
    original_input = kinematics_result.original_input
    input_norm = np.sqrt(original_input.x**2 + original_input.y**2 + original_input.z**2)
    normalized_dir = [original_input.x/input_norm, original_input.y/input_norm, original_input.z/input_norm]
    
    point_G = [joint_H[0] + normalized_dir[0] * vector_length,
               joint_H[1] + normalized_dir[1] * vector_length, 
               joint_H[2] + normalized_dir[2] * vector_length]
    ax.scatter(*point_G, color='purple', s=80, label='Point G', marker='^')
    
    # Draw H-G line
    ax.plot([joint_H[0], point_G[0]], 
            [joint_H[1], point_G[1]], 
            [joint_H[2], point_G[2]], 
            'purple', linewidth=2, alpha=0.7, label='H→G Vector')
    
    # Set axis properties
    max_range = max(abs(coord) for pos in [end_eff_pos, fermat_pos, point_G] for coord in pos) * 1.2
    ax.set_xlim([-max_range/2, max_range/2])
    ax.set_ylim([-max_range/2, max_range/2])
    ax.set_zlim([0, max_range])
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Delta Robot Orientation Visualization\nInput: ({input_x}, {input_y}, {input_z})')
    
    # Legend
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    # Add text info
    info_text = f"""Input: ({input_x:.1f}, {input_y:.1f}, {input_z:.1f})
End-Effector: ({end_eff_pos[0]:.1f}, {end_eff_pos[1]:.1f}, {end_eff_pos[2]:.1f})
Fermat Point: ({fermat_pos[0]:.1f}, {fermat_pos[1]:.1f}, {fermat_pos[2]:.1f})
Prismatic: {prismatic_length:.3f}"""
    
    ax.text2D(0.02, 0.98, info_text, transform=ax.transAxes, 
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    plt.show()
    
    print("=" * 50)

if __name__ == "__main__":
    main()