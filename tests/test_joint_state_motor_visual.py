#!/usr/bin/env python3
"""
Visual Joint State Motor Test - Shows 3D visualization with coordinate frames
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def parse_coordinates(coord_str):
    """Parse coordinate string like '545,554,122' into x,y,z values"""
    try:
        coords = [float(x.strip()) for x in coord_str.split(',')]
        if len(coords) != 3:
            raise ValueError("Expected 3 coordinates")
        return coords
    except ValueError as e:
        print(f"Error parsing coordinates '{coord_str}': {e}")
        print("Expected format: x,y,z (e.g., 545,554,122)")
        sys.exit(1)

def draw_coordinate_frame(ax, origin, u_axis, v_axis, w_axis, scale=50, colors=['red', 'green', 'blue'], label=""):
    """Draw a coordinate frame with colored arrows"""
    # Draw axes
    ax.quiver(origin[0], origin[1], origin[2], 
              u_axis[0]*scale, u_axis[1]*scale, u_axis[2]*scale, 
              color=colors[0], arrow_length_ratio=0.1, linewidth=2, label=f'{label}X' if label else 'X')
    
    ax.quiver(origin[0], origin[1], origin[2], 
              v_axis[0]*scale, v_axis[1]*scale, v_axis[2]*scale, 
              color=colors[1], arrow_length_ratio=0.1, linewidth=2, label=f'{label}Y' if label else 'Y')
    
    ax.quiver(origin[0], origin[1], origin[2], 
              w_axis[0]*scale, w_axis[1]*scale, w_axis[2]*scale, 
              color=colors[2], arrow_length_ratio=0.1, linewidth=2, label=f'{label}Z' if label else 'Z')

def main():
    # Parse command line arguments for target
    if len(sys.argv) == 2:
        target_x, target_y, target_z = parse_coordinates(sys.argv[1])
        print(f"Target: ({target_x}, {target_y}, {target_z})")
    else:
        target_x, target_y, target_z = 100, 50, 300
        print(f"Default target: ({target_x}, {target_y}, {target_z})")
        print("Usage: python3 test_joint_state_motor_visual.py x,y,z")
    
    try:
        import delta_robot.joint_state_motor as motor
        import delta_robot.kinematics_module as kinematics
        import delta_robot.orientation_module as orientation
        import delta_robot.delta_types as dt
    except ImportError as e:
        print(f"Error: {e}")
        print("Build the modules first: python setup.py build_ext --inplace")
        return
    
    print("=" * 50)
    
    # Calculate motor positions
    result = motor.JointStateMotorModule.calculate_motors(target_x, target_y, target_z)
    
    print("=" * 50)
    print("RESULTS:")
    print(f"Target:    ({result.target_position.x:.3f}, {result.target_position.y:.3f}, {result.target_position.z:.3f})")
    print(f"Achieved:  ({result.achieved_end_effector.x:.3f}, {result.achieved_end_effector.y:.3f}, {result.achieved_end_effector.z:.3f})")
    print(f"Converged: {'YES' if result.fabrik_converged else 'NO'}")
    print(f"Error:     {result.fabrik_error:.6f}")
    
    # Get first segment end-effector and run through kinematics + orientation
    if result.fabrik_result.segment_end_effectors:
        first_segment = result.fabrik_result.segment_end_effectors[0]
        seg_pos = first_segment.end_effector_position
        
        print("=" * 50)
        print("FIRST SEGMENT ANALYSIS:")
        print(f"Segment 1 End-Effector: ({seg_pos.x:.3f}, {seg_pos.y:.3f}, {seg_pos.z:.3f})")
        
        # Run through kinematics module
        kinematics_result = kinematics.KinematicsModule.calculate(seg_pos.x, seg_pos.y, seg_pos.z)
        
        print(f"\nA B C Z Values:")
        print(f"  z_A: {kinematics_result.fermat_data.z_A:.6f}")
        print(f"  z_B: {kinematics_result.fermat_data.z_B:.6f}")
        print(f"  z_C: {kinematics_result.fermat_data.z_C:.6f}")
        
        print(f"\nJoint States:")
        print(f"  Prismatic: {kinematics_result.joint_state_data.prismatic_joint:.6f}")
        print(f"  Roll:      {kinematics_result.joint_state_data.roll_joint:.6f} rad ({kinematics_result.joint_state_data.roll_joint * 180 / 3.14159:.2f}°)")
        print(f"  Pitch:     {kinematics_result.joint_state_data.pitch_joint:.6f} rad ({kinematics_result.joint_state_data.pitch_joint * 180 / 3.14159:.2f}°)")
        
        # Run through orientation module
        orientation_result = orientation.OrientationModule.calculate_from_kinematics(kinematics_result)
        
        print(f"\nEnd-Effector UVW Orientation:")
        final_frame = orientation_result.final_frame
        print(f"  U-axis: ({final_frame.u_axis.x:.6f}, {final_frame.u_axis.y:.6f}, {final_frame.u_axis.z:.6f})")
        print(f"  V-axis: ({final_frame.v_axis.x:.6f}, {final_frame.v_axis.y:.6f}, {final_frame.v_axis.z:.6f})")
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
        end_eff_pos = [seg_pos.x, seg_pos.y, seg_pos.z]
        ax.scatter(*end_eff_pos, color='black', s=100, label='End-Effector', marker='o')
        
        # End-effector UVW frame (MAGENTA, YELLOW, CYAN)
        uvw_u = [final_frame.u_axis.x, final_frame.u_axis.y, final_frame.u_axis.z]
        uvw_v = [final_frame.v_axis.x, final_frame.v_axis.y, final_frame.v_axis.z]
        uvw_w = [final_frame.w_axis.x, final_frame.w_axis.y, final_frame.w_axis.z]
        draw_coordinate_frame(ax, end_eff_pos, uvw_u, uvw_v, uvw_w,
                            scale=60, colors=['magenta', 'yellow', 'cyan'], label="UVW ")
        
        # Calculate and show H and G points
        # H point: Joint H at (0, 0, WORKING_HEIGHT)
        WORKING_HEIGHT = 11.5  # From constants
        MIN_HEIGHT = 101.0     # From constants  
        MOTOR_LIMIT = 11.0     # From constants
        
        joint_H = [0, 0, WORKING_HEIGHT]
        ax.scatter(*joint_H, color='orange', s=80, label='Joint H', marker='s')
        
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
        
        # Target position
        target_pos = [result.target_position.x, result.target_position.y, result.target_position.z]
        ax.scatter(*target_pos, color='gold', s=150, label='Target', marker='X')
        
        # Set axis properties
        max_range = max(abs(coord) for pos in [end_eff_pos, target_pos, point_G] for coord in pos) * 1.2
        ax.set_xlim([-max_range/2, max_range/2])
        ax.set_ylim([-max_range/2, max_range/2])
        ax.set_zlim([0, max_range])
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f'Delta Robot Joint State Visualization\nTarget: ({target_x}, {target_y}, {target_z})')
        
        # Legend
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        
        # Add text info
        info_text = f"""Target: ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})
End-Effector: ({seg_pos.x:.1f}, {seg_pos.y:.1f}, {seg_pos.z:.1f})
Roll: {kinematics_result.joint_state_data.roll_joint * 180 / 3.14159:.1f}°
Pitch: {kinematics_result.joint_state_data.pitch_joint * 180 / 3.14159:.1f}°
Prismatic: {prismatic_length:.3f}"""
        
        ax.text2D(0.02, 0.98, info_text, transform=ax.transAxes, 
                 verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        plt.tight_layout()
        plt.show()
    
    print("=" * 50)

if __name__ == "__main__":
    main()