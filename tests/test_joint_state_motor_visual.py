#!/usr/bin/env python3
"""
Clean Joint State Motor Test - Shows motor positions for all segments
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

def draw_coordinate_frame(ax, origin, u_axis, v_axis, w_axis, scale=50, colors=['red', 'green', 'blue'], label="", alpha=1.0):
    """Draw a coordinate frame with colored arrows"""
    # Draw axes
    ax.quiver(origin[0], origin[1], origin[2], 
              u_axis[0]*scale, u_axis[1]*scale, u_axis[2]*scale, 
              color=colors[0], arrow_length_ratio=0.1, linewidth=2, 
              label=f'{label}U' if label else 'U', alpha=alpha)
    
    ax.quiver(origin[0], origin[1], origin[2], 
              v_axis[0]*scale, v_axis[1]*scale, v_axis[2]*scale, 
              color=colors[1], arrow_length_ratio=0.1, linewidth=2, 
              label=f'{label}V' if label else 'V', alpha=alpha)
    
    ax.quiver(origin[0], origin[1], origin[2], 
              w_axis[0]*scale, w_axis[1]*scale, w_axis[2]*scale, 
              color=colors[2], arrow_length_ratio=0.1, linewidth=2, 
              label=f'{label}W' if label else 'W', alpha=alpha)

def create_motor_plot(result, target_x, target_y, target_z):
    """Create motor visualization plot"""
    
    # Create figure
    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # World XYZ coordinate frame at origin (RED, GREEN, BLUE)
    world_origin = [0, 0, 0]
    world_x = [1, 0, 0]
    world_y = [0, 1, 0] 
    world_z = [0, 0, 1]
    draw_coordinate_frame(ax, world_origin, world_x, world_y, world_z, 
                        scale=80, colors=['red', 'green', 'blue'], label="World ")
    
    # Target position (GOLD star)
    target_pos = [result.target_position.x, result.target_position.y, result.target_position.z]
    ax.scatter(*target_pos, color='gold', s=200, label='Target', marker='*')
    
    # Final achieved position (BLACK point)
    achieved_pos = [result.achieved_end_effector.x, result.achieved_end_effector.y, result.achieved_end_effector.z]
    ax.scatter(*achieved_pos, color='black', s=100, label='Final Achieved', marker='o')
    
    # Plot all segment positions and motor data
    colors = ['orange', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan', 'navy']
    
    for i, motor_data in enumerate(result.all_segment_motors):
        color = colors[i % len(colors)]
        
        # FABRIK position (original)
        fabrik_pos = [motor_data.fabrik_position.x, motor_data.fabrik_position.y, motor_data.fabrik_position.z]
        ax.scatter(*fabrik_pos, color=color, s=80, alpha=0.6, marker='o', label=f'Seg {motor_data.segment_number} FABRIK')
        
        # Local position (transformed)
        local_pos = [motor_data.local_position.x, motor_data.local_position.y, motor_data.local_position.z]
        ax.scatter(*local_pos, color=color, s=100, alpha=1.0, marker='s', label=f'Seg {motor_data.segment_number} Local')
        
        # Draw line between FABRIK and local positions if they differ
        distance = np.sqrt(sum((f-l)**2 for f,l in zip(fabrik_pos, local_pos)))
        if distance > 1.0:  # Only draw if significant transformation
            ax.plot([fabrik_pos[0], local_pos[0]], 
                   [fabrik_pos[1], local_pos[1]], 
                   [fabrik_pos[2], local_pos[2]], 
                   color=color, linestyle='--', alpha=0.8, linewidth=2)
        
        # Draw UVW coordinate frame at local position (smaller)
        uvw_u = [motor_data.uvw_frame.u_axis.x, motor_data.uvw_frame.u_axis.y, motor_data.uvw_frame.u_axis.z]
        uvw_v = [motor_data.uvw_frame.v_axis.x, motor_data.uvw_frame.v_axis.y, motor_data.uvw_frame.v_axis.z]
        uvw_w = [motor_data.uvw_frame.w_axis.x, motor_data.uvw_frame.w_axis.y, motor_data.uvw_frame.w_axis.z]
        
        # Only draw UVW frame for first few segments to avoid clutter
        if i < 3:
            draw_coordinate_frame(ax, local_pos, uvw_u, uvw_v, uvw_w,
                                scale=30, colors=['magenta', 'yellow', 'cyan'], alpha=0.7)
    
    # Connect segments with lines to show kinematic chain
    if len(result.all_segment_motors) > 1:
        fabrik_positions = []
        local_positions = []
        for motor_data in result.all_segment_motors:
            fabrik_positions.append([motor_data.fabrik_position.x, motor_data.fabrik_position.y, motor_data.fabrik_position.z])
            local_positions.append([motor_data.local_position.x, motor_data.local_position.y, motor_data.local_position.z])
        
        fabrik_positions = np.array(fabrik_positions)
        local_positions = np.array(local_positions)
        
        # FABRIK chain
        ax.plot(fabrik_positions[:, 0], fabrik_positions[:, 1], fabrik_positions[:, 2], 
               'k--', alpha=0.3, linewidth=1, label='FABRIK Chain')
        
        # Local chain  
        ax.plot(local_positions[:, 0], local_positions[:, 1], local_positions[:, 2], 
               'r-', alpha=0.8, linewidth=2, label='Sequential Chain')
    
    # Set axis properties
    all_positions = []
    for motor_data in result.all_segment_motors:
        all_positions.append([motor_data.fabrik_position.x, motor_data.fabrik_position.y, motor_data.fabrik_position.z])
        all_positions.append([motor_data.local_position.x, motor_data.local_position.y, motor_data.local_position.z])
    
    all_positions.extend([target_pos, achieved_pos])
    
    if all_positions:
        max_range = max(abs(coord) for pos in all_positions for coord in pos) * 1.2
        ax.set_xlim([-max_range/2, max_range/2])
        ax.set_ylim([-max_range/2, max_range/2])
        ax.set_zlim([0, max_range])
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Joint State Motor Result\nTarget: ({target_x}, {target_y}, {target_z})')
    
    # Legend (smaller)
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=8)
    
    # Add text info
    info_text = f"""Converged: {result.fabrik_converged}
Error: {result.fabrik_error:.6f}
Segments: {len(result.all_segment_motors)}
Time: {result.solve_time_ms:.2f}ms"""
    
    ax.text2D(0.02, 0.98, info_text, transform=ax.transAxes, 
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    return fig

def print_motor_summary(result):
    """Print summary of motor values"""
    
    print("\n" + "="*60)
    print("MOTOR SUMMARY")
    print("="*60)
    
    print(f"Target: ({result.target_position.x:.3f}, {result.target_position.y:.3f}, {result.target_position.z:.3f})")
    print(f"Achieved: ({result.achieved_end_effector.x:.3f}, {result.achieved_end_effector.y:.3f}, {result.achieved_end_effector.z:.3f})")
    print(f"Converged: {result.fabrik_converged}, Error: {result.fabrik_error:.6f}")
    print(f"Segments: {len(result.all_segment_motors)}")
    
    print(f"\nMOTOR VALUES:")
    print("-" * 60)
    for motor_data in result.all_segment_motors:
        print(f"Segment {motor_data.segment_number}:")
        print(f"  FABRIK Position: ({motor_data.fabrik_position.x:.3f}, {motor_data.fabrik_position.y:.3f}, {motor_data.fabrik_position.z:.3f})")
        print(f"  Local Position:  ({motor_data.local_position.x:.3f}, {motor_data.local_position.y:.3f}, {motor_data.local_position.z:.3f})")
        
        # Calculate transformation distance
        fabrik_pos = motor_data.fabrik_position
        local_pos = motor_data.local_position
        distance = ((fabrik_pos.x - local_pos.x)**2 + (fabrik_pos.y - local_pos.y)**2 + (fabrik_pos.z - local_pos.z)**2)**0.5
        print(f"  Transformation Distance: {distance:.3f}")
        
        print(f"  Base Motors: z_A={motor_data.z_A:.3f}, z_B={motor_data.z_B:.3f}, z_C={motor_data.z_C:.3f}")
        print(f"  Joint Motors: prismatic={motor_data.prismatic_joint:.3f}, roll={motor_data.roll_joint*180/3.14159:.1f}°, pitch={motor_data.pitch_joint*180/3.14159:.1f}°")

def main():
    # Parse command line arguments for target
    if len(sys.argv) == 2:
        target_x, target_y, target_z = parse_coordinates(sys.argv[1])
        print(f"Target: ({target_x}, {target_y}, {target_z})")
    else:
        target_x, target_y, target_z = 100, 50, 300
        print(f"Default target: ({target_x}, {target_y}, {target_z})")
        print("Usage: python3 test_joint_state_motor_clean.py x,y,z")
    
    try:
        import delta_robot.joint_state_motor as motor
    except ImportError as e:
        print(f"Error: {e}")
        print("Build the modules first: python setup.py build_ext --inplace")
        return
    
    print("=" * 60)
    print("CLEAN JOINT STATE MOTOR TEST")
    print("=" * 60)
    
    # Calculate motor positions
    result = motor.JointStateMotorModule.calculate_motors(target_x, target_y, target_z)
    
    # Print summary
    print_motor_summary(result)
    
    # Create visualization
    print("\n" + "="*60)
    print("Creating 3D visualization...")
    print("="*60)
    
    fig = create_motor_plot(result, target_x, target_y, target_z)
    
    # Show plot
    plt.show()

if __name__ == "__main__":
    main()