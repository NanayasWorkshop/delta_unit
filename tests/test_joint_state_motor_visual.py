#!/usr/bin/env python3
"""
Simple Single Segment Test - Just first segment, no transformations
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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

def draw_coordinate_frame(ax, origin, u_axis, v_axis, w_axis, scale=50, colors=['red', 'green', 'blue'], label="", alpha=1.0, linewidth=2):
    """Draw a coordinate frame with colored arrows"""
    ax.quiver(origin[0], origin[1], origin[2], 
              u_axis[0]*scale, u_axis[1]*scale, u_axis[2]*scale, 
              color=colors[0], arrow_length_ratio=0.1, linewidth=linewidth, 
              label=f'{label}U' if label else 'U', alpha=alpha)
    
    ax.quiver(origin[0], origin[1], origin[2], 
              v_axis[0]*scale, v_axis[1]*scale, v_axis[2]*scale, 
              color=colors[1], arrow_length_ratio=0.1, linewidth=linewidth, 
              label=f'{label}V' if label else 'V', alpha=alpha)
    
    ax.quiver(origin[0], origin[1], origin[2], 
              w_axis[0]*scale, w_axis[1]*scale, w_axis[2]*scale, 
              color=colors[2], arrow_length_ratio=0.1, linewidth=linewidth, 
              label=f'{label}W' if label else 'W', alpha=alpha)

def create_simple_plot(result, target_x, target_y, target_z):
    """Create simple visualization for single segment"""
    
    # Create figure
    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # World XYZ coordinate frame at origin (RED, GREEN, BLUE)
    world_origin = [0, 0, 0]
    world_x = [1, 0, 0]
    world_y = [0, 1, 0] 
    world_z = [0, 0, 1]
    draw_coordinate_frame(ax, world_origin, world_x, world_y, world_z, 
                        scale=80, colors=['red', 'green', 'blue'], label="World ", linewidth=3)
    
    # Target position (GOLD star)
    target_pos = [result.target_position.x, result.target_position.y, result.target_position.z]
    ax.scatter(*target_pos, color='gold', s=300, label='Target', marker='*', edgecolor='orange', linewidth=2)
    
    # Final achieved position (BLACK point)
    achieved_pos = [result.achieved_end_effector.x, result.achieved_end_effector.y, result.achieved_end_effector.z]
    ax.scatter(*achieved_pos, color='black', s=150, label='Final Achieved', marker='o', edgecolor='white', linewidth=2)
    
    # First segment only
    if len(result.all_segment_motors) > 0:
        motor_data = result.all_segment_motors[0]
        
        # Segment position (ORANGE square)
        segment_pos = [motor_data.fabrik_position.x, motor_data.fabrik_position.y, motor_data.fabrik_position.z]
        ax.scatter(*segment_pos, color='orange', s=200, label=f'Segment {motor_data.segment_number}', 
                  marker='s', edgecolor='black', linewidth=2)
        
        # UVW coordinate frame at segment position
        uvw_u = [motor_data.uvw_frame.u_axis.x, motor_data.uvw_frame.u_axis.y, motor_data.uvw_frame.u_axis.z]
        uvw_v = [motor_data.uvw_frame.v_axis.x, motor_data.uvw_frame.v_axis.y, motor_data.uvw_frame.v_axis.z]
        uvw_w = [motor_data.uvw_frame.w_axis.x, motor_data.uvw_frame.w_axis.y, motor_data.uvw_frame.w_axis.z]
        
        draw_coordinate_frame(ax, segment_pos, uvw_u, uvw_v, uvw_w,
                            scale=60, colors=['magenta', 'yellow', 'cyan'], label="UVW ", alpha=0.8, linewidth=2)
        
        # UVW frame origin (if different from segment position)
        uvw_origin = [motor_data.uvw_frame.origin.x, motor_data.uvw_frame.origin.y, motor_data.uvw_frame.origin.z]
        distance = np.sqrt(sum((s-u)**2 for s,u in zip(segment_pos, uvw_origin)))
        
        if distance > 1.0:  # If origins are different
            ax.scatter(*uvw_origin, color='purple', s=100, label='UVW Origin', marker='v', alpha=0.8)
            # Draw line connecting them
            ax.plot([segment_pos[0], uvw_origin[0]], 
                   [segment_pos[1], uvw_origin[1]], 
                   [segment_pos[2], uvw_origin[2]], 
                   color='purple', linestyle='--', alpha=0.6, linewidth=2, label='Position Mismatch')
        
        # Add segment label
        ax.text(segment_pos[0], segment_pos[1], segment_pos[2] + 30, f'S1', 
                fontsize=14, fontweight='bold', ha='center')
    
    # Show all FABRIK segments for reference
    if len(result.fabrik_result.segment_end_effectors) > 0:
        for i, seg_data in enumerate(result.fabrik_result.segment_end_effectors):
            pos = [seg_data.end_effector_position.x, seg_data.end_effector_position.y, seg_data.end_effector_position.z]
            
            if i == 0:
                continue  # Skip first segment (already shown)
            
            # Show other segments as small gray dots
            ax.scatter(*pos, color='gray', s=50, alpha=0.5, marker='o')
            ax.text(pos[0], pos[1], pos[2] + 10, f'S{i+1}', 
                    fontsize=8, ha='center', alpha=0.7)
        
        # Connect all FABRIK segments with line
        fabrik_positions = [[seg.end_effector_position.x, seg.end_effector_position.y, seg.end_effector_position.z] 
                           for seg in result.fabrik_result.segment_end_effectors]
        fabrik_array = np.array(fabrik_positions)
        ax.plot(fabrik_array[:, 0], fabrik_array[:, 1], fabrik_array[:, 2], 
               'k--', alpha=0.3, linewidth=1, label='FABRIK Chain')
    
    # Set axis properties
    all_positions = [target_pos, achieved_pos]
    if len(result.all_segment_motors) > 0:
        segment_pos = [result.all_segment_motors[0].fabrik_position.x, 
                      result.all_segment_motors[0].fabrik_position.y, 
                      result.all_segment_motors[0].fabrik_position.z]
        all_positions.append(segment_pos)
    
    if all_positions:
        max_range = max(abs(coord) for pos in all_positions for coord in pos) * 1.3
        ax.set_xlim([-max_range/2, max_range/2])
        ax.set_ylim([-max_range/2, max_range/2])
        ax.set_zlim([0, max_range])
    
    ax.set_xlabel('X', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y', fontsize=12, fontweight='bold')
    ax.set_zlabel('Z', fontsize=12, fontweight='bold')
    ax.set_title(f'Single Segment Analysis\nTarget: ({target_x}, {target_y}, {target_z})', 
                 fontsize=14, fontweight='bold')
    
    # Legend
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=10)
    
    # Add detailed info text
    if len(result.all_segment_motors) > 0:
        motor_data = result.all_segment_motors[0]
        
        info_text = f"""FABRIK Results:
Converged: {result.fabrik_converged}
Error: {result.fabrik_error:.6f}
Iterations: {result.fabrik_iterations}
Time: {result.solve_time_ms:.2f}ms

Segment 1 Motors:
z_A: {motor_data.z_A:.3f}
z_B: {motor_data.z_B:.3f}
z_C: {motor_data.z_C:.3f}
Prismatic: {motor_data.prismatic_joint:.3f}
Roll: {motor_data.roll_joint*180/3.14159:.1f}°
Pitch: {motor_data.pitch_joint*180/3.14159:.1f}°

UVW Frame:
Origin: ({motor_data.uvw_frame.origin.x:.1f}, {motor_data.uvw_frame.origin.y:.1f}, {motor_data.uvw_frame.origin.z:.1f})

Position Match:
Segment: ({motor_data.fabrik_position.x:.1f}, {motor_data.fabrik_position.y:.1f}, {motor_data.fabrik_position.z:.1f})
UVW Origin: ({motor_data.uvw_frame.origin.x:.1f}, {motor_data.uvw_frame.origin.y:.1f}, {motor_data.uvw_frame.origin.z:.1f})
Distance: {((motor_data.fabrik_position.x - motor_data.uvw_frame.origin.x)**2 + (motor_data.fabrik_position.y - motor_data.uvw_frame.origin.y)**2 + (motor_data.fabrik_position.z - motor_data.uvw_frame.origin.z)**2)**0.5:.3f}"""
        
        ax.text2D(0.02, 0.98, info_text, transform=ax.transAxes, 
                 verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9),
                 fontsize=9, family='monospace')
    
    plt.tight_layout()
    return fig

def print_simple_summary(result):
    """Print simple summary for single segment"""
    
    print("\n" + "="*60)
    print("SIMPLE SINGLE SEGMENT SUMMARY")
    print("="*60)
    
    print(f"Target: ({result.target_position.x:.3f}, {result.target_position.y:.3f}, {result.target_position.z:.3f})")
    print(f"Achieved: ({result.achieved_end_effector.x:.3f}, {result.achieved_end_effector.y:.3f}, {result.achieved_end_effector.z:.3f})")
    print(f"FABRIK Converged: {result.fabrik_converged}, Error: {result.fabrik_error:.6f}")
    
    if len(result.all_segment_motors) > 0:
        motor_data = result.all_segment_motors[0]
        
        print(f"\nSegment 1 Analysis:")
        print(f"Position: ({motor_data.fabrik_position.x:.3f}, {motor_data.fabrik_position.y:.3f}, {motor_data.fabrik_position.z:.3f})")
        print(f"Motors: z_A={motor_data.z_A:.3f}, z_B={motor_data.z_B:.3f}, z_C={motor_data.z_C:.3f}")
        print(f"Joints: prismatic={motor_data.prismatic_joint:.3f}, roll={motor_data.roll_joint*180/3.14159:.1f}°, pitch={motor_data.pitch_joint*180/3.14159:.1f}°")
        
        # UVW frame analysis
        uvw = motor_data.uvw_frame
        print(f"\nUVW Frame Analysis:")
        print(f"Origin: ({uvw.origin.x:.3f}, {uvw.origin.y:.3f}, {uvw.origin.z:.3f})")
        print(f"U-axis: ({uvw.u_axis.x:.3f}, {uvw.u_axis.y:.3f}, {uvw.u_axis.z:.3f})")
        print(f"V-axis: ({uvw.v_axis.x:.3f}, {uvw.v_axis.y:.3f}, {uvw.v_axis.z:.3f})")
        print(f"W-axis: ({uvw.w_axis.x:.3f}, {uvw.w_axis.y:.3f}, {uvw.w_axis.z:.3f})")
        
        # Position match check
        segment_pos = motor_data.fabrik_position
        uvw_origin = uvw.origin
        distance = ((segment_pos.x - uvw_origin.x)**2 + 
                   (segment_pos.y - uvw_origin.y)**2 + 
                   (segment_pos.z - uvw_origin.z)**2)**0.5
        
        print(f"\nPosition Match Check:")
        print(f"Segment Position: ({segment_pos.x:.3f}, {segment_pos.y:.3f}, {segment_pos.z:.3f})")
        print(f"UVW Origin:       ({uvw_origin.x:.3f}, {uvw_origin.y:.3f}, {uvw_origin.z:.3f})")
        print(f"Distance:         {distance:.3f}")
        
        if distance < 0.001:
            print("✓ UVW frame origin matches segment position perfectly")
        elif distance < 1.0:
            print("⚠ UVW frame origin close to segment position")
        else:
            print("❌ UVW frame origin significantly different from segment position")
    
    else:
        print("No motor data calculated")

def main():
    # Parse command line arguments for target
    if len(sys.argv) == 2:
        target_x, target_y, target_z = parse_coordinates(sys.argv[1])
        print(f"Target: ({target_x}, {target_y}, {target_z})")
    else:
        target_x, target_y, target_z = 100, 50, 300
        print(f"Default target: ({target_x}, {target_y}, {target_z})")
        print("Usage: python3 test_single_segment_simple.py x,y,z")
    
    try:
        import delta_robot.joint_state_motor as motor
    except ImportError as e:
        print(f"Error: {e}")
        print("Build the modules first: python setup.py build_ext --inplace")
        return
    
    print("=" * 60)
    print("SIMPLE SINGLE SEGMENT TEST")
    print("=" * 60)
    
    # Calculate motor positions (single segment only)
    result = motor.JointStateMotorModule.calculate_motors(target_x, target_y, target_z)
    
    # Print simple summary
    print_simple_summary(result)
    
    # Create simple visualization
    print("\n" + "="*60)
    print("Creating simple 3D visualization for single segment...")
    print("="*60)
    
    fig = create_simple_plot(result, target_x, target_y, target_z)
    
    # Show plot
    plt.show()

if __name__ == "__main__":
    main()