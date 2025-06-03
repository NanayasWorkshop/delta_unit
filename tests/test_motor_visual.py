#!/usr/bin/env python3
"""
Motor Module Visual Test - Minimal visualization of FABRIK segments and first segment analysis
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

def draw_coordinate_frame(ax, origin, u_axis, v_axis, w_axis, scale=5.0, colors=['red', 'green', 'blue'], label=""):
    """Draw a coordinate frame with colored arrows"""
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
    # Parse command line arguments for target position
    if len(sys.argv) == 2:
        target_x, target_y, target_z = parse_coordinates(sys.argv[1])
        print(f"Target: ({target_x}, {target_y}, {target_z})")
    else:
        target_x, target_y, target_z = 100, 50, 300
        print(f"Default target: ({target_x}, {target_y}, {target_z})")
        print("Usage: python3 test_motor_visual.py x,y,z")
    
    print("=" * 50)
    
    # Test using the motor module
    try:
        import delta_robot
        result = delta_robot.calculate_motors(target_x, target_y, target_z)
        
        if result is None:
            print("Motor module not available")
            return
        
        # Print segment positions
        print("SEGMENT POSITIONS:")
        for i, (seg_num, pos) in enumerate(zip(result.segment_numbers, result.segment_positions)):
            print(f"Segment {seg_num}: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
        
        # Print first segment kinematics data
        if result.segment_positions:
            print(f"\nFIRST SEGMENT KINEMATICS:")
            print(f"Position: ({result.first_segment_position.x:.3f}, {result.first_segment_position.y:.3f}, {result.first_segment_position.z:.3f})")
            print(f"Motors: z_A={result.z_A:.3f}, z_B={result.z_B:.3f}, z_C={result.z_C:.3f}")
            print(f"Joints: prismatic={result.prismatic_joint:.3f}, roll={result.roll_joint:.1f}°, pitch={result.pitch_joint:.1f}°")
            
            # Print first segment orientation data (Final UVW)
            print(f"\nFIRST SEGMENT ORIENTATION (Final UVW):")
            print(f"Origin: ({result.uvw_origin.x:.3f}, {result.uvw_origin.y:.3f}, {result.uvw_origin.z:.3f})")
            print(f"U-axis: ({result.uvw_u_axis.x:.3f}, {result.uvw_u_axis.y:.3f}, {result.uvw_u_axis.z:.3f})")
            print(f"V-axis: ({result.uvw_v_axis.x:.3f}, {result.uvw_v_axis.y:.3f}, {result.uvw_v_axis.z:.3f})")
            print(f"W-axis: ({result.uvw_w_axis.x:.3f}, {result.uvw_w_axis.y:.3f}, {result.uvw_w_axis.z:.3f})")
        
        # === VISUALIZATION ===
        print("\n" + "=" * 50)
        print("Creating motor module visualization...")
        
        # Create 3D plot
        fig = plt.figure(figsize=(15, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot target position (GOLD)
        ax.scatter([target_x], [target_y], [target_z], color='gold', s=300, 
                  marker='X', label='Target', edgecolor='black', linewidth=2)
        
        # Plot base at origin (BLACK)
        ax.scatter([0], [0], [0], color='black', s=200, 
                  marker='s', label='Base (0,0,0)', edgecolor='white', linewidth=2)
        
        # Plot all segment positions (GREEN LINE - actual segment end-effectors)
        if result.segment_positions:
            # Add base to the beginning of segment positions
            all_positions = [[0, 0, 0]]  # Start from base
            for pos in result.segment_positions:
                all_positions.append([pos.x, pos.y, pos.z])
            all_positions = np.array(all_positions)
            
            # Plot complete chain from base through all segments
            ax.plot(all_positions[:, 0], all_positions[:, 1], all_positions[:, 2], 
                   'g-o', linewidth=3, markersize=6, label='Segment Chain', alpha=0.8)
            
            # Label each segment (skip base at index 0)
            for i, (seg_num, pos) in enumerate(zip(result.segment_numbers, result.segment_positions)):
                ax.scatter([pos.x], [pos.y], [pos.z], color='green', s=100, marker='o')
                ax.text(pos.x, pos.y, pos.z + 20, f'S{seg_num}', 
                       fontsize=9, ha='center', va='bottom', color='green', weight='bold')
        
        # Highlight first segment (RED)
        if result.segment_positions:
            first_pos = result.first_segment_position
            ax.scatter([first_pos.x], [first_pos.y], [first_pos.z], 
                      color='red', s=200, marker='*', label='First Segment', 
                      edgecolor='darkred', linewidth=2)
        
        # Show first segment UVW coordinate frame at its proper origin
        if result.segment_positions:
            # UVW frame should be at its origin position, not at 0,0,0
            uvw_origin = [result.uvw_origin.x, result.uvw_origin.y, result.uvw_origin.z]
            u_axis = [result.uvw_u_axis.x, result.uvw_u_axis.y, result.uvw_u_axis.z]
            v_axis = [result.uvw_v_axis.x, result.uvw_v_axis.y, result.uvw_v_axis.z]
            w_axis = [result.uvw_w_axis.x, result.uvw_w_axis.y, result.uvw_w_axis.z]
            
            draw_coordinate_frame(ax, uvw_origin, u_axis, v_axis, w_axis,
                                scale=30.0, colors=['red', 'orange', 'purple'], label="UVW ")
            
            # Mark UVW origin
            ax.scatter(*uvw_origin, color='red', s=150, marker='s', 
                      label='UVW Origin', edgecolor='darkred', linewidth=2)
            
            # Add H and G points from kinematics
            # H point: Joint H at (0, 0, WORKING_HEIGHT)
            WORKING_HEIGHT = 11.5  # From constants
            MIN_HEIGHT = 101.0     # From constants  
            MOTOR_LIMIT = 11.0     # From constants
            
            joint_H = [0, 0, WORKING_HEIGHT]
            ax.scatter(*joint_H, color='cyan', s=120, marker='^', 
                      label='Joint H', edgecolor='darkcyan', linewidth=2)
            
            # G point: Calculate from first segment kinematics
            first_pos = result.first_segment_position
            vector_length = MIN_HEIGHT + 2 * MOTOR_LIMIT + result.prismatic_joint
            
            # Direction from first segment position (normalized)
            first_norm = (first_pos.x**2 + first_pos.y**2 + first_pos.z**2)**0.5
            normalized_dir = [first_pos.x/first_norm, first_pos.y/first_norm, first_pos.z/first_norm]
            
            point_G = [joint_H[0] + normalized_dir[0] * vector_length,
                       joint_H[1] + normalized_dir[1] * vector_length, 
                       joint_H[2] + normalized_dir[2] * vector_length]
            ax.scatter(*point_G, color='magenta', s=120, marker='^', 
                      label='Point G', edgecolor='darkmagenta', linewidth=2)
            
            # Draw H→G line
            ax.plot([joint_H[0], point_G[0]], 
                    [joint_H[1], point_G[1]], 
                    [joint_H[2], point_G[2]], 
                    'magenta', linewidth=3, alpha=0.7, label='H→G Vector')
        
        # Set axis properties
        if result.segment_positions:
            # Calculate bounds from all positions (including base and UVW origin)
            all_coords = [0, 0, 0]  # Base
            for pos in result.segment_positions:
                all_coords.extend([pos.x, pos.y, pos.z])
            all_coords.extend([target_x, target_y, target_z])
            all_coords.extend([uvw_origin[0], uvw_origin[1], uvw_origin[2]])
            all_coords.extend(joint_H)
            all_coords.extend(point_G)
            
            max_range = max(abs(coord) for coord in all_coords) * 1.2
            
            ax.set_xlim([-max_range*0.5, max_range])
            ax.set_ylim([-max_range*0.5, max_range])
            ax.set_zlim([0, max_range])
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f'Motor Module Results\nTarget: ({target_x}, {target_y}, {target_z})')
        
        # Legend
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        
        # Add summary text
        if result.segment_positions:
            summary_text = f"""MOTOR MODULE RESULTS

Target: ({target_x}, {target_y}, {target_z})
Segments Found: {len(result.segment_positions)}

First Segment Analysis:
Position: ({result.first_segment_position.x:.1f}, {result.first_segment_position.y:.1f}, {result.first_segment_position.z:.1f})
Motors: z_A={result.z_A:.3f}, z_B={result.z_B:.3f}, z_C={result.z_C:.3f}
Joints: prismatic={result.prismatic_joint:.3f}
        roll={result.roll_joint:.1f}°, pitch={result.pitch_joint:.1f}°

UVW Frame Origin: ({result.uvw_origin.x:.1f}, {result.uvw_origin.y:.1f}, {result.uvw_origin.z:.1f})

Legend:
- Black Square: Base at (0,0,0)
- Green Line: Complete segment chain from base
- Red Star: First segment (analyzed)
- Red Square: UVW coordinate frame origin
- Cyan Triangle: Joint H (working height)
- Magenta Triangle: Point G (kinematics)
- Magenta Line: H→G vector
- Gold X: Target position"""
            
            ax.text2D(0.02, 0.98, summary_text, transform=ax.transAxes, 
                     verticalalignment='top', fontsize=9, fontfamily='monospace',
                     bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        
        plt.tight_layout()
        plt.show()
        
        print("✓ Motor module visualization complete!")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()