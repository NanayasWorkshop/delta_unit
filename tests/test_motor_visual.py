#!/usr/bin/env python3
"""
Motor Module Detailed Visual Test - Chain overview + individual segment details
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

def draw_coordinate_frame(ax, origin, u_axis, v_axis, w_axis, scale=30.0, colors=['red', 'green', 'blue'], label="", alpha=1.0, linewidth=2):
    """Draw a coordinate frame with colored arrows"""
    origin = np.array(origin)
    u_axis = np.array(u_axis)
    v_axis = np.array(v_axis)
    w_axis = np.array(w_axis)
    
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

def plot_overall_chain(result, target_pos):
    """Plot the overall chain with all UVW frames visible"""
    fig = plt.figure(figsize=(16, 12))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot target position (GOLD)
    ax.scatter([target_pos[0]], [target_pos[1]], [target_pos[2]], color='gold', s=400, 
              marker='X', label='Target', edgecolor='black', linewidth=2)
    
    # Plot base at origin (BLACK)
    ax.scatter([0], [0], [0], color='black', s=200, 
              marker='s', label='Base (0,0,0)', edgecolor='white', linewidth=2)
    
    # Plot complete original chain (BLUE LINE)
    if result.original_segment_positions:
        # Add base to the beginning
        all_positions = [[0, 0, 0]]  # Start from base
        for pos in result.original_segment_positions:
            all_positions.append([pos.x, pos.y, pos.z])
        all_positions = np.array(all_positions)
        
        # Plot complete chain from base through all segments
        ax.plot(all_positions[:, 0], all_positions[:, 1], all_positions[:, 2], 
               'b-o', linewidth=3, markersize=8, label='Original Chain', alpha=0.8)
        
        # Label each original segment
        for i, (seg_num, pos) in enumerate(zip(result.original_segment_numbers, result.original_segment_positions)):
            ax.text(pos.x, pos.y, pos.z + 20, f'S{seg_num}', 
                   fontsize=10, ha='center', va='bottom', color='blue', weight='bold')
    
    # Plot UVW frames for each level
    colors_per_level = [
        ['red', 'orange', 'purple'],      # Level 0
        ['darkred', 'darkorange', 'darkmagenta'],  # Level 1
        ['crimson', 'chocolate', 'indigo'], # Level 2
        ['maroon', 'orangered', 'violet'],  # Level 3
        ['darkslategray', 'peru', 'mediumorchid'], # Level 4
    ]
    
    for level_idx, level_data in enumerate(result.levels[:5]):  # Limit to first 5 levels for clarity
        base_original_seg_num = result.original_segment_numbers[level_idx]
        level_primes = "'" * level_idx
        
        # Highlight the base segment for this level
        base_pos = level_data.base_segment_position
        ax.scatter([base_pos.x], [base_pos.y], [base_pos.z], 
                  color=colors_per_level[level_idx % len(colors_per_level)][0], 
                  s=250, marker='*', 
                  label=f'Base S{base_original_seg_num}{level_primes}', 
                  edgecolor='black', linewidth=1)
        
        # Draw UVW coordinate frame
        uvw_origin = [level_data.uvw_origin.x, level_data.uvw_origin.y, level_data.uvw_origin.z]
        u_axis = [level_data.uvw_u_axis.x, level_data.uvw_u_axis.y, level_data.uvw_u_axis.z]
        v_axis = [level_data.uvw_v_axis.x, level_data.uvw_v_axis.y, level_data.uvw_v_axis.z]
        w_axis = [level_data.uvw_w_axis.x, level_data.uvw_w_axis.y, level_data.uvw_w_axis.z]
        
        draw_coordinate_frame(ax, uvw_origin, u_axis, v_axis, w_axis,
                             scale=40.0, 
                             colors=colors_per_level[level_idx % len(colors_per_level)], 
                             label=f"L{level_idx} ", 
                             alpha=0.8,
                             linewidth=2)
        
        # Mark UVW origin
        ax.scatter(*uvw_origin, 
                  color=colors_per_level[level_idx % len(colors_per_level)][2], 
                  s=120, marker='s', 
                  label=f'L{level_idx} UVW Origin', 
                  edgecolor='black', linewidth=1)
    
    # Set axis properties
    all_coords = [0, 0, 0]  # Base
    for pos in result.original_segment_positions:
        all_coords.extend([pos.x, pos.y, pos.z])
    all_coords.extend([target_pos[0], target_pos[1], target_pos[2]])
    
    for level_data in result.levels[:5]:
        all_coords.extend([level_data.uvw_origin.x, level_data.uvw_origin.y, level_data.uvw_origin.z])
    
    max_range = max(abs(coord) for coord in all_coords) * 1.2
    
    ax.set_xlim([-max_range*0.3, max_range])
    ax.set_ylim([-max_range*0.3, max_range])
    ax.set_zlim([0, max_range])
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Overall Chain with UVW Frames\nTarget: ({target_pos[0]}, {target_pos[1]}, {target_pos[2]})')
    
    # Legend
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=8)
    
    plt.tight_layout()
    return fig

def plot_segment_detail(level_idx, level_data, result):
    """Plot detailed analysis for a specific segment"""
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Get segment info
    base_original_seg_num = result.original_segment_numbers[level_idx]
    level_primes = "'" * level_idx
    base_pos = level_data.base_segment_position
    
    # Constants from delta robot (should match your actual constants)
    WORKING_HEIGHT = 11.5
    MIN_HEIGHT = 101.0
    MOTOR_LIMIT = 11.0
    
    # Plot origin (0,0,0)
    ax.scatter([0], [0], [0], color='black', s=200, marker='s', 
              label='Origin (0,0,0)', edgecolor='white', linewidth=2)
    
    # Plot the segment being analyzed (LARGE RED STAR)
    ax.scatter([base_pos.x], [base_pos.y], [base_pos.z], 
              color='red', s=500, marker='*', 
              label=f'Segment {base_original_seg_num}{level_primes} (Analyzed)', 
              edgecolor='darkred', linewidth=3)
    
    # Calculate and plot H point (Joint H at working height)
    joint_H = [0, 0, WORKING_HEIGHT]
    ax.scatter(*joint_H, color='cyan', s=200, marker='^', 
              label='Joint H (Working Height)', edgecolor='darkcyan', linewidth=2)
    
    # Calculate and plot G point (from kinematics)
    vector_length = MIN_HEIGHT + 2 * MOTOR_LIMIT + level_data.prismatic_joint
    
    # Direction from segment position (normalized)
    segment_norm = (base_pos.x**2 + base_pos.y**2 + base_pos.z**2)**0.5
    if segment_norm > 0:
        normalized_dir = [base_pos.x/segment_norm, base_pos.y/segment_norm, base_pos.z/segment_norm]
        
        point_G = [joint_H[0] + normalized_dir[0] * vector_length,
                   joint_H[1] + normalized_dir[1] * vector_length, 
                   joint_H[2] + normalized_dir[2] * vector_length]
        
        ax.scatter(*point_G, color='magenta', s=200, marker='^', 
                  label='Point G (Kinematics)', edgecolor='darkmagenta', linewidth=2)
        
        # Draw H→G vector (THICK MAGENTA LINE)
        ax.plot([joint_H[0], point_G[0]], 
                [joint_H[1], point_G[1]], 
                [joint_H[2], point_G[2]], 
                'magenta', linewidth=4, alpha=0.8, label='H→G Vector')
    
    # Draw UVW coordinate frame at UVW origin (LARGE SCALE)
    uvw_origin = [level_data.uvw_origin.x, level_data.uvw_origin.y, level_data.uvw_origin.z]
    u_axis = [level_data.uvw_u_axis.x, level_data.uvw_u_axis.y, level_data.uvw_u_axis.z]
    v_axis = [level_data.uvw_v_axis.x, level_data.uvw_v_axis.y, level_data.uvw_v_axis.z]
    w_axis = [level_data.uvw_w_axis.x, level_data.uvw_w_axis.y, level_data.uvw_w_axis.z]
    
    draw_coordinate_frame(ax, uvw_origin, u_axis, v_axis, w_axis,
                         scale=60.0, colors=['red', 'orange', 'purple'], 
                         label="UVW ", alpha=1.0, linewidth=3)
    
    # Mark UVW origin (LARGE PURPLE SQUARE)
    ax.scatter(*uvw_origin, color='purple', s=250, marker='s', 
              label='UVW Origin', edgecolor='darkmagenta', linewidth=2)
    
    # Draw standard XYZ frame at origin for reference (SMALLER, TRANSPARENT)
    draw_coordinate_frame(ax, [0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1],
                         scale=40.0, colors=['cyan', 'lime', 'yellow'], 
                         label="XYZ ", alpha=0.5, linewidth=2)
    
    # Draw line from origin to segment (DASHED GRAY)
    ax.plot([0, base_pos.x], [0, base_pos.y], [0, base_pos.z], 
           '--', color='gray', linewidth=2, alpha=0.7, label='Origin→Segment')
    
    # Set axis properties
    all_coords = [0, 0, 0, base_pos.x, base_pos.y, base_pos.z]
    all_coords.extend(uvw_origin)
    all_coords.extend(joint_H)
    if 'point_G' in locals():
        all_coords.extend(point_G)
    
    max_range = max(abs(coord) for coord in all_coords) * 1.3
    
    ax.set_xlim([-max_range*0.5, max_range])
    ax.set_ylim([-max_range*0.5, max_range])
    ax.set_zlim([0, max_range])
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Detailed Analysis: Segment {base_original_seg_num}{level_primes}\n'
                f'Position: ({base_pos.x:.3f}, {base_pos.y:.3f}, {base_pos.z:.3f})')
    
    # Legend
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=9)
    
    # Add detailed analysis text
    analysis_text = f"""DETAILED SEGMENT ANALYSIS

Segment: {base_original_seg_num}{level_primes}
Position: ({base_pos.x:.3f}, {base_pos.y:.3f}, {base_pos.z:.3f})

KINEMATICS:
Motors: z_A={level_data.z_A:.3f}, z_B={level_data.z_B:.3f}, z_C={level_data.z_C:.3f}
Joints: prismatic={level_data.prismatic_joint:.3f}
        roll={level_data.roll_joint:.1f}°, pitch={level_data.pitch_joint:.1f}°

H Point: ({joint_H[0]:.1f}, {joint_H[1]:.1f}, {joint_H[2]:.1f})
Vector Length: {vector_length:.3f}
Segment Distance: {segment_norm:.3f}

ORIENTATION (UVW Frame):
Origin: ({level_data.uvw_origin.x:.3f}, {level_data.uvw_origin.y:.3f}, {level_data.uvw_origin.z:.3f})
U-axis: ({level_data.uvw_u_axis.x:.3f}, {level_data.uvw_u_axis.y:.3f}, {level_data.uvw_u_axis.z:.3f})
V-axis: ({level_data.uvw_v_axis.x:.3f}, {level_data.uvw_v_axis.y:.3f}, {level_data.uvw_v_axis.z:.3f})
W-axis: ({level_data.uvw_w_axis.x:.3f}, {level_data.uvw_w_axis.y:.3f}, {level_data.uvw_w_axis.z:.3f})

AXIS DIRECTIONS:
U dot X: {level_data.uvw_u_axis.x:.3f} {'(+)' if level_data.uvw_u_axis.x > 0 else '(-)'}
V dot Y: {level_data.uvw_v_axis.y:.3f} {'(+)' if level_data.uvw_v_axis.y > 0 else '(-)'}
W dot Z: {level_data.uvw_w_axis.z:.3f} {'(+)' if level_data.uvw_w_axis.z > 0 else '(-)'}

Legend:
- Red Star: Analyzed segment
- Purple Square: UVW frame origin
- Cyan Triangle: Joint H
- Magenta Triangle: Point G (if calculated)
- Red/Orange/Purple: UVW axes
- Cyan/Lime/Yellow: XYZ reference axes
- Magenta Line: H→G kinematics vector
- Gray Dashed: Origin to segment"""
    
    ax.text2D(0.02, 0.98, analysis_text, transform=ax.transAxes, 
             verticalalignment='top', fontsize=8, fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.9))
    
    plt.tight_layout()
    return fig

def main():
    if len(sys.argv) == 2:
        target_x, target_y, target_z = parse_coordinates(sys.argv[1])
        print(f"Target: ({target_x}, {target_y}, {target_z})")
    else:
        target_x, target_y, target_z = 100, 50, 300
        print(f"Default target: ({target_x}, {target_y}, {target_z})")
        print(f"Usage: python3 {os.path.basename(__file__)} x,y,z")
    
    print("=" * 50)
    
    try:
        import delta_robot
        result = delta_robot.calculate_motors(target_x, target_y, target_z)
        
        if result is None:
            print("Motor module did not return a result.")
            return

        print(f"Processing {len(result.levels)} transformation levels...")
        
        # Create overall chain plot
        print("Creating overall chain plot...")
        overall_fig = plot_overall_chain(result, (target_x, target_y, target_z))
        
        # Create detailed plots for each segment
        detail_figures = []
        for level_idx, level_data in enumerate(result.levels):
            base_original_seg_num = result.original_segment_numbers[level_idx]
            level_primes = "'" * level_idx
            print(f"Creating detailed plot for Segment {base_original_seg_num}{level_primes}...")
            
            fig = plot_segment_detail(level_idx, level_data, result)
            detail_figures.append(fig)
        
        print(f"\n✓ Created 1 overall plot + {len(detail_figures)} detailed plots!")
        print("First plot shows the overall chain with all UVW frames.")
        print("Subsequent plots show detailed analysis for each segment.")
        print("Close each plot window to see the next one.")
        
        # Show all plots
        plt.show()
        
    except ImportError as e:
        print(f"ImportError: {e}")
        print("Please ensure the compiled C++ module is in the Python path.")
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()