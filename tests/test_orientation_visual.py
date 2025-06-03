#!/usr/bin/env python3
"""
Simple Visual Orientation Test - Shows all 4 coordinate transformation steps
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

def draw_coordinate_frame(ax, origin, u_axis, v_axis, w_axis, scale=10.0, colors=['red', 'green', 'blue'], label=""):
    """Draw a coordinate frame with colored arrows of length 10"""
    # Draw axes with fixed length of 10
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
    
    # Add A point
    base_A = dt.get_base_position_A()
    A_point = [base_A.x, base_A.y, kinematics_result.fermat_data.z_A]
    print(f"Point A: ({A_point[0]:.3f}, {A_point[1]:.3f}, {A_point[2]:.3f})")
    
    print(f"\nUVW Orientation (Step 1):")
    uvw_frame = orientation_result.UVW_at_fermat
    print(f"  Origin: ({uvw_frame.origin.x:.6f}, {uvw_frame.origin.y:.6f}, {uvw_frame.origin.z:.6f})")
    print(f"  U-axis: ({uvw_frame.u_axis.x:.6f}, {uvw_frame.u_axis.y:.6f}, {uvw_frame.u_axis.z:.6f})")
    print(f"  V-axis: ({uvw_frame.v_axis.x:.6f}, {uvw_frame.v_axis.y:.6f}, {uvw_frame.v_axis.z:.6f})")
    print(f"  W-axis: ({uvw_frame.w_axis.x:.6f}, {uvw_frame.w_axis.y:.6f}, {uvw_frame.w_axis.z:.6f})")
    
    print(f"\nIJK Mirrored (Step 2):")
    ijk_frame = orientation_result.IJK_mirrored
    print(f"  Origin: ({ijk_frame.origin.x:.6f}, {ijk_frame.origin.y:.6f}, {ijk_frame.origin.z:.6f})")
    print(f"  I-axis: ({ijk_frame.u_axis.x:.6f}, {ijk_frame.u_axis.y:.6f}, {ijk_frame.u_axis.z:.6f})")
    print(f"  J-axis: ({ijk_frame.v_axis.x:.6f}, {ijk_frame.v_axis.y:.6f}, {ijk_frame.v_axis.z:.6f})")
    print(f"  K-axis: ({ijk_frame.w_axis.x:.6f}, {ijk_frame.w_axis.y:.6f}, {ijk_frame.w_axis.z:.6f})")
    
    print(f"\nU'V'W' Aligned (Step 3):")
    uvw_prime_frame = orientation_result.UVW_prime_aligned
    print(f"  Origin: ({uvw_prime_frame.origin.x:.6f}, {uvw_prime_frame.origin.y:.6f}, {uvw_prime_frame.origin.z:.6f})")
    print(f"  U'-axis: ({uvw_prime_frame.u_axis.x:.6f}, {uvw_prime_frame.u_axis.y:.6f}, {uvw_prime_frame.u_axis.z:.6f})")
    print(f"  V'-axis: ({uvw_prime_frame.v_axis.x:.6f}, {uvw_prime_frame.v_axis.y:.6f}, {uvw_prime_frame.v_axis.z:.6f})")
    print(f"  W'-axis: ({uvw_prime_frame.w_axis.x:.6f}, {uvw_prime_frame.w_axis.y:.6f}, {uvw_prime_frame.w_axis.z:.6f})")
    
    print(f"\nFinal U''V''W'' (Step 4):")
    final_frame = orientation_result.final_frame
    print(f"  Origin: ({final_frame.origin.x:.6f}, {final_frame.origin.y:.6f}, {final_frame.origin.z:.6f})")
    print(f"  U''-axis: ({final_frame.u_axis.x:.6f}, {final_frame.u_axis.y:.6f}, {final_frame.u_axis.z:.6f})")
    print(f"  V''-axis: ({final_frame.v_axis.x:.6f}, {final_frame.v_axis.y:.6f}, {final_frame.v_axis.z:.6f})")
    print(f"  W''-axis: ({final_frame.w_axis.x:.6f}, {final_frame.w_axis.y:.6f}, {final_frame.w_axis.z:.6f})")
    
    # === VISUALIZATION ===
    print("=" * 50)
    print("Creating 3D visualization...")
    
    # Create 3D plot
    fig = plt.figure(figsize=(14, 12))
    ax = fig.add_subplot(111, projection='3d')
    
    # World XYZ coordinate frame at origin (RED, GREEN, BLUE)
    world_origin = [0, 0, 0]
    world_x = [1, 0, 0]
    world_y = [0, 1, 0] 
    world_z = [0, 0, 1]
    draw_coordinate_frame(ax, world_origin, world_x, world_y, world_z, 
                        scale=10.0, colors=['red', 'green', 'blue'], label="World ")
    
    # === A, B, C POINTS ===
    # Get A, B, C points from kinematics result
    base_A = dt.get_base_position_A()
    base_B = dt.get_base_position_B()
    base_C = dt.get_base_position_C()
    
    A_point = [base_A.x, base_A.y, kinematics_result.fermat_data.z_A]
    B_point = [base_B.x, base_B.y, kinematics_result.fermat_data.z_B]
    C_point = [base_C.x, base_C.y, kinematics_result.fermat_data.z_C]
    
    # Plot A, B, C points
    ax.scatter(*A_point, color='red', s=120, marker='^', label='Point A', edgecolor='darkred', linewidth=2)
    ax.scatter(*B_point, color='green', s=120, marker='^', label='Point B', edgecolor='darkgreen', linewidth=2)
    ax.scatter(*C_point, color='blue', s=120, marker='^', label='Point C', edgecolor='darkblue', linewidth=2)
    
    # Draw ABC triangle
    triangle_x = [A_point[0], B_point[0], C_point[0], A_point[0]]
    triangle_y = [A_point[1], B_point[1], C_point[1], A_point[1]]
    triangle_z = [A_point[2], B_point[2], C_point[2], A_point[2]]
    ax.plot(triangle_x, triangle_y, triangle_z, 'black', linewidth=2, alpha=0.7, label='Triangle ABC')
    
    # End-effector position (BLACK point)
    end_eff_pos = [orientation_result.end_effector_position.x, 
                   orientation_result.end_effector_position.y, 
                   orientation_result.end_effector_position.z]
    ax.scatter(*end_eff_pos, color='black', s=100, label='End-Effector', marker='o')
    
    # Fermat point (ORANGE point)
    fermat_pos = [orientation_result.fermat_point.x, 
                  orientation_result.fermat_point.y, 
                  orientation_result.fermat_point.z]
    ax.scatter(*fermat_pos, color='orange', s=80, label='Fermat Point', marker='s')
    
    # === ALL 4 COORDINATE SYSTEMS ===
    
    # Step 1: UVW frame at Fermat point (MAGENTA, YELLOW, CYAN)
    uvw_frame = orientation_result.UVW_at_fermat
    uvw_u = [uvw_frame.u_axis.x, uvw_frame.u_axis.y, uvw_frame.u_axis.z]
    uvw_v = [uvw_frame.v_axis.x, uvw_frame.v_axis.y, uvw_frame.v_axis.z]
    uvw_w = [uvw_frame.w_axis.x, uvw_frame.w_axis.y, uvw_frame.w_axis.z]
    draw_coordinate_frame(ax, fermat_pos, uvw_u, uvw_v, uvw_w,
                        scale=10.0, colors=['magenta', 'yellow', 'cyan'], label="1.UVW ")
    
    # Step 2: IJK frame (mirrored across XY) (BROWN, TAN, CHOCOLATE)
    ijk_frame = orientation_result.IJK_mirrored
    ijk_origin = [ijk_frame.origin.x, ijk_frame.origin.y, ijk_frame.origin.z]
    ijk_u = [ijk_frame.u_axis.x, ijk_frame.u_axis.y, ijk_frame.u_axis.z]
    ijk_v = [ijk_frame.v_axis.x, ijk_frame.v_axis.y, ijk_frame.v_axis.z]
    ijk_w = [ijk_frame.w_axis.x, ijk_frame.w_axis.y, ijk_frame.w_axis.z]
    draw_coordinate_frame(ax, ijk_origin, ijk_u, ijk_v, ijk_w,
                        scale=10.0, colors=['brown', 'tan', 'chocolate'], label="2.IJK ")
    ax.scatter(*ijk_origin, color='brown', s=60, label='IJK Origin', marker='v', alpha=0.8)
    
    # Step 3: U'V'W' frame (aligned with origin) (PINK, LIGHTCORAL, HOTPINK)
    uvw_prime_frame = orientation_result.UVW_prime_aligned
    uvw_prime_origin = [uvw_prime_frame.origin.x, uvw_prime_frame.origin.y, uvw_prime_frame.origin.z]
    uvw_prime_u = [uvw_prime_frame.u_axis.x, uvw_prime_frame.u_axis.y, uvw_prime_frame.u_axis.z]
    uvw_prime_v = [uvw_prime_frame.v_axis.x, uvw_prime_frame.v_axis.y, uvw_prime_frame.v_axis.z]
    uvw_prime_w = [uvw_prime_frame.w_axis.x, uvw_prime_frame.w_axis.y, uvw_prime_frame.w_axis.z]
    draw_coordinate_frame(ax, uvw_prime_origin, uvw_prime_u, uvw_prime_v, uvw_prime_w,
                        scale=10.0, colors=['pink', 'lightcoral', 'hotpink'], label="3.U'V'W' ")
    ax.scatter(*uvw_prime_origin, color='pink', s=60, label="U'V'W' Origin", marker='d', alpha=0.8)
    
    # Step 4: U''V''W'' frame (final at end-effector) (LIME, SPRINGGREEN, DARKGREEN)
    final_frame = orientation_result.final_frame
    final_u = [final_frame.u_axis.x, final_frame.u_axis.y, final_frame.u_axis.z]
    final_v = [final_frame.v_axis.x, final_frame.v_axis.y, final_frame.v_axis.z]
    final_w = [final_frame.w_axis.x, final_frame.w_axis.y, final_frame.w_axis.z]
    draw_coordinate_frame(ax, end_eff_pos, final_u, final_v, final_w,
                        scale=10.0, colors=['lime', 'springgreen', 'darkgreen'], label="4.U''V''W'' ")
    
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
    
    # Set axis properties - EQUAL SCALING
    max_range = max(abs(coord) for pos in [end_eff_pos, fermat_pos, point_G, ijk_origin, uvw_prime_origin, A_point, B_point, C_point] for coord in pos) * 1.2
    
    # Equal scaling for all axes
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    ax.set_zlim([-max_range, max_range])
    
    # Force equal aspect ratio
    ax.set_box_aspect([1,1,1])
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Delta Robot 4-Step Coordinate Transformation\nInput: ({input_x}, {input_y}, {input_z})')
    
    # Legend
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    # Add text info
    info_text = f"""Input: ({input_x:.1f}, {input_y:.1f}, {input_z:.1f})
End-Effector: ({end_eff_pos[0]:.1f}, {end_eff_pos[1]:.1f}, {end_eff_pos[2]:.1f})
Fermat Point: ({fermat_pos[0]:.1f}, {fermat_pos[1]:.1f}, {fermat_pos[2]:.1f})
ABC Points: A({A_point[0]:.1f},{A_point[1]:.1f},{A_point[2]:.1f})
           B({B_point[0]:.1f},{B_point[1]:.1f},{B_point[2]:.1f})
           C({C_point[0]:.1f},{C_point[1]:.1f},{C_point[2]:.1f})

Transformation Steps:
1. UVW at Fermat (Magenta/Yellow/Cyan)
2. IJK Mirrored (Brown/Tan/Chocolate)  
3. U'V'W' Aligned (Pink/LightCoral/HotPink)
4. U''V''W'' Final (Lime/SpringGreen/DarkGreen)"""
    
    ax.text2D(0.02, 0.98, info_text, transform=ax.transAxes, 
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    plt.show()
    
    print("=" * 50)

if __name__ == "__main__":
    main()