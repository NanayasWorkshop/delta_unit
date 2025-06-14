#!/usr/bin/env python3
"""
Test UVW coordinate system visualization with U'V'W' frame
"""

import sys
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def parse_coordinates(coord_str):
    """Parse coordinate string like '5,4,7' into x,y,z values"""
    coords = [float(x.strip()) for x in coord_str.split(',')]
    if len(coords) != 3:
        raise ValueError("Expected 3 coordinates")
    return coords

def draw_coordinate_system(ax, origin, u_axis, v_axis, w_axis, scale=10, colors=['cyan', 'magenta', 'yellow'], label_prefix=''):
    """Draw UVW coordinate system"""
    # Draw axes
    ax.quiver(origin[0], origin[1], origin[2], u_axis[0]*scale, u_axis[1]*scale, u_axis[2]*scale, 
              color=colors[0], arrow_length_ratio=0.1, linewidth=2, label=f'{label_prefix}U')
    ax.quiver(origin[0], origin[1], origin[2], v_axis[0]*scale, v_axis[1]*scale, v_axis[2]*scale, 
              color=colors[1], arrow_length_ratio=0.1, linewidth=2, label=f'{label_prefix}V')
    ax.quiver(origin[0], origin[1], origin[2], w_axis[0]*scale, w_axis[1]*scale, w_axis[2]*scale, 
              color=colors[2], arrow_length_ratio=0.1, linewidth=2, label=f'{label_prefix}W')

def main():
    # Parse input
    if len(sys.argv) == 2:
        x, y, z = parse_coordinates(sys.argv[1])
    else:
        x, y, z = 5, 4, 7
        print("Usage: python3 test_uvw_viz.py x,y,z")
        print(f"Using default: ({x}, {y}, {z})")
    
    # Import modules
    try:
        from delta_robot import kinematics_module as km
        from delta_robot import orientation_module as om
        from delta_robot import fermat_module as fm
    except ImportError as e:
        print(f"Error: {e}")
        print("Build the modules first.")
        return
    
    # Calculate kinematics and orientation
    kinematics = km.KinematicsModule.calculate(x, y, z)
    orientation = om.OrientationModule.calculate_from_kinematics(kinematics)
    
    # Extract points
    fermat = orientation.fermat_point
    end_effector = orientation.end_effector_position
    
    # Get base positions and A,B,C points
    base_A = fm.FermatModule.get_base_A()
    base_B = fm.FermatModule.get_base_B() 
    base_C = fm.FermatModule.get_base_C()
    
    A_point = (base_A.x, base_A.y, kinematics.fermat_data.z_A)
    B_point = (base_B.x, base_B.y, kinematics.fermat_data.z_B)
    C_point = (base_C.x, base_C.y, kinematics.fermat_data.z_C)
    
    # Extract all coordinate frames
    uvw_frame = orientation.UVW_at_fermat
    ijk_frame = orientation.IJK_mirrored
    uvw_prime_frame = orientation.UVW_prime_aligned
    final_frame = orientation.final_frame
    
    # Create plot
    fig = plt.figure(figsize=(15, 12))
    ax = fig.add_subplot(111, projection='3d')
    
    # Calculate H and G points for kinematics
    H_point = (0, 0, 11.5)  # Joint H at WORKING_HEIGHT
    
    # Calculate G point
    vector_length = 101.0 + 2*11.0 + kinematics.prismatic_joint_length  # MIN_HEIGHT + 2*MOTOR_LIMIT + prismatic
    normalized_input = (x, y, z)
    input_norm = (x*x + y*y + z*z)**0.5
    normalized_input = (x/input_norm, y/input_norm, z/input_norm)
    
    G_point = (H_point[0] + normalized_input[0] * vector_length,
               H_point[1] + normalized_input[1] * vector_length, 
               H_point[2] + normalized_input[2] * vector_length)
    
    # Plot points
    ax.scatter(*A_point, color='red', s=100, label='A point')
    ax.scatter(*B_point, color='green', s=100, label='B point') 
    ax.scatter(*C_point, color='blue', s=100, label='C point')
    ax.scatter(fermat.x, fermat.y, fermat.z, color='black', s=150, label='Fermat point')
    ax.scatter(end_effector.x, end_effector.y, end_effector.z, color='orange', s=100, label='End effector')
    ax.scatter(*H_point, color='purple', s=120, label='H point')
    ax.scatter(*G_point, color='brown', s=120, label='G point')
    
    # Draw ABC triangle
    triangle_x = [A_point[0], B_point[0], C_point[0], A_point[0]]
    triangle_y = [A_point[1], B_point[1], C_point[1], A_point[1]]
    triangle_z = [A_point[2], B_point[2], C_point[2], A_point[2]]
    ax.plot(triangle_x, triangle_y, triangle_z, 'k--', alpha=0.5)
    
    # Draw Base-H-G-Target line
    base_point = (0, 0, 0)
    line_x = [base_point[0], H_point[0], G_point[0], end_effector.x]
    line_y = [base_point[1], H_point[1], G_point[1], end_effector.y] 
    line_z = [base_point[2], H_point[2], G_point[2], end_effector.z]
    ax.plot(line_x, line_y, line_z, 'r-', linewidth=2, alpha=0.7, label='Base-H-G-Target')
    
    # 1. Draw UVW coordinate system at ACTUAL Fermat point position from C++
    fermat_origin = (uvw_frame.origin.x, uvw_frame.origin.y, uvw_frame.origin.z)
    u_axis = (uvw_frame.u_axis.x, uvw_frame.u_axis.y, uvw_frame.u_axis.z)
    v_axis = (uvw_frame.v_axis.x, uvw_frame.v_axis.y, uvw_frame.v_axis.z)
    w_axis = (uvw_frame.w_axis.x, uvw_frame.w_axis.y, uvw_frame.w_axis.z)
    
    draw_coordinate_system(ax, fermat_origin, u_axis, v_axis, w_axis, 
                          scale=12, colors=['cyan', 'magenta', 'yellow'], label_prefix='UVW-')
    
    # 2. Draw IJK coordinate system at ACTUAL mirrored position from C++
    ijk_origin = (ijk_frame.origin.x, ijk_frame.origin.y, ijk_frame.origin.z)
    i_axis = (ijk_frame.u_axis.x, ijk_frame.u_axis.y, ijk_frame.u_axis.z)
    j_axis = (ijk_frame.v_axis.x, ijk_frame.v_axis.y, ijk_frame.v_axis.z)
    k_axis = (ijk_frame.w_axis.x, ijk_frame.w_axis.y, ijk_frame.w_axis.z)
    
    draw_coordinate_system(ax, ijk_origin, i_axis, j_axis, k_axis,
                          scale=12, colors=['orange', 'lime', 'purple'], label_prefix='IJK-')
    
    # 3. Draw U'V'W' coordinate system at ACTUAL aligned position from C++
    uvw_prime_origin = (uvw_prime_frame.origin.x, uvw_prime_frame.origin.y, uvw_prime_frame.origin.z)
    u_prime_axis = (uvw_prime_frame.u_axis.x, uvw_prime_frame.u_axis.y, uvw_prime_frame.u_axis.z)
    v_prime_axis = (uvw_prime_frame.v_axis.x, uvw_prime_frame.v_axis.y, uvw_prime_frame.v_axis.z)
    w_prime_axis = (uvw_prime_frame.w_axis.x, uvw_prime_frame.w_axis.y, uvw_prime_frame.w_axis.z)
    
    draw_coordinate_system(ax, uvw_prime_origin, u_prime_axis, v_prime_axis, w_prime_axis,
                          scale=12, colors=['darkturquoise', 'hotpink', 'gold'], label_prefix="U'V'W'-")
    
    # 4. Draw final U''V''W'' coordinate system at ACTUAL position from C++
    final_origin = (final_frame.origin.x, final_frame.origin.y, final_frame.origin.z)
    u_final_axis = (final_frame.u_axis.x, final_frame.u_axis.y, final_frame.u_axis.z)
    v_final_axis = (final_frame.v_axis.x, final_frame.v_axis.y, final_frame.v_axis.z)
    w_final_axis = (final_frame.w_axis.x, final_frame.w_axis.y, final_frame.w_axis.z)
    
    draw_coordinate_system(ax, final_origin, u_final_axis, v_final_axis, w_final_axis,
                          scale=12, colors=['navy', 'maroon', 'olive'], label_prefix="U''V''W''-")
    
    # 5. Draw XYZ coordinate system at origin
    origin = (0, 0, 0)
    x_axis = (1, 0, 0)
    y_axis = (0, 1, 0) 
    z_axis = (0, 0, 1)
    
    draw_coordinate_system(ax, origin, x_axis, y_axis, z_axis,
                          scale=25, colors=['red', 'green', 'blue'], label_prefix='XYZ-')
    
    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y') 
    ax.set_zlabel('Z')
    ax.set_title(f'Complete Coordinate System Transformation\nInput: ({x}, {y}, {z})')
    
    # Force equal aspect ratio and same scale on all axes
    max_range = 120  # Increased range for better visibility
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    ax.set_zlim([-max_range, max_range])
    ax.set_box_aspect([1,1,1])
    
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    # Print coordinate system info
    # Check if W axis is parallel to H-G vector
    HG_vector = (G_point[0] - H_point[0], G_point[1] - H_point[1], G_point[2] - H_point[2])
    HG_norm = (HG_vector[0]**2 + HG_vector[1]**2 + HG_vector[2]**2)**0.5
    HG_normalized = (HG_vector[0]/HG_norm, HG_vector[1]/HG_norm, HG_vector[2]/HG_norm)
    
    # Check parallelism using dot product
    w_dot_HG = abs(w_axis[0]*HG_normalized[0] + w_axis[1]*HG_normalized[1] + w_axis[2]*HG_normalized[2])
    is_parallel = abs(w_dot_HG - 1.0) < 0.001  # Should be close to 1 if parallel
    
    print(f"Input vector: ({x}, {y}, {z})")
    print(f"Fermat point: ({fermat.x:.3f}, {fermat.y:.3f}, {fermat.z:.3f})")
    print(f"A point: ({A_point[0]:.3f}, {A_point[1]:.3f}, {A_point[2]:.3f})")
    print(f"B point: ({B_point[0]:.3f}, {B_point[1]:.3f}, {B_point[2]:.3f})")
    print(f"C point: ({C_point[0]:.3f}, {C_point[1]:.3f}, {C_point[2]:.3f})")
    print(f"H point: ({H_point[0]:.3f}, {H_point[1]:.3f}, {H_point[2]:.3f})")
    print(f"G point: ({G_point[0]:.3f}, {G_point[1]:.3f}, {G_point[2]:.3f})")
    print(f"End effector: ({end_effector.x:.3f}, {end_effector.y:.3f}, {end_effector.z:.3f})")
    
    print(f"\n=== RAW C++ COORDINATE FRAME VALUES (NO OFFSETS) ===")
    
    print(f"\n1. UVW frame at Fermat (cyan/magenta/yellow):")
    print(f"   Origin: ({uvw_frame.origin.x:.3f}, {uvw_frame.origin.y:.3f}, {uvw_frame.origin.z:.3f})")
    print(f"   Fermat: ({fermat.x:.3f}, {fermat.y:.3f}, {fermat.z:.3f})")
    print(f"   Match:  {'✓ YES' if abs(uvw_frame.origin.x - fermat.x) < 0.001 and abs(uvw_frame.origin.y - fermat.y) < 0.001 and abs(uvw_frame.origin.z - fermat.z) < 0.001 else '✗ NO'}")
    print(f"   U: ({u_axis[0]:.3f}, {u_axis[1]:.3f}, {u_axis[2]:.3f})")
    print(f"   V: ({v_axis[0]:.3f}, {v_axis[1]:.3f}, {v_axis[2]:.3f})")
    print(f"   W: ({w_axis[0]:.3f}, {w_axis[1]:.3f}, {w_axis[2]:.3f})")
    
    print(f"\n2. IJK frame mirrored (orange/lime/purple):")
    print(f"   Origin: ({ijk_frame.origin.x:.3f}, {ijk_frame.origin.y:.3f}, {ijk_frame.origin.z:.3f})")
    print(f"   I: ({i_axis[0]:.3f}, {i_axis[1]:.3f}, {i_axis[2]:.3f})")
    print(f"   J: ({j_axis[0]:.3f}, {j_axis[1]:.3f}, {j_axis[2]:.3f})")
    print(f"   K: ({k_axis[0]:.3f}, {k_axis[1]:.3f}, {k_axis[2]:.3f})")
    
    print(f"\n3. U'V'W' frame aligned (darkturquoise/hotpink/gold):")
    print(f"   Origin: ({uvw_prime_frame.origin.x:.3f}, {uvw_prime_frame.origin.y:.3f}, {uvw_prime_frame.origin.z:.3f})")
    print(f"   U': ({u_prime_axis[0]:.3f}, {u_prime_axis[1]:.3f}, {u_prime_axis[2]:.3f})")
    print(f"   V': ({v_prime_axis[0]:.3f}, {v_prime_axis[1]:.3f}, {v_prime_axis[2]:.3f})")
    print(f"   W': ({w_prime_axis[0]:.3f}, {w_prime_axis[1]:.3f}, {w_prime_axis[2]:.3f})")
    
    print(f"\n4. U''V''W'' final frame (navy/maroon/olive):")
    print(f"   C++ Origin: ({final_frame.origin.x:.3f}, {final_frame.origin.y:.3f}, {final_frame.origin.z:.3f})")
    print(f"   Target:     ({end_effector.x:.3f}, {end_effector.y:.3f}, {end_effector.z:.3f})")
    print(f"   Input:      ({x}, {y}, {z})")
    
    # Check if C++ final frame origin matches target
    final_matches_target = (abs(final_frame.origin.x - end_effector.x) < 0.001 and 
                           abs(final_frame.origin.y - end_effector.y) < 0.001 and 
                           abs(final_frame.origin.z - end_effector.z) < 0.001)
    
    # Check if target matches input
    target_matches_input = (abs(end_effector.x - x) < 0.001 and 
                           abs(end_effector.y - y) < 0.001 and 
                           abs(end_effector.z - z) < 0.001)
    
    print(f"   Final→Target: {'✓ YES' if final_matches_target else '✗ NO'}")
    print(f"   Target→Input: {'✓ YES' if target_matches_input else '✗ NO'}")
    print(f"   U'': ({u_final_axis[0]:.3f}, {u_final_axis[1]:.3f}, {u_final_axis[2]:.3f})")
    print(f"   V'': ({v_final_axis[0]:.3f}, {v_final_axis[1]:.3f}, {v_final_axis[2]:.3f})")
    print(f"   W'': ({w_final_axis[0]:.3f}, {w_final_axis[1]:.3f}, {w_final_axis[2]:.3f})")
    
    print(f"\nH-G vector normalized: ({HG_normalized[0]:.3f}, {HG_normalized[1]:.3f}, {HG_normalized[2]:.3f})")
    print(f"W · H-G dot product: {w_dot_HG:.6f}")
    print(f"W parallel to H-G: {'✓ YES' if is_parallel else '✗ NO'}")
    
    # Check if U'V'W' axes are aligned with XYZ
    print(f"\n=== ALIGNMENT CHECK ===")
    u_prime_dot_x = abs(u_prime_axis[0]*1 + u_prime_axis[1]*0 + u_prime_axis[2]*0)
    v_prime_dot_y = abs(v_prime_axis[0]*0 + v_prime_axis[1]*1 + v_prime_axis[2]*0)
    w_prime_dot_z = abs(w_prime_axis[0]*0 + w_prime_axis[1]*0 + w_prime_axis[2]*1)
    
    print(f"U' · X axis: {u_prime_dot_x:.6f} (should be ~1.0)")
    print(f"V' · Y axis: {v_prime_dot_y:.6f} (should be ~1.0)")
    print(f"W' · Z axis: {w_prime_dot_z:.6f} (should be ~1.0)")
    
    is_aligned = (abs(u_prime_dot_x - 1.0) < 0.001 and 
                  abs(v_prime_dot_y - 1.0) < 0.001 and 
                  abs(w_prime_dot_z - 1.0) < 0.001)
    
    print(f"U'V'W' aligned with XYZ: {'✓ YES' if is_aligned else '✗ NO'}")
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()