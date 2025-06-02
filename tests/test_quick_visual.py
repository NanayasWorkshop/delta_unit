#!/usr/bin/env python3
"""
Quick 3D Visualization Test - Uses only kinematics module to avoid import conflicts
"""

import sys
import os
import math
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

def main():
    # Parse input
    if len(sys.argv) == 2:
        input_x, input_y, input_z = parse_coordinates(sys.argv[1])
        print(f"Using command line input: ({input_x}, {input_y}, {input_z})")
    else:
        input_x, input_y, input_z = 5, 4, 7
        print("Using default input: (5, 4, 7)")
        print("Usage: python3 test_quick_visual.py x,y,z")
    
    # Import only kinematics module to avoid conflicts
    try:
        import kinematics_module as km
        print("✓ Kinematics module imported successfully")
        
        # Try to also import orientation module for coordinate systems
        try:
            import orientation_module as om
            print("✓ Orientation module also imported successfully")
            has_orientation = True
        except ImportError as e:
            print(f"⚠ Orientation module not available: {e}")
            has_orientation = False
            
    except ImportError as e:
        print(f"✗ Error importing kinematics module: {e}")
        print("Build first with: python3 setup.py build_ext --inplace")
        sys.exit(1)
    
    print("\n" + "="*60)
    print("QUICK DELTA ROBOT VISUALIZATION")
    print("="*60)
    
    # Calculate kinematics (includes all internal calculations)
    print(f"Calculating for input vector: ({input_x}, {input_y}, {input_z})")
    result = km.KinematicsModule.calculate(input_x, input_y, input_z)
    
    # Try to get orientation data if available
    orientation_result = None
    if has_orientation:
        try:
            orientation_result = om.OrientationModule.calculate_from_kinematics(result)
            print("✓ Orientation calculation successful")
        except Exception as e:
            print(f"⚠ Orientation calculation failed: {e}")
            has_orientation = False
    
    # Extract data
    fermat = result.fermat_data
    joint_state = result.joint_state_data
    end_eff = result.end_effector_position
    
    print(f"✓ Calculation complete!")
    print(f"  Fermat point: ({fermat.fermat_point.x:.2f}, {fermat.fermat_point.y:.2f}, {fermat.fermat_point.z:.2f})")
    print(f"  Prismatic joint: {joint_state.prismatic_joint:.2f} mm")
    print(f"  End-effector: ({end_eff.x:.2f}, {end_eff.y:.2f}, {end_eff.z:.2f})")
    
    # Create visualization
    fig = plt.figure(figsize=(15, 10))
    fig.suptitle(f'Delta Robot Quick Visualization - Input: ({input_x}, {input_y}, {input_z})', fontsize=14)
    
    # Main 3D plot
    ax = fig.add_subplot(1, 1, 1, projection='3d')
    
    # Constants
    ROBOT_RADIUS = 24.8
    WORKING_HEIGHT = 11.5
    MIN_HEIGHT = 101.0
    MOTOR_LIMIT = 11.0
    
    # Base positions
    base_A = [0, ROBOT_RADIUS, 0]
    base_B = [ROBOT_RADIUS * math.cos(-math.pi/6), ROBOT_RADIUS * math.sin(-math.pi/6), 0]
    base_C = [ROBOT_RADIUS * math.cos(-5*math.pi/6), ROBOT_RADIUS * math.sin(-5*math.pi/6), 0]
    
    # Robot base circle
    theta = np.linspace(0, 2*np.pi, 100)
    base_x = ROBOT_RADIUS * np.cos(theta)
    base_y = ROBOT_RADIUS * np.sin(theta)
    base_z = np.zeros_like(theta)
    ax.plot(base_x, base_y, base_z, 'k-', linewidth=2, alpha=0.3, label='Robot Base')
    
    # Base positions
    ax.scatter(*base_A, color='red', s=100, marker='o', label='Base A')
    ax.scatter(*base_B, color='green', s=100, marker='o', label='Base B')
    ax.scatter(*base_C, color='blue', s=100, marker='o', label='Base C')
    
    # Triangle ABC with calculated Z values
    A_3d = [base_A[0], base_A[1], fermat.z_A]
    B_3d = [base_B[0], base_B[1], fermat.z_B]
    C_3d = [base_C[0], base_C[1], fermat.z_C]
    
    ax.scatter(*A_3d, color='red', s=120, marker='^', label=f'A (z={fermat.z_A:.1f})')
    ax.scatter(*B_3d, color='green', s=120, marker='^', label=f'B (z={fermat.z_B:.1f})')
    ax.scatter(*C_3d, color='blue', s=120, marker='^', label=f'C (z={fermat.z_C:.1f})')
    
    # Triangle edges
    triangle = np.array([A_3d, B_3d, C_3d, A_3d])
    ax.plot(triangle[:,0], triangle[:,1], triangle[:,2], 'k-', linewidth=2, alpha=0.7)
    
    # Fermat point
    fermat_pos = [fermat.fermat_point.x, fermat.fermat_point.y, fermat.fermat_point.z]
    ax.scatter(*fermat_pos, color='magenta', s=150, marker='*', label='Fermat Point')
    
    # Lines from Fermat to triangle vertices
    for point, color in [(A_3d, 'red'), (B_3d, 'green'), (C_3d, 'blue')]:
        ax.plot([fermat_pos[0], point[0]], [fermat_pos[1], point[1]], [fermat_pos[2], point[2]], 
               color=color, linestyle=':', alpha=0.6)
    
    # Kinematic chain
    origin = [0, 0, 0]
    joint_H = [0, 0, WORKING_HEIGHT]
    end_effector = [end_eff.x, end_eff.y, end_eff.z]
    
    # Calculate Point G
    vector_length = MIN_HEIGHT + 2 * MOTOR_LIMIT + joint_state.prismatic_joint
    input_vec = np.array([input_x, input_y, input_z])
    input_norm = input_vec / np.linalg.norm(input_vec)
    point_G = np.array(joint_H) + input_norm * vector_length
    
    # Calculate distance from G to End-Effector
    G_to_endeff_distance = np.linalg.norm(np.array(end_effector) - point_G)
    
    # Plot kinematic chain
    ax.scatter(*origin, color='black', s=100, marker='o', label='Origin')
    ax.scatter(*joint_H, color='purple', s=120, marker='s', label='Joint H')
    ax.scatter(*point_G, color='orange', s=120, marker='^', label='Point G')
    ax.scatter(*end_effector, color='red', s=200, marker='*', label='End-Effector')
    
    # Links
    ax.plot([origin[0], joint_H[0]], [origin[1], joint_H[1]], [origin[2], joint_H[2]], 
           'purple', linewidth=4, label='Fixed Link')
    ax.plot([joint_H[0], point_G[0]], [joint_H[1], point_G[1]], [joint_H[2], point_G[2]], 
           'orange', linewidth=4, label=f'Prismatic ({joint_state.prismatic_joint:.1f}mm)')
    
    # Direction vectors
    ax.quiver(0, 0, 0, input_x*0.8, input_y*0.8, input_z*0.8, 
             color='orange', arrow_length_ratio=0.1, linewidth=3, label='Input Direction')
    
    # Transformed (half-angle) vector
    transformed = result.transformed_vector
    ax.quiver(0, 0, 0, transformed.x*12, transformed.y*12, transformed.z*12,
             color='green', arrow_length_ratio=0.1, linewidth=2, label='Half-Angle Vector')
    
    # Direct lines instead of dotted mirror lines
    ax.plot([origin[0], end_effector[0]], [origin[1], end_effector[1]], [origin[2], end_effector[2]], 
           'red', linewidth=3, alpha=0.8, label='Base→End-Effector')
    ax.plot([point_G[0], end_effector[0]], [point_G[1], end_effector[1]], [point_G[2], end_effector[2]], 
           'orange', linewidth=3, alpha=0.8, label='G→End-Effector')
    
    # Add coordinate systems if orientation is available
    if has_orientation and orientation_result:
        print("Adding coordinate systems visualization...")
        
        # World coordinate system (XYZ) at origin
        axis_length = 15
        ax.quiver(0, 0, 0, axis_length, 0, 0, color='darkred', arrow_length_ratio=0.15, linewidth=3, label='World X')
        ax.quiver(0, 0, 0, 0, axis_length, 0, color='darkgreen', arrow_length_ratio=0.15, linewidth=3, label='World Y') 
        ax.quiver(0, 0, 0, 0, 0, axis_length, color='darkblue', arrow_length_ratio=0.15, linewidth=3, label='World Z')
        
        # UVW coordinate system at Fermat point
        uvw_frame = orientation_result.UVW_at_fermat
        fermat_origin = [uvw_frame.origin.x, uvw_frame.origin.y, uvw_frame.origin.z]
        
        # UVW axes at Fermat point
        uvw_length = 12
        ax.quiver(fermat_origin[0], fermat_origin[1], fermat_origin[2],
                 uvw_frame.u_axis.x * uvw_length, uvw_frame.u_axis.y * uvw_length, uvw_frame.u_axis.z * uvw_length,
                 color='salmon', arrow_length_ratio=0.15, linewidth=2, alpha=0.8, label='UVW-U (Fermat)')
        ax.quiver(fermat_origin[0], fermat_origin[1], fermat_origin[2],
                 uvw_frame.v_axis.x * uvw_length, uvw_frame.v_axis.y * uvw_length, uvw_frame.v_axis.z * uvw_length,
                 color='lightgreen', arrow_length_ratio=0.15, linewidth=2, alpha=0.8, label='UVW-V (Fermat)')
        ax.quiver(fermat_origin[0], fermat_origin[1], fermat_origin[2],
                 uvw_frame.w_axis.x * uvw_length, uvw_frame.w_axis.y * uvw_length, uvw_frame.w_axis.z * uvw_length,
                 color='lightblue', arrow_length_ratio=0.15, linewidth=2, alpha=0.8, label='UVW-W (Fermat)')
        
        # Final U''V''W'' coordinate system at End-Effector
        final_frame = orientation_result.final_frame
        final_length = 20
        
        ax.quiver(end_effector[0], end_effector[1], end_effector[2],
                 final_frame.u_axis.x * final_length, final_frame.u_axis.y * final_length, final_frame.u_axis.z * final_length,
                 color='red', arrow_length_ratio=0.12, linewidth=4, label="U'' (End-Eff)")
        ax.quiver(end_effector[0], end_effector[1], end_effector[2],
                 final_frame.v_axis.x * final_length, final_frame.v_axis.y * final_length, final_frame.v_axis.z * final_length,
                 color='green', arrow_length_ratio=0.12, linewidth=4, label="V'' (End-Eff)")
        ax.quiver(end_effector[0], end_effector[1], end_effector[2],
                 final_frame.w_axis.x * final_length, final_frame.w_axis.y * final_length, final_frame.w_axis.z * final_length,
                 color='blue', arrow_length_ratio=0.12, linewidth=4, label="W'' (End-Eff)")
    else:
        print("Coordinate systems not available - showing basic XYZ only")
        # Just show world coordinate system
        axis_length = 15
        ax.quiver(0, 0, 0, axis_length, 0, 0, color='darkred', arrow_length_ratio=0.15, linewidth=3, label='World X')
        ax.quiver(0, 0, 0, 0, axis_length, 0, color='darkgreen', arrow_length_ratio=0.15, linewidth=3, label='World Y')
        ax.quiver(0, 0, 0, 0, 0, axis_length, color='darkblue', arrow_length_ratio=0.15, linewidth=3, label='World Z')
    
    # Add info text
    info_text = f"""
CALCULATION RESULTS:
Input Vector: ({input_x}, {input_y}, {input_z})
Input Angle from Z: {math.degrees(result.input_angle_from_z):.1f}°
Half Angle: {math.degrees(result.input_angle_from_z/2):.1f}°

FERMAT CALCULATION:
Z Heights: A={fermat.z_A:.1f}, B={fermat.z_B:.1f}, C={fermat.z_C:.1f}
Fermat Point: ({fermat.fermat_point.x:.2f}, {fermat.fermat_point.y:.2f}, {fermat.fermat_point.z:.2f})

JOINT STATE:
Prismatic: {joint_state.prismatic_joint:.2f} mm
Roll: {math.degrees(joint_state.roll_joint):.1f}°
Pitch: {math.degrees(joint_state.pitch_joint):.1f}°

KINEMATICS:
End-Effector: ({end_eff.x:.2f}, {end_eff.y:.2f}, {end_eff.z:.2f})
Distance H→G: {vector_length:.1f} mm
Distance G→End-Eff: {G_to_endeff_distance:.2f} mm

COORDINATE SYSTEMS:
UVW at Fermat: {'✓' if has_orientation and orientation_result else '✗'}
U''V''W'' at End-Eff: {'✓' if has_orientation and orientation_result else '✗'}
Transform Matrix: {'✓' if has_orientation and orientation_result else '✗'}
    """
    
    ax.text2D(0.02, 0.98, info_text, transform=ax.transAxes, fontsize=8, 
             verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
    
    ax.set_title('Complete Delta Robot Pipeline\n(Fermat → Joint State → Kinematics)')
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    # Save and show
    save_path = f"delta_quick_visual_{input_x}_{input_y}_{input_z}.png"
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"\n✓ Visualization saved to: {save_path}")
    
    plt.tight_layout()
    plt.show()
    
    print("\n" + "="*60)
    print("VISUALIZATION COMPLETE!")
    print("This shows all your modules working together:")
    print("1. Fermat calculation (triangle ABC, Fermat point)")
    print("2. Joint state calculation (prismatic, roll, pitch)")
    print("3. Kinematics calculation (end-effector position)")
    print("4. Half-angle transformation and mirror geometry")
    print("="*60)

if __name__ == "__main__":
    main()