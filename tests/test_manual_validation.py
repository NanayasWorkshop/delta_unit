#!/usr/bin/env python3
"""
Manual Validation Tool for Kinematics and Orientation Modules
Test with custom targets and validate each step
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

def draw_coordinate_frame(ax, origin, u_axis, v_axis, w_axis, scale=20.0, colors=['red', 'green', 'blue'], label="", alpha=1.0):
    """Draw a coordinate frame with colored arrows"""
    origin = np.array(origin)
    u_axis = np.array(u_axis)
    v_axis = np.array(v_axis)
    w_axis = np.array(w_axis)
    
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

def test_kinematics(input_x, input_y, input_z):
    """Test kinematics module with custom input"""
    try:
        import delta_robot
        kinematics = delta_robot.kinematics
        delta_types = delta_robot.delta_types
    except ImportError as e:
        print(f"Import error: {e}")
        return None
    
    print(f"\n{'='*60}")
    print(f"KINEMATICS TEST: Input ({input_x}, {input_y}, {input_z})")
    print(f"{'='*60}")
    
    # Calculate kinematics
    result = kinematics.KinematicsModule.calculate(input_x, input_y, input_z)
    
    # Get constants
    WORKING_HEIGHT = 11.5
    MIN_HEIGHT = 101.0
    MOTOR_LIMIT = 11.0
    
    print(f"\nINPUT ANALYSIS:")
    input_magnitude = np.sqrt(input_x**2 + input_y**2 + input_z**2)
    print(f"  Input vector: ({input_x}, {input_y}, {input_z})")
    print(f"  Magnitude: {input_magnitude:.3f}")
    print(f"  Angle from +Z: {result.input_angle_from_z:.3f} rad = {np.degrees(result.input_angle_from_z):.1f}°")
    
    print(f"\nTRANSFORMED VECTOR:")
    tv = result.transformed_vector
    print(f"  Transformed: ({tv.x:.6f}, {tv.y:.6f}, {tv.z:.6f})")
    print(f"  Magnitude: {np.sqrt(tv.x**2 + tv.y**2 + tv.z**2):.6f} (should be 1.0)")
    
    print(f"\nFERMAT CALCULATION:")
    fp = result.fermat_data.fermat_point
    print(f"  Fermat point: ({fp.x:.3f}, {fp.y:.3f}, {fp.z:.3f})")
    print(f"  Motor heights: z_A={result.fermat_data.z_A:.3f}, z_B={result.fermat_data.z_B:.3f}, z_C={result.fermat_data.z_C:.3f}")
    
    print(f"\nJOINT STATE:")
    print(f"  Prismatic joint: {result.joint_state_data.prismatic_joint:.3f}")
    print(f"  Roll joint: {np.degrees(result.joint_state_data.roll_joint):.1f}°")
    print(f"  Pitch joint: {np.degrees(result.joint_state_data.pitch_joint):.1f}°")
    
    print(f"\nEND-EFFECTOR CALCULATION:")
    ee = result.end_effector_position
    print(f"  End-effector: ({ee.x:.3f}, {ee.y:.3f}, {ee.z:.3f})")
    
    # Manual validation of end-effector calculation
    joint_H = np.array([0, 0, WORKING_HEIGHT])
    vector_length = MIN_HEIGHT + 2 * MOTOR_LIMIT + result.joint_state_data.prismatic_joint
    normalized_input = np.array([input_x, input_y, input_z]) / input_magnitude
    point_G = joint_H + normalized_input * vector_length
    
    print(f"\nMANUAL VALIDATION:")
    print(f"  Joint H: ({joint_H[0]:.3f}, {joint_H[1]:.3f}, {joint_H[2]:.3f})")
    print(f"  Vector length: {vector_length:.3f}")
    print(f"  Point G: ({point_G[0]:.3f}, {point_G[1]:.3f}, {point_G[2]:.3f})")
    
    # Mirror validation
    midpoint = (joint_H + point_G) / 2
    base_point = np.array([0, 0, 0])
    to_plane = base_point - midpoint
    distance_to_plane = np.dot(to_plane, normalized_input)
    manual_ee = base_point - normalized_input * (2.0 * distance_to_plane)
    
    print(f"  Midpoint H-G: ({midpoint[0]:.3f}, {midpoint[1]:.3f}, {midpoint[2]:.3f})")
    print(f"  Manual end-effector: ({manual_ee[0]:.3f}, {manual_ee[1]:.3f}, {manual_ee[2]:.3f})")
    
    # Check if they match
    ee_diff = np.linalg.norm([ee.x - manual_ee[0], ee.y - manual_ee[1], ee.z - manual_ee[2]])
    print(f"  Difference: {ee_diff:.6f} {'✅ MATCH' if ee_diff < 0.001 else '❌ MISMATCH'}")
    
    return result

def test_orientation(input_x, input_y, input_z):
    """Test orientation module with custom input"""
    try:
        import delta_robot
        orientation = delta_robot.orientation
        delta_types = delta_robot.delta_types
    except ImportError as e:
        print(f"Import error: {e}")
        return None
    
    print(f"\n{'='*60}")
    print(f"ORIENTATION TEST: Input ({input_x}, {input_y}, {input_z})")
    print(f"{'='*60}")
    
    # Calculate orientation
    result = orientation.OrientationModule.calculate(input_x, input_y, input_z)
    
    print(f"\nBASIC INFO:")
    fp = result.fermat_point
    ee = result.end_effector_position
    print(f"  Fermat point: ({fp.x:.3f}, {fp.y:.3f}, {fp.z:.3f})")
    print(f"  End-effector: ({ee.x:.3f}, {ee.y:.3f}, {ee.z:.3f})")
    
    # Get A, B, C points
    base_A = delta_types.get_base_position_A()
    base_B = delta_types.get_base_position_B()
    base_C = delta_types.get_base_position_C()
    
    # Get kinematics for z values
    kinematics_result = delta_robot.kinematics.KinematicsModule.calculate(input_x, input_y, input_z)
    A_point = np.array([base_A.x, base_A.y, kinematics_result.fermat_data.z_A])
    B_point = np.array([base_B.x, base_B.y, kinematics_result.fermat_data.z_B])
    C_point = np.array([base_C.x, base_C.y, kinematics_result.fermat_data.z_C])
    
    print(f"\nA, B, C POINTS:")
    print(f"  A: ({A_point[0]:.3f}, {A_point[1]:.3f}, {A_point[2]:.3f})")
    print(f"  B: ({B_point[0]:.3f}, {B_point[1]:.3f}, {B_point[2]:.3f})")
    print(f"  C: ({C_point[0]:.3f}, {C_point[1]:.3f}, {C_point[2]:.3f})")
    
    print(f"\nSTEP 1 - UVW AT FERMAT:")
    uvw = result.UVW_at_fermat
    print(f"  Origin: ({uvw.origin.x:.6f}, {uvw.origin.y:.6f}, {uvw.origin.z:.6f})")
    print(f"  U-axis: ({uvw.u_axis.x:.6f}, {uvw.u_axis.y:.6f}, {uvw.u_axis.z:.6f})")
    print(f"  V-axis: ({uvw.v_axis.x:.6f}, {uvw.v_axis.y:.6f}, {uvw.v_axis.z:.6f})")
    print(f"  W-axis: ({uvw.w_axis.x:.6f}, {uvw.w_axis.y:.6f}, {uvw.w_axis.z:.6f})")
    
    # Manual calculation of UVW frame
    fermat_np = np.array([fp.x, fp.y, fp.z])
    manual_u = (A_point - fermat_np)
    manual_u = manual_u / np.linalg.norm(manual_u)
    
    # Calculate plane normal
    AB = B_point - A_point
    AC = C_point - A_point
    manual_w = np.cross(AC, AB)  # Note: AC × AB (reversed from AB × AC)
    manual_w = manual_w / np.linalg.norm(manual_w)
    if manual_w[2] < 0:  # Ensure W points in +Z direction
        manual_w = -manual_w
    
    manual_v = np.cross(manual_u, manual_w)
    manual_v = manual_v / np.linalg.norm(manual_v)
    
    print(f"\nMANUAL UVW CALCULATION:")
    print(f"  Manual U: ({manual_u[0]:.6f}, {manual_u[1]:.6f}, {manual_u[2]:.6f})")
    print(f"  Manual V: ({manual_v[0]:.6f}, {manual_v[1]:.6f}, {manual_v[2]:.6f})")
    print(f"  Manual W: ({manual_w[0]:.6f}, {manual_w[1]:.6f}, {manual_w[2]:.6f})")
    
    # Check differences
    u_diff = np.linalg.norm([uvw.u_axis.x - manual_u[0], uvw.u_axis.y - manual_u[1], uvw.u_axis.z - manual_u[2]])
    v_diff = np.linalg.norm([uvw.v_axis.x - manual_v[0], uvw.v_axis.y - manual_v[1], uvw.v_axis.z - manual_v[2]])
    w_diff = np.linalg.norm([uvw.w_axis.x - manual_w[0], uvw.w_axis.y - manual_w[1], uvw.w_axis.z - manual_w[2]])
    
    print(f"\nDIFFERENCES:")
    print(f"  U diff: {u_diff:.6f} {'✅' if u_diff < 0.001 else '❌'}")
    print(f"  V diff: {v_diff:.6f} {'✅' if v_diff < 0.001 else '❌'}")
    print(f"  W diff: {w_diff:.6f} {'✅' if w_diff < 0.001 else '❌'}")
    
    print(f"\nSTEP 2 - IJK MIRRORED:")
    ijk = result.IJK_mirrored
    print(f"  Origin: ({ijk.origin.x:.6f}, {ijk.origin.y:.6f}, {ijk.origin.z:.6f})")
    print(f"  I-axis: ({ijk.u_axis.x:.6f}, {ijk.u_axis.y:.6f}, {ijk.u_axis.z:.6f})")
    print(f"  J-axis: ({ijk.v_axis.x:.6f}, {ijk.v_axis.y:.6f}, {ijk.v_axis.z:.6f})")
    print(f"  K-axis: ({ijk.w_axis.x:.6f}, {ijk.w_axis.y:.6f}, {ijk.w_axis.z:.6f})")
    
    print(f"\nSTEP 3 - U'V'W' ALIGNED:")
    uvw_prime = result.UVW_prime_aligned
    print(f"  Origin: ({uvw_prime.origin.x:.6f}, {uvw_prime.origin.y:.6f}, {uvw_prime.origin.z:.6f})")
    print(f"  U'-axis: ({uvw_prime.u_axis.x:.6f}, {uvw_prime.u_axis.y:.6f}, {uvw_prime.u_axis.z:.6f})")
    print(f"  V'-axis: ({uvw_prime.v_axis.x:.6f}, {uvw_prime.v_axis.y:.6f}, {uvw_prime.v_axis.z:.6f})")
    print(f"  W'-axis: ({uvw_prime.w_axis.x:.6f}, {uvw_prime.w_axis.y:.6f}, {uvw_prime.w_axis.z:.6f})")
    
    print(f"\nSTEP 4 - FINAL U''V''W'':")
    final = result.final_frame
    print(f"  Origin: ({final.origin.x:.6f}, {final.origin.y:.6f}, {final.origin.z:.6f})")
    print(f"  U''-axis: ({final.u_axis.x:.6f}, {final.u_axis.y:.6f}, {final.u_axis.z:.6f})")
    print(f"  V''-axis: ({final.v_axis.x:.6f}, {final.v_axis.y:.6f}, {final.v_axis.z:.6f})")
    print(f"  W''-axis: ({final.w_axis.x:.6f}, {final.w_axis.y:.6f}, {final.w_axis.z:.6f})")
    
    # Check final axis directions
    print(f"\nFINAL AXIS DIRECTION ANALYSIS:")
    print(f"  U'' · X: {final.u_axis.x:.3f} {'(+)' if final.u_axis.x > 0 else '(-)'}")
    print(f"  V'' · Y: {final.v_axis.y:.3f} {'(+)' if final.v_axis.y > 0 else '(-)'}")
    print(f"  W'' · Z: {final.w_axis.z:.3f} {'(+)' if final.w_axis.z > 0 else '(-)'}")
    
    if final.u_axis.x < 0 or final.v_axis.y < 0 or final.w_axis.z < 0:
        print(f"  ⚠️  WARNING: Some axes pointing in negative directions!")
    else:
        print(f"  ✅ All axes pointing in positive directions")
    
    return result

def visualize_comparison(input_x, input_y, input_z):
    """Create visual comparison of kinematics and orientation results"""
    try:
        import delta_robot
    except ImportError as e:
        print(f"Import error: {e}")
        return
    
    # Get both results
    kinematics_result = delta_robot.kinematics.KinematicsModule.calculate(input_x, input_y, input_z)
    orientation_result = delta_robot.orientation.OrientationModule.calculate(input_x, input_y, input_z)
    
    # Create figure
    fig = plt.figure(figsize=(16, 12))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot origin
    ax.scatter([0], [0], [0], color='black', s=200, marker='s', 
              label='Origin (0,0,0)', edgecolor='white', linewidth=2)
    
    # Plot XYZ reference frame
    draw_coordinate_frame(ax, [0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1],
                         scale=30.0, colors=['red', 'green', 'blue'], label="XYZ ", alpha=0.6)
    
    # Plot input vector
    input_magnitude = np.sqrt(input_x**2 + input_y**2 + input_z**2)
    ax.quiver(0, 0, 0, input_x, input_y, input_z, 
              color='purple', arrow_length_ratio=0.05, linewidth=4, label='Input Vector')
    
    # Plot kinematics points
    ee_k = kinematics_result.end_effector_position
    ax.scatter([ee_k.x], [ee_k.y], [ee_k.z], color='red', s=200, marker='*', 
              label='Kinematics End-Effector', edgecolor='darkred', linewidth=2)
    
    fp_k = kinematics_result.fermat_data.fermat_point
    ax.scatter([fp_k.x], [fp_k.y], [fp_k.z], color='orange', s=150, marker='s', 
              label='Kinematics Fermat', edgecolor='darkorange', linewidth=2)
    
    # Plot orientation points
    ee_o = orientation_result.end_effector_position
    fp_o = orientation_result.fermat_point
    
    # Check if they match
    ee_diff = np.linalg.norm([ee_k.x - ee_o.x, ee_k.y - ee_o.y, ee_k.z - ee_o.z])
    fp_diff = np.linalg.norm([fp_k.x - fp_o.x, fp_k.y - fp_o.y, fp_k.z - fp_o.z])
    
    if ee_diff < 0.001:
        print(f"✅ End-effector positions match (diff: {ee_diff:.6f})")
    else:
        print(f"❌ End-effector positions differ (diff: {ee_diff:.6f})")
        ax.scatter([ee_o.x], [ee_o.y], [ee_o.z], color='cyan', s=200, marker='*', 
                  label='Orientation End-Effector', edgecolor='darkcyan', linewidth=2)
    
    if fp_diff < 0.001:
        print(f"✅ Fermat points match (diff: {fp_diff:.6f})")
    else:
        print(f"❌ Fermat points differ (diff: {fp_diff:.6f})")
        ax.scatter([fp_o.x], [fp_o.y], [fp_o.z], color='lightblue', s=150, marker='s', 
                  label='Orientation Fermat', edgecolor='darkblue', linewidth=2)
    
    # Plot final UVW frame
    final = orientation_result.final_frame
    uvw_origin = [final.origin.x, final.origin.y, final.origin.z]
    u_axis = [final.u_axis.x, final.u_axis.y, final.u_axis.z]
    v_axis = [final.v_axis.x, final.v_axis.y, final.v_axis.z]
    w_axis = [final.w_axis.x, final.w_axis.y, final.w_axis.z]
    
    draw_coordinate_frame(ax, uvw_origin, u_axis, v_axis, w_axis,
                         scale=40.0, colors=['magenta', 'yellow', 'cyan'], label="UVW ")
    
    # Set axis properties
    all_coords = [0, 0, 0, input_x, input_y, input_z, ee_k.x, ee_k.y, ee_k.z, fp_k.x, fp_k.y, fp_k.z]
    max_range = max(abs(coord) for coord in all_coords) * 1.3
    
    ax.set_xlim([-max_range*0.5, max_range])
    ax.set_ylim([-max_range*0.5, max_range])
    ax.set_zlim([0, max_range])
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Manual Validation: Input ({input_x}, {input_y}, {input_z})')
    
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    # Add analysis text
    analysis_text = f"""MANUAL VALIDATION RESULTS

Input: ({input_x}, {input_y}, {input_z})
Magnitude: {input_magnitude:.3f}

Kinematics:
End-Effector: ({ee_k.x:.3f}, {ee_k.y:.3f}, {ee_k.z:.3f})
Fermat: ({fp_k.x:.3f}, {fp_k.y:.3f}, {fp_k.z:.3f})

Orientation:
End-Effector: ({ee_o.x:.3f}, {ee_o.y:.3f}, {ee_o.z:.3f})
Fermat: ({fp_o.x:.3f}, {fp_o.y:.3f}, {fp_o.z:.3f})

Final UVW Axes:
U: ({final.u_axis.x:.3f}, {final.u_axis.y:.3f}, {final.u_axis.z:.3f})
V: ({final.v_axis.x:.3f}, {final.v_axis.y:.3f}, {final.v_axis.z:.3f})
W: ({final.w_axis.x:.3f}, {final.w_axis.y:.3f}, {final.w_axis.z:.3f})

Axis Directions:
U·X: {final.u_axis.x:.3f} {'(+)' if final.u_axis.x > 0 else '(-)'}
V·Y: {final.v_axis.y:.3f} {'(+)' if final.v_axis.y > 0 else '(-)'}
W·Z: {final.w_axis.z:.3f} {'(+)' if final.w_axis.z > 0 else '(-)'}

Consistency:
EE diff: {ee_diff:.6f} {'✅' if ee_diff < 0.001 else '❌'}
FP diff: {fp_diff:.6f} {'✅' if fp_diff < 0.001 else '❌'}"""
    
    ax.text2D(0.02, 0.98, analysis_text, transform=ax.transAxes, 
             verticalalignment='top', fontsize=8, fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
    
    plt.tight_layout()
    plt.show()

def main():
    if len(sys.argv) == 2:
        input_x, input_y, input_z = parse_coordinates(sys.argv[1])
        print(f"Testing with input: ({input_x}, {input_y}, {input_z})")
    else:
        # Test with a simple case first
        input_x, input_y, input_z = 10, 0, 10
        print(f"Testing with default input: ({input_x}, {input_y}, {input_z})")
        print(f"Usage: python3 {os.path.basename(__file__)} x,y,z")
        print("Try simple test cases like: 10,0,10 or 0,10,10 or 5,5,10")
    
    # Run tests
    kinematics_result = test_kinematics(input_x, input_y, input_z)
    orientation_result = test_orientation(input_x, input_y, input_z)
    
    if kinematics_result and orientation_result:
        print(f"\n{'='*60}")
        print("Creating visual comparison...")
        visualize_comparison(input_x, input_y, input_z)
        
        print(f"\n{'='*60}")
        print("RECOMMENDATIONS:")
        print("1. Try simple test cases: 10,0,10 or 0,10,10")
        print("2. Check if UVW axes are consistently oriented")
        print("3. Look for axis flipping in the orientation calculation")
        print("4. Verify that the plane normal calculation is correct")

if __name__ == "__main__":
    main()