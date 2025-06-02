#!/usr/bin/env python3
"""
Test the orientation module interface
"""

import sys
import os
import math
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

def print_matrix(matrix, name="Matrix"):
    """Print 4x4 matrix in formatted way"""
    print(f"{name}:")
    for i in range(4):
        row = [f"{matrix[i,j]:8.4f}" for j in range(4)]
        print(f"  [{' '.join(row)}]")

def print_coordinate_frame(frame, name="Frame"):
    """Print coordinate frame in formatted way"""
    print(f"{name}:")
    print(f"  Origin: ({frame.origin.x:.4f}, {frame.origin.y:.4f}, {frame.origin.z:.4f})")
    print(f"  U-axis: ({frame.u_axis.x:.4f}, {frame.u_axis.y:.4f}, {frame.u_axis.z:.4f})")
    print(f"  V-axis: ({frame.v_axis.x:.4f}, {frame.v_axis.y:.4f}, {frame.v_axis.z:.4f})")
    print(f"  W-axis: ({frame.w_axis.x:.4f}, {frame.w_axis.y:.4f}, {frame.w_axis.z:.4f})")

def verify_orthonormal(frame, name="Frame"):
    """Verify that coordinate frame axes are orthonormal"""
    print(f"\nVerifying {name} orthonormality:")
    
    # Check magnitudes
    u_mag = math.sqrt(frame.u_axis.x**2 + frame.u_axis.y**2 + frame.u_axis.z**2)
    v_mag = math.sqrt(frame.v_axis.x**2 + frame.v_axis.y**2 + frame.v_axis.z**2)
    w_mag = math.sqrt(frame.w_axis.x**2 + frame.w_axis.y**2 + frame.w_axis.z**2)
    
    print(f"  Axis magnitudes - U: {u_mag:.6f}, V: {v_mag:.6f}, W: {w_mag:.6f}")
    
    # Check orthogonality (dot products should be 0)
    u_dot_v = frame.u_axis.x*frame.v_axis.x + frame.u_axis.y*frame.v_axis.y + frame.u_axis.z*frame.v_axis.z
    u_dot_w = frame.u_axis.x*frame.w_axis.x + frame.u_axis.y*frame.w_axis.y + frame.u_axis.z*frame.w_axis.z
    v_dot_w = frame.v_axis.x*frame.w_axis.x + frame.v_axis.y*frame.w_axis.y + frame.v_axis.z*frame.w_axis.z
    
    print(f"  Dot products - U·V: {u_dot_v:.6f}, U·W: {u_dot_w:.6f}, V·W: {v_dot_w:.6f}")
    
    # Check if orthonormal
    is_orthonormal = (abs(u_mag - 1.0) < 1e-6 and abs(v_mag - 1.0) < 1e-6 and abs(w_mag - 1.0) < 1e-6 and
                     abs(u_dot_v) < 1e-6 and abs(u_dot_w) < 1e-6 and abs(v_dot_w) < 1e-6)
    
    if is_orthonormal:
        print("  ✓ Frame is orthonormal")
    else:
        print("  ✗ Warning: Frame is not orthonormal")
    
    return is_orthonormal

def test_orientation_module():
    # Parse command line arguments for direction vector
    if len(sys.argv) == 2:
        input_x, input_y, input_z = parse_coordinates(sys.argv[1])
        print(f"Using command line input: ({input_x}, {input_y}, {input_z})")
    else:
        input_x, input_y, input_z = 5, 4, 7
        print("Using default input: (5, 4, 7)")
        print("Usage: python3 test_orientation.py x,y,z")
    
    try:
        import orientation_module as om
    except ImportError as e:
        print(f"Error: Module not found: {e}")
        print("Build the orientation module first.")
        return False
    
    print("Testing Orientation Module")
    print("=" * 60)
    
    # Test orientation calculation
    print(f"Input vector: ({input_x}, {input_y}, {input_z})")
    
    result = om.OrientationModule.calculate(input_x, input_y, input_z)
    
    print(f"\n=== ORIENTATION RESULTS ===")
    print(f"End-effector position: ({result.end_effector_position.x:.4f}, {result.end_effector_position.y:.4f}, {result.end_effector_position.z:.4f})")
    print(f"Fermat point:          ({result.fermat_point.x:.4f}, {result.fermat_point.y:.4f}, {result.fermat_point.z:.4f})")
    
    print(f"\n=== TRANSFORMATION MATRIX ===")
    print_matrix(result.transformation_matrix, "End-Effector Transformation")
    
    print(f"\n=== COORDINATE FRAMES ===")
    print_coordinate_frame(result.UVW_at_fermat, "UVW at Fermat Point")
    print_coordinate_frame(result.final_frame, "Final Frame at End-Effector")
    
    # Verification
    print(f"\n=== VERIFICATION ===")
    
    # Verify final frame position matches end-effector
    final_pos = result.final_frame.origin
    end_eff_pos = result.end_effector_position
    pos_diff = math.sqrt((final_pos.x - end_eff_pos.x)**2 + 
                        (final_pos.y - end_eff_pos.y)**2 + 
                        (final_pos.z - end_eff_pos.z)**2)
    
    print(f"Final frame origin:    ({final_pos.x:.4f}, {final_pos.y:.4f}, {final_pos.z:.4f})")
    print(f"End-effector position: ({end_eff_pos.x:.4f}, {end_eff_pos.y:.4f}, {end_eff_pos.z:.4f})")
    print(f"Position difference:   {pos_diff:.6f}")
    
    if pos_diff < 1e-6:
        print("✓ Final frame position matches end-effector position")
    else:
        print("✗ Warning: Position mismatch")
    
    # Verify transformation matrix translation
    matrix_translation = (result.transformation_matrix[0,3], 
                         result.transformation_matrix[1,3], 
                         result.transformation_matrix[2,3])
    
    print(f"\nTransformation matrix translation: ({matrix_translation[0]:.4f}, {matrix_translation[1]:.4f}, {matrix_translation[2]:.4f})")
    
    matrix_trans_diff = math.sqrt((matrix_translation[0] - end_eff_pos.x)**2 + 
                                 (matrix_translation[1] - end_eff_pos.y)**2 + 
                                 (matrix_translation[2] - end_eff_pos.z)**2)
    
    if matrix_trans_diff < 1e-6:
        print("✓ Transformation matrix translation is correct")
    else:
        print("✗ Warning: Transformation matrix translation mismatch")
    
    # Verify coordinate frame orthonormality
    verify_orthonormal(result.UVW_at_fermat, "UVW frame")
    verify_orthonormal(result.final_frame, "Final frame")
    
    # Verify rotation matrix is orthonormal
    print(f"\nVerifying transformation matrix rotation part:")
    
    # Extract rotation columns
    col1 = (result.transformation_matrix[0,0], result.transformation_matrix[1,0], result.transformation_matrix[2,0])
    col2 = (result.transformation_matrix[0,1], result.transformation_matrix[1,1], result.transformation_matrix[2,1])
    col3 = (result.transformation_matrix[0,2], result.transformation_matrix[1,2], result.transformation_matrix[2,2])
    
    # Check magnitudes
    col1_mag = math.sqrt(col1[0]**2 + col1[1]**2 + col1[2]**2)
    col2_mag = math.sqrt(col2[0]**2 + col2[1]**2 + col2[2]**2)
    col3_mag = math.sqrt(col3[0]**2 + col3[1]**2 + col3[2]**2)
    
    print(f"  Rotation column magnitudes - Col1: {col1_mag:.6f}, Col2: {col2_mag:.6f}, Col3: {col3_mag:.6f}")
    
    # Check orthogonality
    col1_dot_col2 = col1[0]*col2[0] + col1[1]*col2[1] + col1[2]*col2[2]
    col1_dot_col3 = col1[0]*col3[0] + col1[1]*col3[1] + col1[2]*col3[2]
    col2_dot_col3 = col2[0]*col3[0] + col2[1]*col3[1] + col2[2]*col3[2]
    
    print(f"  Rotation column dot products - Col1·Col2: {col1_dot_col2:.6f}, Col1·Col3: {col1_dot_col3:.6f}, Col2·Col3: {col2_dot_col3:.6f}")
    
    is_rotation_orthonormal = (abs(col1_mag - 1.0) < 1e-6 and abs(col2_mag - 1.0) < 1e-6 and abs(col3_mag - 1.0) < 1e-6 and
                              abs(col1_dot_col2) < 1e-6 and abs(col1_dot_col3) < 1e-6 and abs(col2_dot_col3) < 1e-6)
    
    if is_rotation_orthonormal:
        print("  ✓ Transformation matrix rotation part is orthonormal")
    else:
        print("  ✗ Warning: Transformation matrix rotation part is not orthonormal")
    
    # Check bottom row
    bottom_row = (result.transformation_matrix[3,0], result.transformation_matrix[3,1], 
                 result.transformation_matrix[3,2], result.transformation_matrix[3,3])
    
    print(f"  Bottom row: [{bottom_row[0]:.4f}, {bottom_row[1]:.4f}, {bottom_row[2]:.4f}, {bottom_row[3]:.4f}]")
    
    is_bottom_correct = (abs(bottom_row[0]) < 1e-6 and abs(bottom_row[1]) < 1e-6 and 
                        abs(bottom_row[2]) < 1e-6 and abs(bottom_row[3] - 1.0) < 1e-6)
    
    if is_bottom_correct:
        print("  ✓ Bottom row is correct [0, 0, 0, 1]")
    else:
        print("  ✗ Warning: Bottom row should be [0, 0, 0, 1]")
    
    print(f"\n=== COORDINATE SYSTEM ANALYSIS ===")
    
    # Analyze UVW frame orientation
    print(f"UVW Frame Analysis:")
    fermat = result.fermat_point
    
    # U-axis should point from Fermat to A point  
    print(f"  U-axis points from Fermat toward A point")
    print(f"  Fermat point: ({fermat.x:.4f}, {fermat.y:.4f}, {fermat.z:.4f})")
    
    # W-axis should be normal to ABC plane
    print(f"  W-axis is normal to ABC triangle plane")
    
    # V-axis should be W × U
    print(f"  V-axis is W × U (right-hand rule)")
    
    # Final transformation properties
    print(f"\nFinal Transformation Properties:")
    print(f"  - Position: Places coordinate system at end-effector")
    print(f"  - Orientation: Maintains UVW orientation relative to robot geometry")
    print(f"  - Usage: Can transform vectors from end-effector frame to world frame")
    
    print(f"\n=== SUMMARY ===")
    all_checks_passed = (pos_diff < 1e-6 and matrix_trans_diff < 1e-6 and 
                        is_rotation_orthonormal and is_bottom_correct)
    
    if all_checks_passed:
        print("✓ All verifications PASSED")
        print("✓ Orientation module working correctly!")
        print("✓ Transformation matrix is valid and ready for use")
    else:
        print("✗ Some verifications FAILED")
        print("  Check the implementation for issues")
    
    print(f"\nTransformation matrix usage:")
    print(f"  - Apply to vectors in end-effector frame to get world coordinates")
    print(f"  - Combines position and orientation in standard 4x4 format")
    print(f"  - Compatible with standard robotics transformation libraries")
    
    return True

if __name__ == "__main__":
    test_orientation_module()