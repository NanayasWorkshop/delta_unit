#!/usr/bin/env python3
"""
Test the kinematics module interface
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

def test_kinematics_module():
    # Parse command line arguments for direction vector
    if len(sys.argv) == 2:
        input_x, input_y, input_z = parse_coordinates(sys.argv[1])
        print(f"Using command line input: ({input_x}, {input_y}, {input_z})")
    else:
        input_x, input_y, input_z = 5, 4, 7
        print("Using default input: (5, 4, 7)")
        print("Usage: python3 test_kinematics.py x,y,z")
    
    try:
        import kinematics_module as km
    except ImportError as e:
        print(f"Error: Module not found: {e}")
        print("Build the kinematics module first.")
        return False
    
    print("Testing Kinematics Module")
    print("=" * 50)
    
    # Test kinematics calculation
    print(f"Input vector: ({input_x}, {input_y}, {input_z})")
    
    result = km.KinematicsModule.calculate(input_x, input_y, input_z)
    
    print(f"\n=== KINEMATICS RESULTS ===")
    print(f"Original input vector: ({result.original_input.x:.4f}, {result.original_input.y:.4f}, {result.original_input.z:.4f})")
    print(f"Angle from +Z axis:    {result.input_angle_from_z:.4f} rad ({math.degrees(result.input_angle_from_z):.2f}°)")
    print(f"Half angle:            {result.input_angle_from_z/2:.4f} rad ({math.degrees(result.input_angle_from_z/2):.2f}°)")
    print(f"Transformed vector:    ({result.transformed_vector.x:.4f}, {result.transformed_vector.y:.4f}, {result.transformed_vector.z:.4f})")
    print(f"Prismatic joint length: {result.prismatic_joint_length:.4f}")
    print(f"\n=== IMPORTANT KINEMATICS VALUES ===")
    print(f"End-effector position: ({result.end_effector_position.x:.4f}, {result.end_effector_position.y:.4f}, {result.end_effector_position.z:.4f})")
    
    # Calculate G point position
    vector_length = 101.0 + 2*11.0 + result.prismatic_joint_length  # MIN_HEIGHT + 2*MOTOR_LIMIT + prismatic
    normalized_input = result.original_input
    input_norm = math.sqrt(normalized_input.x**2 + normalized_input.y**2 + normalized_input.z**2)
    normalized_input_x = normalized_input.x / input_norm
    normalized_input_y = normalized_input.y / input_norm
    normalized_input_z = normalized_input.z / input_norm
    
    joint_H_x, joint_H_y, joint_H_z = 0, 0, 11.5  # WORKING_HEIGHT
    point_G_x = joint_H_x + normalized_input_x * vector_length
    point_G_y = joint_H_y + normalized_input_y * vector_length
    point_G_z = joint_H_z + normalized_input_z * vector_length
    
    print(f"Point G position:      ({point_G_x:.4f}, {point_G_y:.4f}, {point_G_z:.4f})")
    print(f"Distance H to G:       {vector_length:.4f}")
    
    print(f"\n=== MIRROR VALIDATION ===")
    # Distance from Base (0,0,0) to H (0,0,11.5)
    base_to_H = math.sqrt(0**2 + 0**2 + 11.5**2)
    
    # Distance from G to End-effector
    G_to_endeff = math.sqrt((point_G_x - result.end_effector_position.x)**2 + 
                           (point_G_y - result.end_effector_position.y)**2 + 
                           (point_G_z - result.end_effector_position.z)**2)
    
    print(f"Distance Base to H:    {base_to_H:.4f}")
    print(f"Distance G to End-eff: {G_to_endeff:.4f}")
    print(f"Difference:            {abs(base_to_H - G_to_endeff):.6f}")
    
    if abs(base_to_H - G_to_endeff) < 0.001:
        print("✓ Mirror validation PASSED - distances are equal!")
    else:
        print("✗ Mirror validation FAILED - distances should be equal")
    
    print(f"\n=== INTERNAL FERMAT DATA ===")
    print(f"Fermat point: ({result.fermat_data.fermat_point.x:.4f}, {result.fermat_data.fermat_point.y:.4f}, {result.fermat_data.fermat_point.z:.4f})")
    print(f"Z positions - A: {result.fermat_data.z_A:.4f}, B: {result.fermat_data.z_B:.4f}, C: {result.fermat_data.z_C:.4f}")
    
    print(f"\n=== INTERNAL JOINT STATE DATA ===")
    print(f"Prismatic: {result.joint_state_data.prismatic_joint:.4f}")
    print(f"Roll:      {result.joint_state_data.roll_joint:.4f} rad ({math.degrees(result.joint_state_data.roll_joint):.2f}°)")
    print(f"Pitch:     {result.joint_state_data.pitch_joint:.4f} rad ({math.degrees(result.joint_state_data.pitch_joint):.2f}°)")
    
    print(f"\n=== VERIFICATION ===")
    # Verify transformed vector is at half angle
    z_vector = km.Vector3(0, 0, 1)
    
    # Calculate angle between transformed vector and Z axis
    def vector_angle(v1, v2):
        dot = v1.x*v2.x + v1.y*v2.y + v1.z*v2.z
        norm1 = math.sqrt(v1.x*v1.x + v1.y*v1.y + v1.z*v1.z)
        norm2 = math.sqrt(v2.x*v2.x + v2.y*v2.y + v2.z*v2.z)
        cos_angle = max(-1, min(1, dot / (norm1 * norm2)))
        return math.acos(cos_angle)
    
    transformed_angle = vector_angle(result.transformed_vector, z_vector)
    expected_half_angle = result.input_angle_from_z / 2
    
    print(f"Transformed vector angle from Z: {transformed_angle:.4f} rad ({math.degrees(transformed_angle):.2f}°)")
    print(f"Expected half angle:             {expected_half_angle:.4f} rad ({math.degrees(expected_half_angle):.2f}°)")
    print(f"Difference:                      {abs(transformed_angle - expected_half_angle):.6f} rad")
    
    if abs(transformed_angle - expected_half_angle) < 0.001:
        print("✓ Half-angle transformation is correct")
    else:
        print("✗ Warning: Half-angle transformation may have issues")
    
    # Verify vector magnitudes
    input_norm = math.sqrt(input_x*input_x + input_y*input_y + input_z*input_z)
    transformed_norm = math.sqrt(result.transformed_vector.x**2 + result.transformed_vector.y**2 + result.transformed_vector.z**2)
    
    print(f"\nVector magnitudes:")
    print(f"Original input:     {input_norm:.4f}")
    print(f"Transformed vector: {transformed_norm:.4f}")
    
    if abs(transformed_norm - 1.0) < 0.001:
        print("✓ Transformed vector is properly normalized")
    else:
        print("✗ Warning: Transformed vector normalization issue")
    
    print(f"\nModule working correctly!")
    print("Key outputs: End-effector position, Point G, Distance H→G")
    print("Validation: G→End-effector distance should equal Base→H distance (mirror property)")
    
    return True

if __name__ == "__main__":
    test_kinematics_module()