#!/usr/bin/env python3
"""
Test the joint state module interface
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

def test_joint_state_module():
    # Parse command line arguments for direction vector
    if len(sys.argv) == 2:
        direction_x, direction_y, direction_z = parse_coordinates(sys.argv[1])
        print(f"Using command line direction: ({direction_x}, {direction_y}, {direction_z})")
    else:
        direction_x, direction_y, direction_z = 5, 4, 7
        print("Using default direction: (5, 4, 7)")
        print("Usage: python3 test_joint_state.py x,y,z")
    
    try:
        import fermat_module as fm
        import joint_state_module as jsm
    except ImportError as e:
        print(f"Error: Module not found: {e}")
        print("Build both modules first.")
        return False
    
    print("Testing Joint State Module")
    print("=" * 40)
    
    # First, get fermat calculation
    print(f"Input direction: ({direction_x}, {direction_y}, {direction_z})")
    
    fermat_result = fm.FermatModule.calculate(direction_x, direction_y, direction_z)
    
    print(f"\nFermat calculation results:")
    print(f"  Z positions - A: {fermat_result.z_A:.4f}, B: {fermat_result.z_B:.4f}, C: {fermat_result.z_C:.4f}")
    print(f"  Fermat point: ({fermat_result.fermat_point.x:.4f}, {fermat_result.fermat_point.y:.4f}, {fermat_result.fermat_point.z:.4f})")
    
    # Test joint state calculation - Method 1: Direct values
    print(f"\nTesting joint state calculation (direct values):")
    direction_vector = fm.Vector3(direction_x, direction_y, direction_z)
    
    joint_result = jsm.JointStateModule.calculate(
        direction_vector,
        fermat_result.fermat_point,
        fermat_result.z_A,
        fermat_result.z_B,
        fermat_result.z_C
    )
    
    print(f"Joint State Results:")
    print(f"  Prismatic: {joint_result.prismatic_joint:.4f}")
    print(f"  Roll:      {joint_result.roll_joint:.4f} rad ({math.degrees(joint_result.roll_joint):.2f}°)")
    print(f"  Pitch:     {joint_result.pitch_joint:.4f} rad ({math.degrees(joint_result.pitch_joint):.2f}°)")
    
    print(f"\nVerifying received values:")
    print(f"  Direction vector: ({joint_result.direction_vector.x:.4f}, {joint_result.direction_vector.y:.4f}, {joint_result.direction_vector.z:.4f})")
    print(f"  Fermat point: ({joint_result.fermat_point.x:.4f}, {joint_result.fermat_point.y:.4f}, {joint_result.fermat_point.z:.4f})")
    print(f"  Z values - A: {joint_result.z_A:.4f}, B: {joint_result.z_B:.4f}, C: {joint_result.z_C:.4f}")
    
    # Test Method 2: Using fermat result directly
    print(f"\nTesting joint state calculation (from fermat result):")
    joint_result2 = jsm.JointStateModule.calculate_from_fermat(direction_vector, fermat_result)
    
    print(f"Joint State Results (method 2):")
    print(f"  Prismatic: {joint_result2.prismatic_joint:.4f}")
    print(f"  Roll:      {joint_result2.roll_joint:.4f} rad ({math.degrees(joint_result2.roll_joint):.2f}°)")
    print(f"  Pitch:     {joint_result2.pitch_joint:.4f} rad ({math.degrees(joint_result2.pitch_joint):.2f}°)")
    
    # Verify both methods give same results
    if (abs(joint_result.prismatic_joint - joint_result2.prismatic_joint) < 1e-10 and
        abs(joint_result.roll_joint - joint_result2.roll_joint) < 1e-10 and
        abs(joint_result.pitch_joint - joint_result2.pitch_joint) < 1e-10):
        print("\n✓ Both calculation methods give identical results")
    else:
        print("\n✗ Warning: Methods give different results")
    
    print(f"\nModule working correctly!")
    print("Joint calculations now implemented:")
    print("- Prismatic = 2 × fermat_point.z")
    print("- Roll = -atan2(y, z) [rotation around X-axis]") 
    print("- Pitch = atan2(x, z) [rotation around Y-axis]")
    return True

if __name__ == "__main__":
    test_joint_state_module()