#!/usr/bin/env python3
"""
Simple Joint State Motor Test - Just target and end-effector position
"""

import sys
import os

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def parse_coordinates(coord_str):
    """Parse coordinate string like '545,554,122' into x,y,z values"""
    try:
        coords = [float(x.strip()) for x in coord_str.split(',')]
        if len(coords) != 3:
            raise ValueError("Expected 3 coordinates")
        return coords
    except ValueError as e:
        print(f"Error parsing coordinates '{coord_str}': {e}")
        print("Expected format: x,y,z (e.g., 545,554,122)")
        sys.exit(1)

def main():
    # Parse command line arguments for target
    if len(sys.argv) == 2:
        target_x, target_y, target_z = parse_coordinates(sys.argv[1])
        print(f"Target: ({target_x}, {target_y}, {target_z})")
    else:
        target_x, target_y, target_z = 100, 50, 300
        print(f"Default target: ({target_x}, {target_y}, {target_z})")
        print("Usage: python3 test_joint_state_motor_simple.py x,y,z")
    
    try:
        import delta_robot.joint_state_motor as motor
    except ImportError as e:
        print(f"Error: {e}")
        print("Build the module first: python setup.py build_ext --inplace")
        return
    
    print("=" * 50)
    
    # Calculate motor positions
    result = motor.JointStateMotorModule.calculate_motors(target_x, target_y, target_z)
    
    print("=" * 50)
    print("RESULTS:")
    print(f"Target:    ({result.target_position.x:.3f}, {result.target_position.y:.3f}, {result.target_position.z:.3f})")
    print(f"Achieved:  ({result.achieved_end_effector.x:.3f}, {result.achieved_end_effector.y:.3f}, {result.achieved_end_effector.z:.3f})")
    print(f"Converged: {'YES' if result.fabrik_converged else 'NO'}")
    print(f"Error:     {result.fabrik_error:.6f}")
    print("=" * 50)

if __name__ == "__main__":
    main()