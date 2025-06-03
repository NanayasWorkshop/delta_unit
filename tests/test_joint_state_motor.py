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
        import delta_robot.kinematics_module as kinematics
        import delta_robot.orientation_module as orientation
    except ImportError as e:
        print(f"Error: {e}")
        print("Build the modules first: python setup.py build_ext --inplace")
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
    
    # NEW: Get first segment end-effector and run through kinematics + orientation
    if result.fabrik_result.segment_end_effectors:
        first_segment = result.fabrik_result.segment_end_effectors[0]
        seg_pos = first_segment.end_effector_position
        
        print("=" * 50)
        print("FIRST SEGMENT ANALYSIS:")
        print(f"Segment 1 End-Effector: ({seg_pos.x:.3f}, {seg_pos.y:.3f}, {seg_pos.z:.3f})")
        
        # Run through kinematics module
        kinematics_result = kinematics.KinematicsModule.calculate(seg_pos.x, seg_pos.y, seg_pos.z)
        
        print(f"\nA B C Z Values:")
        print(f"  z_A: {kinematics_result.fermat_data.z_A:.6f}")
        print(f"  z_B: {kinematics_result.fermat_data.z_B:.6f}")
        print(f"  z_C: {kinematics_result.fermat_data.z_C:.6f}")
        
        print(f"\nJoint States:")
        print(f"  Prismatic: {kinematics_result.joint_state_data.prismatic_joint:.6f}")
        print(f"  Roll:      {kinematics_result.joint_state_data.roll_joint:.6f} rad ({kinematics_result.joint_state_data.roll_joint * 180 / 3.14159:.2f}°)")
        print(f"  Pitch:     {kinematics_result.joint_state_data.pitch_joint:.6f} rad ({kinematics_result.joint_state_data.pitch_joint * 180 / 3.14159:.2f}°)")
        
        # Run through orientation module
        orientation_result = orientation.OrientationModule.calculate_from_kinematics(kinematics_result)
        
        print(f"\nEnd-Effector UVW Orientation:")
        final_frame = orientation_result.final_frame
        print(f"  U-axis: ({final_frame.u_axis.x:.6f}, {final_frame.u_axis.y:.6f}, {final_frame.u_axis.z:.6f})")
        print(f"  V-axis: ({final_frame.v_axis.x:.6f}, {final_frame.v_axis.y:.6f}, {final_frame.v_axis.z:.6f})")
        print(f"  W-axis: ({final_frame.w_axis.x:.6f}, {final_frame.w_axis.y:.6f}, {final_frame.w_axis.z:.6f})")
    
    print("=" * 50)

if __name__ == "__main__":
    main()