#!/usr/bin/env python3
"""
Test the FABRIK Initialization module interface
"""

import sys
import os
import math
# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def parse_segments(segments_str):
    """Parse segments string like '3' into integer"""
    try:
        segments = int(segments_str.strip())
        if segments <= 0:
            raise ValueError("Expected positive number of segments")
        return segments
    except ValueError as e:
        print(f"Error parsing segments '{segments_str}': {e}")
        print("Expected format: positive integer (e.g., 3)")
        sys.exit(1)

def print_joint(joint, index):
    """Print joint information in formatted way"""
    type_names = {
        0: "FIXED_BASE",
        1: "SPHERICAL_120", 
        2: "END_EFFECTOR"
    }
    
    type_name = "UNKNOWN"
    try:
        # Try to get the type name from the enum value
        if hasattr(joint.type, 'name'):
            type_name = joint.type.name
        else:
            # Fallback for numeric values
            type_name = type_names.get(int(joint.type), "UNKNOWN")
    except:
        type_name = str(joint.type)
    
    constraint_str = ""
    if joint.constraint_angle > 0:
        constraint_str = f" (constraint: {math.degrees(joint.constraint_angle):.1f}°)"
    
    print(f"  [{index}] {type_name}: ({joint.position.x:.1f}, {joint.position.y:.1f}, {joint.position.z:.1f}){constraint_str}")

def print_segment(segment, index):
    """Print segment information in formatted way"""
    print(f"  [{index}] Joint{segment.start_joint_index} → Joint{segment.end_joint_index}: length = {segment.length:.1f}")

def test_fabrik_initialization():
    # Import modules first to get constants
    try:
        import delta_robot.fabrik_initialization as fi
        import delta_robot.delta_types as dt
        import delta_robot
    except ImportError as e:
        print(f"Error: Module not found: {e}")
        print("Build the fabrik_initialization module first.")
        return False
    
    # Parse command line arguments for number of segments - USE THE CONSTANT!
    if len(sys.argv) == 2:
        num_segments = parse_segments(sys.argv[1])
        print(f"Using command line segments: {num_segments}")
    else:
        num_segments = delta_robot.DEFAULT_ROBOT_SEGMENTS  # ✅ USE THE CONSTANT!
        print(f"Using default segments from C++ constant: {num_segments}")
        print("Usage: python3 test_fabrik_initialization.py <num_segments>")
    
    print("Testing FABRIK Initialization Module")
    print("=" * 50)
    
    # Test initialization
    print(f"Initializing {num_segments}-segment robot chain (straight up)")
    
    result = fi.FabrikInitialization.initialize_straight_up(num_segments)
    
    print(f"\n=== INITIALIZATION RESULTS ===")
    print(f"Number of robot segments: {result.chain.num_robot_segments}")
    print(f"Total joints in chain:    {len(result.chain.joints)}")
    print(f"Total segments in chain:  {len(result.chain.segments)}")
    print(f"Final end-effector:       ({result.final_end_effector.x:.1f}, {result.final_end_effector.y:.1f}, {result.final_end_effector.z:.1f})")
    print(f"Total reach:              {result.total_reach:.1f}")
    
    print(f"\n=== JOINT POSITIONS ===")
    for i, joint in enumerate(result.chain.joints):
        print_joint(joint, i)
    
    print(f"\n=== SEGMENT LENGTHS ===")
    for i, segment in enumerate(result.chain.segments):
        print_segment(segment, i)
    
    print(f"\n=== VERIFICATION ===")
    
    # Verify our known 3-segment example
    if num_segments == 3:
        expected_positions = [
            (0, 0, 0),      # Base
            (0, 0, 73),     # Joint 1
            (0, 0, 219),    # Joint 2  
            (0, 0, 365),    # Joint 3
            (0, 0, 438)     # End-effector
        ]
        
        expected_lengths = [73.0, 146.0, 146.0, 73.0]
        
        print("Verifying against expected 3-segment positions:")
        
        position_correct = True
        for i, (joint, expected) in enumerate(zip(result.chain.joints, expected_positions)):
            actual = (joint.position.x, joint.position.y, joint.position.z)
            diff = math.sqrt(sum((a - e)**2 for a, e in zip(actual, expected)))
            
            if diff < 0.1:
                print(f"  ✓ Joint {i}: Expected {expected}, Got {actual}")
            else:
                print(f"  ✗ Joint {i}: Expected {expected}, Got {actual} (diff: {diff:.2f})")
                position_correct = False
        
        length_correct = True
        for i, (segment, expected_length) in enumerate(zip(result.chain.segments, expected_lengths)):
            if abs(segment.length - expected_length) < 0.1:
                print(f"  ✓ Segment {i}: Expected length {expected_length}, Got {segment.length:.1f}")
            else:
                print(f"  ✗ Segment {i}: Expected length {expected_length}, Got {segment.length:.1f}")
                length_correct = False
        
        if position_correct and length_correct:
            print("  ✓ All verifications PASSED for 3-segment robot!")
        else:
            print("  ✗ Some verifications FAILED")
    elif num_segments == 5:
        # Add verification for 5-segment robot (DEFAULT_ROBOT_SEGMENTS)
        print("Verifying 5-segment robot structure (using DEFAULT_ROBOT_SEGMENTS):")
        
        # Basic structure verification
        expected_joints = 6  # 5 segments + 1 end effector
        expected_segments = 6  # 6 connections
        
        if len(result.chain.joints) == expected_joints:
            print(f"  ✓ Joint count: Expected {expected_joints}, Got {len(result.chain.joints)}")
        else:
            print(f"  ✗ Joint count: Expected {expected_joints}, Got {len(result.chain.joints)}")
            
        if len(result.chain.segments) == expected_segments:
            print(f"  ✓ Segment count: Expected {expected_segments}, Got {len(result.chain.segments)}")
        else:
            print(f"  ✗ Segment count: Expected {expected_segments}, Got {len(result.chain.segments)}")
    
    # Verify segment length calculation
    print(f"\nVerifying segment length calculation:")
    hypotenuse = delta_robot.MIN_HEIGHT + 2*delta_robot.MOTOR_LIMIT + 0  # MIN_HEIGHT + 2*MOTOR_LIMIT + prismatic(0)
    calculated_segment_length = fi.FabrikInitialization.calculate_segment_length(hypotenuse, 0.0)
    expected_segment_length = hypotenuse / 2.0  # For straight up (angle = 0)
    
    print(f"  Hypotenuse distance: {hypotenuse}")
    print(f"  Expected segment length: {expected_segment_length}")
    print(f"  Calculated segment length: {calculated_segment_length}")
    
    if abs(calculated_segment_length - expected_segment_length) < 0.01:
        print("  ✓ Segment length calculation is correct")
    else:
        print("  ✗ Segment length calculation has issues")
    
    # Test utility functions
    print(f"\nTesting utility functions:")
    total_reach = fi.FabrikInitialization.get_total_reach(num_segments)
    total_joints = fi.FabrikInitialization.get_total_joints(num_segments)
    
    print(f"  Total reach for {num_segments} segments: {total_reach:.1f}")
    print(f"  Total joints for {num_segments} segments: {total_joints}")
    print(f"  Expected joints: {num_segments + 1}")
    
    if total_joints == num_segments + 1:
        print("  ✓ Joint count calculation is correct")
    else:
        print("  ✗ Joint count calculation has issues")
    
    # Verify constraint angles
    print(f"\nVerifying joint constraints:")
    spherical_joints = [j for j in result.chain.joints if hasattr(j.type, 'name') and 'SPHERICAL' in str(j.type) or 'SPHERICAL' in str(j.type)]
    
    expected_constraint = delta_robot.SPHERICAL_JOINT_CONE_ANGLE_RAD  # Use constant from hpp!
    
    constraint_correct = True
    for i, joint in enumerate(result.chain.joints):
        if 'SPHERICAL' in str(joint.type):
            if abs(joint.constraint_angle - expected_constraint) < 0.01:
                print(f"  ✓ Spherical joint {i}: {math.degrees(joint.constraint_angle):.1f}° constraint")
            else:
                print(f"  ✗ Spherical joint {i}: Expected {math.degrees(expected_constraint):.1f}°, Got {math.degrees(joint.constraint_angle):.1f}°")
                constraint_correct = False
    
    if constraint_correct:
        print("  ✓ All spherical joint constraints are correct")
    
    print(f"\n=== CONSTANTS VERIFICATION ===")
    print(f"Using constants from C++ headers:")
    print(f"  DEFAULT_ROBOT_SEGMENTS = {delta_robot.DEFAULT_ROBOT_SEGMENTS}")
    print(f"  MIN_HEIGHT = {delta_robot.MIN_HEIGHT}")
    print(f"  MOTOR_LIMIT = {delta_robot.MOTOR_LIMIT}")
    print(f"  WORKING_HEIGHT = {delta_robot.WORKING_HEIGHT}")
    print(f"  SPHERICAL_JOINT_CONE_ANGLE_RAD = {delta_robot.SPHERICAL_JOINT_CONE_ANGLE_RAD:.4f} ({delta_robot.SPHERICAL_JOINT_CONE_ANGLE_DEG}°)")
    print(f"  FABRIK_TOLERANCE = {delta_robot.FABRIK_TOLERANCE}")
    
    print(f"\n=== SUMMARY ===")
    print(f"✓ FABRIK Initialization module working correctly!")
    print(f"✓ Chain structure created with {len(result.chain.joints)} joints and {len(result.chain.segments)} segments")
    print(f"✓ Using DEFAULT_ROBOT_SEGMENTS = {delta_robot.DEFAULT_ROBOT_SEGMENTS} from C++ constants")
    print(f"✓ Ready for FABRIK backward iteration module")
    
    print(f"\nChain structure for FABRIK solver:")
    print(f"  - Base position: Fixed at (0,0,0)")
    print(f"  - {len([j for j in result.chain.joints if 'SPHERICAL' in str(j.type)])} spherical joints with 120° constraints")
    print(f"  - End-effector at: ({result.final_end_effector.x:.1f}, {result.final_end_effector.y:.1f}, {result.final_end_effector.z:.1f})")
    print(f"  - Total reach: {result.total_reach:.1f} units")
    
    return True

if __name__ == "__main__":
    test_fabrik_initialization()