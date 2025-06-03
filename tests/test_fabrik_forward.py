#!/usr/bin/env python3
"""
Test the FABRIK Forward iteration module interface - UPDATED with constants
"""

import sys
import os
import math
# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def parse_coordinates(coord_str):
    """Parse coordinate string like '100,50,200' into x,y,z values"""
    try:
        coords = [float(x.strip()) for x in coord_str.split(',')]
        if len(coords) != 3:
            raise ValueError("Expected 3 coordinates")
        return coords
    except ValueError as e:
        print(f"Error parsing coordinates '{coord_str}': {e}")
        print("Expected format: x,y,z (e.g., 100,50,200)")
        sys.exit(1)

def calculate_distance(pos1, pos2):
    """Calculate Euclidean distance between two positions"""
    return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z - pos2.z)**2)

def print_detailed_chain(chain, title="Chain Analysis"):
    """Print detailed chain analysis"""
    print(f"\n{title}:")
    print("=" * 60)
    
    # Print joints
    print("JOINTS:")
    for i, joint in enumerate(chain.joints):
        type_name = str(joint.type).split('.')[-1] if hasattr(joint.type, 'name') else str(joint.type)
        print(f"  [{i}] {type_name}: ({joint.position.x:.2f}, {joint.position.y:.2f}, {joint.position.z:.2f})")
    
    # Print segments and verify lengths
    print("\nSEGMENTS:")
    for i, segment in enumerate(chain.segments):
        start_pos = chain.joints[segment.start_joint_index].position
        end_pos = chain.joints[segment.end_joint_index].position
        
        actual_length = calculate_distance(start_pos, end_pos)
        expected_length = segment.length
        error = abs(actual_length - expected_length)
        
        status = "✓" if error < 0.01 else "✗"
        print(f"  [{i}] Joint{segment.start_joint_index}→Joint{segment.end_joint_index}: "
              f"Expected={expected_length:.1f}, Actual={actual_length:.2f}, Error={error:.3f} {status}")

def test_fabrik_forward():
    # Parse command line arguments for target position
    if len(sys.argv) == 2:
        target_x, target_y, target_z = parse_coordinates(sys.argv[1])
        print(f"Using command line target: ({target_x}, {target_y}, {target_z})")
    else:
        # Default target: reachable position
        target_x, target_y, target_z = 100, 100, 300
        print("Using default target: (100, 100, 300)")
        print("Usage: python3 test_fabrik_forward.py x,y,z")
    
    try:
        import delta_robot.fabrik_initialization as fi
        import delta_robot.fabrik_backward as fb
        import delta_robot.fabrik_forward as ff
        import delta_robot.delta_types as dt
        import delta_robot  # Import main module for constants
    except ImportError as e:
        print(f"Error: Module not found: {e}")
        print("Build fabrik_initialization, fabrik_backward, and fabrik_forward modules first.")
        return False
    
    print("Testing FABRIK Forward Iteration Module")
    print("=" * 80)
    
    # ✅ USE CONSTANTS FROM C++ HEADERS
    num_segments = delta_robot.DEFAULT_ROBOT_SEGMENTS
    tolerance = delta_robot.FABRIK_TOLERANCE
    max_iterations = delta_robot.FABRIK_MAX_ITERATIONS
    min_height = delta_robot.MIN_HEIGHT
    motor_limit = delta_robot.MOTOR_LIMIT
    working_height = delta_robot.WORKING_HEIGHT
    
    print(f"Using C++ constants:")
    print(f"  DEFAULT_ROBOT_SEGMENTS = {num_segments}")
    print(f"  FABRIK_TOLERANCE = {tolerance}")
    print(f"  FABRIK_MAX_ITERATIONS = {max_iterations}")
    print(f"  MIN_HEIGHT = {min_height}")
    print(f"  MOTOR_LIMIT = {motor_limit}")
    print(f"  WORKING_HEIGHT = {working_height}")
    
    # Initialize robot chain
    print(f"Initializing {num_segments}-segment robot chain...")
    init_result = fi.FabrikInitialization.initialize_straight_up(num_segments)
    initial_chain = init_result.chain
    
    print(f"Initial chain: {len(initial_chain.joints)} joints, total reach: {init_result.total_reach:.1f}")
    
    # Create target position
    target_position = dt.Vector3(target_x, target_y, target_z)
    
    print(f"\n=== TARGET ANALYSIS ===")
    print(f"Target position: ({target_x}, {target_y}, {target_z})")
    
    # Show initial positions
    print_detailed_chain(initial_chain, "INITIAL CHAIN")
    
    print(f"\n=== BACKWARD ITERATION ===")
    print("Running backward iteration to target...")
    
    # Perform backward iteration with constants
    backward_result = fb.FabrikBackward.iterate_to_target(
        initial_chain, 
        target_position,
        tolerance=tolerance,
        max_iterations=max_iterations
    )
    
    print(f"Backward iterations: {backward_result.iterations_used}/{max_iterations}")
    print(f"Backward converged: {'✓ YES' if backward_result.target_reachable else '✗ NO'}")
    
    # Show backward result
    print_detailed_chain(backward_result.updated_chain, "BACKWARD RESULT")
    
    print(f"\n=== FORWARD ITERATION ===")
    print("Running forward iteration to fix base...")
    
    # Test segment length calculation
    print("\nCalculating new segment lengths...")
    new_lengths = ff.FabrikForward.calculate_new_segment_lengths(backward_result.updated_chain)
    
    print(f"Original segment lengths: {[seg.length for seg in initial_chain.segments]}")
    print(f"Recalculated lengths: {[f'{length:.2f}' for length in new_lengths]}")
    
    # Perform forward iteration with constants
    forward_result = ff.FabrikForward.iterate_from_base(
        backward_result.updated_chain,
        tolerance=tolerance,
        max_iterations=max_iterations
    )
    
    print(f"Forward iterations: {forward_result.iterations_used}/{max_iterations}")
    print(f"Base at origin: {'✓ YES' if forward_result.constraints_satisfied else '✗ NO'}")
    print(f"Base position: ({forward_result.base_position.x:.3f}, {forward_result.base_position.y:.3f}, {forward_result.base_position.z:.3f})")
    
    # Show forward result
    print_detailed_chain(forward_result.updated_chain, "FORWARD RESULT")
    
    print(f"\n=== COMPARISON ANALYSIS ===")
    
    # Compare initial vs final positions
    initial_end_effector = initial_chain.joints[-1].position
    final_end_effector = forward_result.final_end_effector
    
    print(f"Initial end-effector: ({initial_end_effector.x:.2f}, {initial_end_effector.y:.2f}, {initial_end_effector.z:.2f})")
    print(f"Final end-effector: ({final_end_effector.x:.2f}, {final_end_effector.y:.2f}, {final_end_effector.z:.2f})")
    print(f"Target position: ({target_x}, {target_y}, {target_z})")
    
    # Calculate final accuracy
    final_distance_to_target = calculate_distance(final_end_effector, target_position)
    print(f"Final distance to target: {final_distance_to_target:.4f}")
    
    # Test segment length preservation
    print(f"\n=== SEGMENT LENGTH VERIFICATION ===")
    all_lengths_ok = True
    
    for i, segment in enumerate(forward_result.updated_chain.segments):
        start_pos = forward_result.updated_chain.joints[segment.start_joint_index].position
        end_pos = forward_result.updated_chain.joints[segment.end_joint_index].position
        
        actual_length = calculate_distance(start_pos, end_pos)
        expected_length = new_lengths[i]
        error = abs(actual_length - expected_length)
        
        if error < 0.01:
            print(f"  ✓ Segment {i}: Expected {expected_length:.2f}, Got {actual_length:.2f}")
        else:
            print(f"  ✗ Segment {i}: Expected {expected_length:.2f}, Got {actual_length:.2f} (error: {error:.3f})")
            all_lengths_ok = False
    
    # Test utility functions
    print(f"\n=== UTILITY FUNCTION TESTS ===")
    
    pairs_count = ff.get_direction_pairs_count(num_segments)
    indices = ff.get_fabrik_segment_indices(num_segments)
    
    print(f"Robot segments: {num_segments}")
    print(f"Direction pairs needed: {pairs_count}")
    print(f"FABRIK segment indices: {indices}")
    
    # Test H→G calculation with constants
    test_prismatic = 10.0
    h_to_g_dist = ff.calculate_h_to_g_distance(test_prismatic)
    expected_h_to_g = min_height + 2*motor_limit + test_prismatic  # Using constants
    
    print(f"H→G distance test: prismatic={test_prismatic} → h_to_g={h_to_g_dist:.1f} (expected: {expected_h_to_g:.1f})")
    
    # Verify constants are correctly used
    print(f"\n=== CONSTANTS VERIFICATION ===")
    print(f"H→G formula: MIN_HEIGHT + 2*MOTOR_LIMIT + prismatic")
    print(f"             {min_height} + 2*{motor_limit} + {test_prismatic} = {expected_h_to_g}")
    print(f"Calculated:  {h_to_g_dist:.1f}")
    print(f"Match: {'✓ YES' if abs(h_to_g_dist - expected_h_to_g) < 0.1 else '✗ NO'}")
    
    print(f"\n=== SUMMARY ===")
    
    success_criteria = [
        (backward_result.iterations_used > 0, "Backward iteration performed"),
        (forward_result.iterations_used > 0, "Forward iteration performed"),
        (forward_result.constraints_satisfied, "Base fixed at origin"),
        (all_lengths_ok, "Segment lengths preserved"),
        (final_distance_to_target <= tolerance * 100, "Reasonable final accuracy"),  # Allow 100x tolerance
    ]
    
    all_success = all(criterion[0] for criterion in success_criteria)
    
    for success, description in success_criteria:
        status = "✓" if success else "✗"
        print(f"  {status} {description}")
    
    if all_success:
        print("✓ FABRIK Forward iteration module working correctly!")
        print("✓ Ready to implement complete FABRIK solver!")
    else:
        print("✗ Some issues detected - check implementation")
    
    print(f"\nForward iteration results:")
    print(f"  - Using: {num_segments} segments ({len(initial_chain.joints)} joints)")
    print(f"  - Target: ({target_x}, {target_y}, {target_z})")
    print(f"  - Backward iterations: {backward_result.iterations_used}/{max_iterations}")
    print(f"  - Forward iterations: {forward_result.iterations_used}/{max_iterations}")
    print(f"  - Final accuracy: {final_distance_to_target:.4f}")
    print(f"  - Base at origin: {forward_result.constraints_satisfied}")
    print(f"  - Segment lengths preserved: {all_lengths_ok}")
    print(f"  - Using constants from C++ headers:")
    print(f"    * DEFAULT_ROBOT_SEGMENTS = {delta_robot.DEFAULT_ROBOT_SEGMENTS}")
    print(f"    * FABRIK_TOLERANCE = {delta_robot.FABRIK_TOLERANCE}")
    print(f"    * MIN_HEIGHT = {delta_robot.MIN_HEIGHT}")
    print(f"    * MOTOR_LIMIT = {delta_robot.MOTOR_LIMIT}")
    
    return True

if __name__ == "__main__":
    test_fabrik_forward()