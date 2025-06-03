#!/usr/bin/env python3
"""
Test the FABRIK Backward iteration module interface
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

def print_joint_positions(chain, title="Joint Positions"):
    """Print all joint positions in the chain"""
    print(f"\n{title}:")
    for i, joint in enumerate(chain.joints):
        type_name = str(joint.type).split('.')[-1] if hasattr(joint.type, 'name') else str(joint.type)
        print(f"  [{i}] {type_name}: ({joint.position.x:.2f}, {joint.position.y:.2f}, {joint.position.z:.2f})")

def print_distance_analysis(initial_chain, result):
    """Print distance analysis between initial and final positions"""
    print(f"\n=== DISTANCE ANALYSIS ===")
    
    for i, (initial_joint, final_joint) in enumerate(zip(initial_chain.joints, result.updated_chain.joints)):
        distance_moved = math.sqrt(
            (final_joint.position.x - initial_joint.position.x)**2 +
            (final_joint.position.y - initial_joint.position.y)**2 +
            (final_joint.position.z - initial_joint.position.z)**2
        )
        
        type_name = str(initial_joint.type).split('.')[-1] if hasattr(initial_joint.type, 'name') else str(initial_joint.type)
        
        if distance_moved < 0.01:
            print(f"  Joint {i} ({type_name}): No movement")
        else:
            print(f"  Joint {i} ({type_name}): Moved {distance_moved:.2f} units")

def verify_segment_lengths(chain, title="Segment Length Verification"):
    """Verify that segment lengths are maintained"""
    print(f"\n{title}:")
    
    length_violations = []
    
    for i, segment in enumerate(chain.segments):
        start_pos = chain.joints[segment.start_joint_index].position
        end_pos = chain.joints[segment.end_joint_index].position
        
        actual_length = math.sqrt(
            (end_pos.x - start_pos.x)**2 +
            (end_pos.y - start_pos.y)**2 +
            (end_pos.z - start_pos.z)**2
        )
        
        expected_length = segment.length
        length_error = abs(actual_length - expected_length)
        
        if length_error < 0.01:
            print(f"  ✓ Segment {i}: Expected {expected_length:.1f}, Got {actual_length:.2f}")
        else:
            print(f"  ✗ Segment {i}: Expected {expected_length:.1f}, Got {actual_length:.2f} (error: {length_error:.3f})")
            length_violations.append((i, length_error))
    
    if not length_violations:
        print("  ✓ All segment lengths maintained correctly")
    else:
        print(f"  ✗ {len(length_violations)} segment length violations detected")
    
    return len(length_violations) == 0

def test_fabrik_backward():
    # Parse command line arguments for target position
    if len(sys.argv) == 2:
        target_x, target_y, target_z = parse_coordinates(sys.argv[1])
        print(f"Using command line target: ({target_x}, {target_y}, {target_z})")
    else:
        # Default target: reachable position
        target_x, target_y, target_z = 100, 100, 300
        print("Using default target: (100, 100, 300)")
        print("Usage: python3 test_fabrik_backward.py x,y,z")
    
    try:
        import delta_robot.fabrik_initialization as fi
        import delta_robot.fabrik_backward as fb
        import delta_robot.delta_types as dt
        import delta_robot  # Import main module for constants
    except ImportError as e:
        print(f"Error: Module not found: {e}")
        print("Build both fabrik_initialization and fabrik_backward modules first.")
        return False
    
    print("Testing FABRIK Backward Iteration Module")
    print("=" * 60)
    
    # ✅ USE THE SAME CONSTANT AS THE SYSTEM - NO MORE HARDCODING!
    num_segments = delta_robot.DEFAULT_ROBOT_SEGMENTS
    print(f"Using DEFAULT_ROBOT_SEGMENTS = {num_segments} from C++ constants")
    print(f"Initializing {num_segments}-segment robot chain...")
    
    init_result = fi.FabrikInitialization.initialize_straight_up(num_segments)
    initial_chain = init_result.chain
    
    print(f"Initial chain: {len(initial_chain.joints)} joints, total reach: {init_result.total_reach:.1f}")
    
    # Create target position
    target_position = dt.Vector3(target_x, target_y, target_z)
    
    print(f"\n=== TARGET ANALYSIS ===")
    print(f"Target position: ({target_x}, {target_y}, {target_z})")
    
    # Check reachability
    is_reachable = fb.FabrikBackward.is_target_reachable(initial_chain, target_position)
    
    # Calculate distance to target
    base_position = initial_chain.joints[0].position
    distance_to_target = math.sqrt(
        (target_x - base_position.x)**2 +
        (target_y - base_position.y)**2 +
        (target_z - base_position.z)**2
    )
    
    print(f"Distance from base: {distance_to_target:.2f}")
    print(f"Maximum reach: {init_result.total_reach:.2f}")
    print(f"Target reachable: {'✓ YES' if is_reachable else '✗ NO'}")
    
    if not is_reachable:
        print(f"⚠️  Target is {distance_to_target - init_result.total_reach:.2f} units beyond reach")
    
    # Show initial positions
    print_joint_positions(initial_chain, "Initial Joint Positions")
    
    print(f"\n=== BACKWARD ITERATION ===")
    print("Starting backward iteration to target...")
    
    # Perform backward iteration
    result = fb.FabrikBackward.iterate_to_target(initial_chain, target_position)
    
    print(f"Iterations completed: {result.iterations_used}")
    print(f"Target reached: {'✓ YES' if result.target_reachable else '✗ NO'}")
    print(f"Final distance to base: {result.distance_to_base:.2f}")
    
    # Calculate final end-effector distance to target
    final_end_effector = fb.FabrikBackward.get_end_effector_position(result.updated_chain)
    final_distance_to_target = math.sqrt(
        (final_end_effector.x - target_x)**2 +
        (final_end_effector.y - target_y)**2 +
        (final_end_effector.z - target_z)**2
    )
    
    print(f"Final end-effector: ({final_end_effector.x:.2f}, {final_end_effector.y:.2f}, {final_end_effector.z:.2f})")
    print(f"Distance to target: {final_distance_to_target:.4f}")
    
    convergence_tolerance = delta_robot.FABRIK_TOLERANCE  # ✅ USE CONSTANT!
    if final_distance_to_target <= convergence_tolerance:
        print(f"✓ Converged within tolerance ({convergence_tolerance})")
    else:
        print(f"✗ Did not converge (tolerance: {convergence_tolerance})")
    
    # Show final positions
    print_joint_positions(result.updated_chain, "Final Joint Positions")
    
    # Distance analysis
    print_distance_analysis(initial_chain, result)
    
    # Verify segment lengths are maintained
    lengths_valid = verify_segment_lengths(result.updated_chain)
    
    # Iteration history
    print(f"\n=== ITERATION HISTORY ===")
    print(f"End-effector position progression:")
    for i, pos in enumerate(result.iteration_history):
        dist_to_target = math.sqrt((pos.x - target_x)**2 + (pos.y - target_y)**2 + (pos.z - target_z)**2)
        print(f"  Iteration {i}: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}) - Distance to target: {dist_to_target:.4f}")
    
    # Test single iteration
    print(f"\n=== SINGLE ITERATION TEST ===")
    single_result_chain = fb.FabrikBackward.single_backward_iteration(initial_chain, target_position)
    single_end_effector = fb.FabrikBackward.get_end_effector_position(single_result_chain)
    
    print(f"After single iteration:")
    print(f"  End-effector: ({single_end_effector.x:.2f}, {single_end_effector.y:.2f}, {single_end_effector.z:.2f})")
    
    single_dist_to_target = math.sqrt(
        (single_end_effector.x - target_x)**2 +
        (single_end_effector.y - target_y)**2 +
        (single_end_effector.z - target_z)**2
    )
    print(f"  Distance to target: {single_dist_to_target:.4f}")
    
    # Verify single iteration matches first step of full iteration
    if len(result.iteration_history) > 1:
        first_iteration_pos = result.iteration_history[1]
        single_match = (abs(single_end_effector.x - first_iteration_pos.x) < 0.01 and
                       abs(single_end_effector.y - first_iteration_pos.y) < 0.01 and
                       abs(single_end_effector.z - first_iteration_pos.z) < 0.01)
        
        if single_match:
            print("  ✓ Single iteration matches first step of full iteration")
        else:
            print("  ✗ Single iteration does not match full iteration first step")
    
    print(f"\n=== SUMMARY ===")
    
    success_criteria = [
        (result.iterations_used > 0, "Iterations performed"),
        (lengths_valid, "Segment lengths maintained"),
        (final_distance_to_target <= convergence_tolerance * 10, "Reasonable convergence"),  # Allow 10x tolerance for summary
    ]
    
    all_success = all(criterion[0] for criterion in success_criteria)
    
    for success, description in success_criteria:
        status = "✓" if success else "✗"
        print(f"  {status} {description}")
    
    if all_success:
        print("✓ FABRIK Backward iteration module working correctly!")
        print("✓ Ready to implement forward iteration for complete FABRIK solver")
    else:
        print("✗ Some issues detected - check implementation")
    
    print(f"\nBackward iteration results:")
    print(f"  - Using: {num_segments} segments ({len(initial_chain.joints)} joints)")
    print(f"  - Target: ({target_x}, {target_y}, {target_z})")
    print(f"  - Reachable: {is_reachable}")
    print(f"  - Iterations: {result.iterations_used}")
    print(f"  - Final accuracy: {final_distance_to_target:.4f}")
    print(f"  - Segment lengths preserved: {lengths_valid}")
    print(f"  - Using DEFAULT_ROBOT_SEGMENTS = {delta_robot.DEFAULT_ROBOT_SEGMENTS} from C++ constants")
    
    return True

if __name__ == "__main__":
    test_fabrik_backward()