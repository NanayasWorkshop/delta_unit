#!/usr/bin/env python3
"""
Debug test for FABRIK Backward - step by step analysis
"""

import sys
import os
import math
# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

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

def debug_backward_step_by_step():
    """Debug the backward iteration step by step"""
    
    try:
        import delta_robot.fabrik_initialization as fi
        import delta_robot.fabrik_backward as fb
        import delta_robot.delta_types as dt
    except ImportError as e:
        print(f"Error: {e}")
        return False
    
    # Initialize
    init_result = fi.FabrikInitialization.initialize_straight_up(3)
    initial_chain = init_result.chain
    target = dt.Vector3(100, 100, 300)
    
    print("FABRIK BACKWARD DEBUG - STEP BY STEP")
    print("=" * 80)
    
    # Show initial state
    print_detailed_chain(initial_chain, "INITIAL CHAIN")
    
    print(f"\nTARGET: ({target.x}, {target.y}, {target.z})")
    
    # Manual step-by-step backward iteration
    print("\n" + "="*80)
    print("MANUAL STEP-BY-STEP BACKWARD ITERATION")
    print("="*80)
    
    # Copy the chain
    current_chain = initial_chain
    
    # Step 1: Set end-effector to target
    print("\nSTEP 1: Set end-effector to target")
    print(f"  Before: Joint[4] = ({current_chain.joints[4].position.x:.2f}, {current_chain.joints[4].position.y:.2f}, {current_chain.joints[4].position.z:.2f})")
    
    # Create a list to track positions (can't deepcopy C++ objects)
    # We'll manually track the joint positions
    joint_positions = []
    for joint in current_chain.joints:
        joint_positions.append(dt.Vector3(joint.position.x, joint.position.y, joint.position.z))
    
    # Step 1: Set end-effector to target
    joint_positions[4] = target
    
    print(f"  After:  Joint[4] = ({joint_positions[4].x:.2f}, {joint_positions[4].y:.2f}, {joint_positions[4].z:.2f})")
    
    # Now work backwards through joints 3, 2, 1, 0
    for joint_idx in [3, 2, 1, 0]:
        print(f"\n" + "-"*50)
        print(f"STEP {5-joint_idx}: Processing Joint[{joint_idx}]")
        print("-"*50)
        
        # Get current positions
        current_joint = current_chain.joints[joint_idx]
        next_joint_pos = joint_positions[joint_idx + 1]  # Use our updated position
        segment = current_chain.segments[joint_idx]
        
        print(f"Current Joint[{joint_idx}]: ({joint_positions[joint_idx].x:.2f}, {joint_positions[joint_idx].y:.2f}, {joint_positions[joint_idx].z:.2f})")
        print(f"Next Joint[{joint_idx+1}]: ({next_joint_pos.x:.2f}, {next_joint_pos.y:.2f}, {next_joint_pos.z:.2f})")
        print(f"Required segment length: {segment.length:.2f}")
        
        # Calculate unconstrained position (move to maintain segment length)
        direction = dt.Vector3(
            joint_positions[joint_idx].x - next_joint_pos.x,
            joint_positions[joint_idx].y - next_joint_pos.y,
            joint_positions[joint_idx].z - next_joint_pos.z
        )
        
        current_distance = math.sqrt(direction.x**2 + direction.y**2 + direction.z**2)
        print(f"Current distance: {current_distance:.2f}")
        
        if current_distance < 1e-10:
            # Edge case
            unconstrained_pos = dt.Vector3(
                next_joint_pos.x + segment.length,
                next_joint_pos.y,
                next_joint_pos.z
            )
        else:
            # Normalize and scale
            scale = segment.length / current_distance
            unconstrained_pos = dt.Vector3(
                next_joint_pos.x + direction.x * scale,
                next_joint_pos.y + direction.y * scale,
                next_joint_pos.z + direction.z * scale
            )
        
        print(f"Unconstrained position: ({unconstrained_pos.x:.2f}, {unconstrained_pos.y:.2f}, {unconstrained_pos.z:.2f})")
        
        # Check distance from unconstrained position to next joint
        unconstrained_distance = calculate_distance(unconstrained_pos, next_joint_pos)
        print(f"Unconstrained distance to next: {unconstrained_distance:.2f} (should be {segment.length:.2f})")
        
        # Apply constraints if this is a spherical joint
        if str(current_joint.type).endswith('SPHERICAL_120'):
            print(f"  → Applying spherical constraint (120°)")
            
            # Get previous joint for constraint calculation
            if joint_idx > 0:
                prev_joint_pos = joint_positions[joint_idx - 1]
                print(f"  → Previous Joint[{joint_idx-1}]: ({prev_joint_pos.x:.2f}, {prev_joint_pos.y:.2f}, {prev_joint_pos.z:.2f})")
                
                # Here we would apply the constraint, but let's see what the C++ code returns
                # For now, let's just use the unconstrained position to see the base problem
                final_pos = unconstrained_pos
            else:
                print(f"  → No previous joint (this is the base)")
                final_pos = unconstrained_pos
        else:
            print(f"  → No constraint (joint type: {current_joint.type})")
            final_pos = unconstrained_pos
        
        # Update the position
        print(f"Final position: ({final_pos.x:.2f}, {final_pos.y:.2f}, {final_pos.z:.2f})")
        joint_positions[joint_idx] = final_pos
        
        # Verify the segment length after update
        actual_distance = calculate_distance(joint_positions[joint_idx], joint_positions[joint_idx + 1])
        print(f"Actual segment length after update: {actual_distance:.2f}")
        
        if abs(actual_distance - segment.length) > 0.01:
            print(f"  ⚠️  SEGMENT LENGTH VIOLATION: {abs(actual_distance - segment.length):.3f}")
        else:
            print(f"  ✓ Segment length preserved")
    
    # Create a simple chain representation for display
    class SimpleChain:
        def __init__(self, positions, original_chain):
            self.joints = []
            for i, pos in enumerate(positions):
                class SimpleJoint:
                    def __init__(self, pos, joint_type):
                        self.position = pos
                        self.type = joint_type
                self.joints.append(SimpleJoint(pos, original_chain.joints[i].type))
            self.segments = original_chain.segments
    
    manual_result = SimpleChain(joint_positions, current_chain)
    print_detailed_chain(manual_result, "MANUAL RESULT")
    
    # Compare with actual C++ implementation
    print("\n" + "="*80)
    print("COMPARING WITH C++ IMPLEMENTATION")
    print("="*80)
    
    result = fb.FabrikBackward.single_backward_iteration(initial_chain, target)
    print_detailed_chain(result, "C++ RESULT")
    
    # Compare positions
    print("\nPOSITION COMPARISON:")
    for i in range(len(joint_positions)):
        manual_pos = joint_positions[i]
        cpp_pos = result.joints[i].position
        
        diff = math.sqrt(
            (manual_pos.x - cpp_pos.x)**2 +
            (manual_pos.y - cpp_pos.y)**2 +
            (manual_pos.z - cpp_pos.z)**2
        )
        
        print(f"  Joint[{i}]: Manual=({manual_pos.x:.2f}, {manual_pos.y:.2f}, {manual_pos.z:.2f}) "
              f"C++=({cpp_pos.x:.2f}, {cpp_pos.y:.2f}, {cpp_pos.z:.2f}) Diff={diff:.3f}")
    
    return True

if __name__ == "__main__":
    debug_backward_step_by_step()