#!/usr/bin/env python3
"""
Enhanced visual test for FABRIK Forward - UPDATED with constants
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
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

def vector_to_spherical(vector):
    """Convert a 3D vector to spherical coordinates (roll, pitch, magnitude)"""
    x, y, z = vector.x, vector.y, vector.z
    
    # Magnitude
    magnitude = math.sqrt(x*x + y*y + z*z)
    
    if magnitude < 1e-9:
        return 0.0, 0.0, 0.0
    
    # Pitch (elevation angle from XY plane) - range [-90, 90] degrees
    pitch_rad = math.asin(z / magnitude)
    pitch_deg = math.degrees(pitch_rad)
    
    # Roll (azimuth angle in XY plane from X axis) - range [-180, 180] degrees
    roll_rad = math.atan2(y, x)
    roll_deg = math.degrees(roll_rad)
    
    return roll_deg, pitch_deg, magnitude

def plot_chain(ax, chain, color='blue', alpha=1.0, linewidth=2, label='Chain'):
    """Plot a robot chain in 3D"""
    
    # Extract joint positions
    positions = []
    for joint in chain.joints:
        positions.append([joint.position.x, joint.position.y, joint.position.z])
    
    positions = np.array(positions)
    
    # Plot joints
    for i, pos in enumerate(positions):
        joint_type = str(chain.joints[i].type)
        if 'FIXED_BASE' in joint_type:
            # Only label the first base we plot
            base_label = f'{label} Base' if i == 0 and label == 'Initial' else ""
            ax.scatter(*pos, color='black', s=150, marker='s', alpha=alpha, label=base_label)
        elif 'SPHERICAL' in joint_type:
            ax.scatter(*pos, color=color, s=80, marker='o', alpha=alpha)
        else:  # END_EFFECTOR
            # Only label the first end-effector we plot
            ee_label = f'{label} End-Effector' if i == len(positions)-1 and label == 'Initial' else ""
            ax.scatter(*pos, color='red', s=120, marker='*', alpha=alpha, label=ee_label)
    
    # Plot segments
    for i in range(len(positions) - 1):
        ax.plot([positions[i][0], positions[i+1][0]], 
                [positions[i][1], positions[i+1][1]], 
                [positions[i][2], positions[i+1][2]], 
                color=color, linewidth=linewidth, alpha=alpha,
                label=f'{label}' if i == 0 else "")
    
    return positions

def visualize_fabrik_enhanced(target_x, target_y, target_z):
    """Create enhanced visualization with constants and improved analysis"""
    
    try:
        import delta_robot.fabrik_initialization as fi
        import delta_robot.fabrik_backward as fb
        import delta_robot.fabrik_forward as ff
        import delta_robot.delta_types as dt
        import delta_robot  # Import main module for constants
    except ImportError as e:
        print(f"Error: delta_robot package not found: {e}")
        return False
    
    # ✅ USE CONSTANTS FROM C++ HEADERS
    num_segments = delta_robot.DEFAULT_ROBOT_SEGMENTS
    tolerance = delta_robot.FABRIK_TOLERANCE
    max_iterations = delta_robot.FABRIK_MAX_ITERATIONS
    min_height = delta_robot.MIN_HEIGHT
    motor_limit = delta_robot.MOTOR_LIMIT
    working_height = delta_robot.WORKING_HEIGHT
    spherical_angle_deg = delta_robot.SPHERICAL_JOINT_CONE_ANGLE_DEG
    
    print(f"Using C++ constants:")
    print(f"  DEFAULT_ROBOT_SEGMENTS = {num_segments}")
    print(f"  FABRIK_TOLERANCE = {tolerance}")
    print(f"  SPHERICAL_JOINT_CONE_ANGLE = {spherical_angle_deg}°")
    
    # Initialize robot chain
    init_result = fi.FabrikInitialization.initialize_straight_up(num_segments)
    initial_chain = init_result.chain
    original_lengths = [seg.length for seg in initial_chain.segments]
    
    # Create target position
    target_position = dt.Vector3(target_x, target_y, target_z)
    
    # Calculate target vector properties
    target_roll, target_pitch, target_magnitude = vector_to_spherical(target_position)
    
    # Perform backward iteration with constants
    backward_result = fb.FabrikBackward.iterate_to_target(
        initial_chain, 
        target_position,
        tolerance=tolerance,
        max_iterations=max_iterations
    )
    backward_chain = backward_result.updated_chain
    
    # Calculate new segment lengths
    new_lengths = ff.FabrikForward.calculate_new_segment_lengths(backward_chain)
    
    # Perform forward iteration with constants
    forward_result = ff.FabrikForward.iterate_from_base(
        backward_chain,
        tolerance=tolerance,
        max_iterations=max_iterations
    )
    forward_chain = forward_result.updated_chain
    
    # Setup figure with large 3D view and text area
    fig = plt.figure(figsize=(20, 12))
    
    # Large 3D plot (left 65% of screen)
    ax = fig.add_subplot(121, projection='3d')
    
    # Plot all three phases
    initial_positions = plot_chain(ax, initial_chain, color='lightblue', alpha=0.4, 
                                 linewidth=2, label='Initial')
    backward_positions = plot_chain(ax, backward_chain, color='orange', alpha=0.6, 
                                  linewidth=3, label='Backward')
    forward_positions = plot_chain(ax, forward_chain, color='red', alpha=1.0, 
                                 linewidth=4, label='Forward')
    
    # Plot target
    ax.scatter([target_x], [target_y], [target_z], color='gold', s=300, 
               marker='X', label='Target', edgecolor='black', linewidth=2)
    
    # Set axis properties
    max_range = max(abs(target_x), abs(target_y), abs(target_z), init_result.total_reach) * 1.1
    ax.set_xlim([-max_range/4, max_range])
    ax.set_ylim([-max_range/4, max_range])
    ax.set_zlim([0, max_range])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('FABRIK Complete Cycle with Constants', fontsize=14, weight='bold')
    ax.legend(loc='upper left')
    
    # Calculate debug information
    final_ee = forward_chain.joints[-1].position
    final_dist_to_target = math.sqrt(
        (final_ee.x - target_x)**2 + 
        (final_ee.y - target_y)**2 + 
        (final_ee.z - target_z)**2
    )
    
    base_error = math.sqrt(
        forward_chain.joints[0].position.x**2 +
        forward_chain.joints[0].position.y**2 +
        forward_chain.joints[0].position.z**2
    )
    
    # Create enhanced debug text (right 35% of screen)
    ax_text = fig.add_subplot(122)
    ax_text.axis('off')
    
    debug_text = f"""FABRIK COMPLETE ANALYSIS (UPDATED WITH CONSTANTS)

C++ CONSTANTS USED:
  DEFAULT_ROBOT_SEGMENTS = {delta_robot.DEFAULT_ROBOT_SEGMENTS}
  FABRIK_TOLERANCE = {delta_robot.FABRIK_TOLERANCE}
  FABRIK_MAX_ITERATIONS = {delta_robot.FABRIK_MAX_ITERATIONS}
  MIN_HEIGHT = {delta_robot.MIN_HEIGHT}
  MOTOR_LIMIT = {delta_robot.MOTOR_LIMIT}
  WORKING_HEIGHT = {delta_robot.WORKING_HEIGHT}
  SPHERICAL_JOINT_CONE_ANGLE = {delta_robot.SPHERICAL_JOINT_CONE_ANGLE_DEG}°

TARGET ANALYSIS:
Target Position: ({target_x}, {target_y}, {target_z})
Target Magnitude: {target_magnitude:.1f}
Target Roll (azimuth): {target_roll:.1f}°
Target Pitch (elevation): {target_pitch:.1f}°
Distance from origin: {target_magnitude:.1f}
Max robot reach: {init_result.total_reach:.1f}

BACKWARD ITERATION:
Iterations: {backward_result.iterations_used}/{max_iterations}
Target reached: {'YES' if final_dist_to_target < 10 else 'NO'}
Base moved to: ({backward_chain.joints[0].position.x:.1f}, {backward_chain.joints[0].position.y:.1f}, {backward_chain.joints[0].position.z:.1f})

SEGMENT LENGTH RECALCULATION:
Using H→G formula: MIN_HEIGHT + 2*MOTOR_LIMIT + prismatic
                   {min_height} + 2*{motor_limit} + prismatic

Original → Recalculated:"""

    for i, (orig, new) in enumerate(zip(original_lengths, new_lengths)):
        change = new - orig
        debug_text += f"\n  Seg {i}: {orig:.1f} → {new:.1f} ({change:+.1f})"

    debug_text += f"""

FORWARD ITERATION:
Iterations: {forward_result.iterations_used}/{max_iterations}
Base fixed: {'YES' if base_error < tolerance else 'NO'} (error: {base_error:.3f})
Lengths preserved: {'YES' if forward_result.constraints_satisfied else 'NO'}

FINAL RESULT:
End-effector: ({final_ee.x:.1f}, {final_ee.y:.1f}, {final_ee.z:.1f})
Distance to target: {final_dist_to_target:.2f}
Within tolerance: {'YES' if final_dist_to_target <= tolerance else 'NO'} (tolerance: {tolerance})
Accuracy: {((1 - final_dist_to_target/target_magnitude) * 100):.1f}%

ANGULAR ACCURACY:
Final EE Roll: {vector_to_spherical(final_ee)[0]:.1f}°
Final EE Pitch: {vector_to_spherical(final_ee)[1]:.1f}°
Target Roll: {target_roll:.1f}°
Target Pitch: {target_pitch:.1f}°
Roll Error: {abs(vector_to_spherical(final_ee)[0] - target_roll):.1f}°
Pitch Error: {abs(vector_to_spherical(final_ee)[1] - target_pitch):.1f}°

FINAL CHAIN SEGMENTS:"""
    
    # Calculate final segment lengths for verification
    for i, segment in enumerate(forward_chain.segments):
        start_pos = forward_chain.joints[segment.start_joint_index].position
        end_pos = forward_chain.joints[segment.end_joint_index].position
        
        actual_length = math.sqrt(
            (end_pos.x - start_pos.x)**2 +
            (end_pos.y - start_pos.y)**2 +
            (end_pos.z - start_pos.z)**2
        )
        expected = new_lengths[i]
        error = abs(actual_length - expected)
        debug_text += f"\n  [{i}] Expected: {expected:.1f}, Actual: {actual_length:.1f}, Error: {error:.2f}"
    
    debug_text += f"""

ALGORITHM IMPROVEMENTS:
✓ Fixed backward iteration (direction projection)
✓ Fixed forward iteration (same improvements)
✓ Uses actual C++ constants (no hardcoding)
✓ Proper tolerance checking
✓ Consistent segment counting
✓ Better cone constraint handling

PERFORMANCE METRICS:
Total iterations: {backward_result.iterations_used + forward_result.iterations_used}
Final position error: {final_dist_to_target:.4f}
Base position error: {base_error:.4f}
All constraints satisfied: {'YES' if forward_result.constraints_satisfied and final_dist_to_target <= tolerance else 'NO'}"""
    
    # Add debug text with scrollable box
    ax_text.text(0.05, 0.95, debug_text, transform=ax_text.transAxes, fontsize=8,
                verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round,pad=0.5', facecolor='lightgreen', alpha=0.8))
    
    plt.tight_layout()
    plt.show()
    
    # Enhanced console output
    print(f"\n{'='*60}")
    print(f"FABRIK COMPLETE ANALYSIS WITH CONSTANTS")
    print(f"{'='*60}")
    print(f"Target: ({target_x}, {target_y}, {target_z})")
    print(f"Target Roll: {target_roll:.1f}°, Pitch: {target_pitch:.1f}°")
    print(f"\nUsing C++ constants:")
    print(f"  Robot segments: {delta_robot.DEFAULT_ROBOT_SEGMENTS}")
    print(f"  Tolerance: {delta_robot.FABRIK_TOLERANCE}")
    print(f"  Max iterations: {delta_robot.FABRIK_MAX_ITERATIONS}")
    print(f"  Cone angle: {delta_robot.SPHERICAL_JOINT_CONE_ANGLE_DEG}°")
    print(f"\nResults:")
    print(f"  Final distance to target: {final_dist_to_target:.4f}")
    print(f"  Final roll: {vector_to_spherical(final_ee)[0]:.1f}°, pitch: {vector_to_spherical(final_ee)[1]:.1f}°")
    print(f"  Angular errors - Roll: {abs(vector_to_spherical(final_ee)[0] - target_roll):.1f}°, Pitch: {abs(vector_to_spherical(final_ee)[1] - target_pitch):.1f}°")
    print(f"  Base error: {base_error:.4f}")
    print(f"  Total iterations: {backward_result.iterations_used + forward_result.iterations_used}")
    print(f"  Within tolerance: {'YES' if final_dist_to_target <= tolerance else 'NO'}")
    
    return True

def main():
    if len(sys.argv) == 2:
        target_x, target_y, target_z = parse_coordinates(sys.argv[1])
    else:
        target_x, target_y, target_z = 100, 100, 300
        print("Usage: python3 test_fabrik_forward_visual.py x,y,z")
    
    print(f"Enhanced FABRIK visualization with constants for target: ({target_x}, {target_y}, {target_z})")
    visualize_fabrik_enhanced(target_x, target_y, target_z)

if __name__ == "__main__":
    main()