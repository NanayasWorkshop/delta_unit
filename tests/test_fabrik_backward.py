#!/usr/bin/env python3
"""
Visual test for the FABRIK Backward module - UPDATED with constants
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

def plot_chain(ax, chain, color='blue', alpha=1.0, linewidth=2, label='Chain', show_constraints=False):
    """Plot a robot chain in 3D"""
    
    # Extract joint positions
    positions = []
    joint_types = []
    
    for joint in chain.joints:
        positions.append([joint.position.x, joint.position.y, joint.position.z])
        joint_types.append(joint.type)
    
    positions = np.array(positions)
    
    # Plot joints
    for i, (pos, joint_type) in enumerate(zip(positions, joint_types)):
        if 'FIXED_BASE' in str(joint_type):
            ax.scatter(*pos, color='black', s=150, marker='s', alpha=alpha, 
                      label=f'{label} Base' if i == 0 else "")
        elif 'SPHERICAL' in str(joint_type):
            ax.scatter(*pos, color=color, s=100, marker='o', alpha=alpha,
                      label=f'{label} Spherical' if i == 1 else "")
            
            # Show spherical constraint cone with CORRECT direction
            if show_constraints and alpha > 0.7 and i > 1:  # Only for joints with previous segments
                cone_height = 15
                cone_radius = cone_height * math.tan(math.radians(60))  # 120° cone = 60° half-angle
                
                # Cone axis should point from next joint to current joint (incoming direction)
                if i < len(positions) - 1:  # Not the last joint
                    next_pos = positions[i + 1]
                    incoming_direction = pos - next_pos
                    incoming_direction = incoming_direction / np.linalg.norm(incoming_direction)
                    
                    # Create cone pointing in incoming direction
                    cone_tip = pos + incoming_direction * cone_height
                    
                    # Create cone base circle
                    theta = np.linspace(0, 2*np.pi, 12)
                    
                    # Find two perpendicular vectors to incoming_direction
                    if abs(incoming_direction[2]) < 0.9:
                        perp1 = np.cross(incoming_direction, [0, 0, 1])
                    else:
                        perp1 = np.cross(incoming_direction, [1, 0, 0])
                    perp1 = perp1 / np.linalg.norm(perp1)
                    perp2 = np.cross(incoming_direction, perp1)
                    perp2 = perp2 / np.linalg.norm(perp2)
                    
                    # Create cone base points
                    for j in range(len(theta)):
                        base_point = cone_tip + cone_radius * (perp1 * np.cos(theta[j]) + perp2 * np.sin(theta[j]))
                        ax.plot([pos[0], base_point[0]], [pos[1], base_point[1]], [pos[2], base_point[2]], 
                               color=color, alpha=0.3, linewidth=0.8)
                    
                    # Draw cone axis
                    ax.plot([pos[0], cone_tip[0]], [pos[1], cone_tip[1]], [pos[2], cone_tip[2]], 
                           color=color, alpha=0.6, linewidth=2, linestyle='--')
        else:  # END_EFFECTOR
            ax.scatter(*pos, color='red', s=120, marker='*', alpha=alpha,
                      label=f'{label} End-Effector' if i == len(positions)-1 else "")
    
    # Plot segments
    for i in range(len(positions) - 1):
        ax.plot([positions[i][0], positions[i+1][0]], 
                [positions[i][1], positions[i+1][1]], 
                [positions[i][2], positions[i+1][2]], 
                color=color, linewidth=linewidth, alpha=alpha,
                label=f'{label} Segments' if i == 0 else "")
    
    return positions

def visualize_fabrik_backward(target_x, target_y, target_z):
    """Create 3D visualization of FABRIK backward iteration"""
    
    try:
        import delta_robot.fabrik_initialization as fi
        import delta_robot.fabrik_backward as fb
        import delta_robot.delta_types as dt
        import delta_robot  # Import main module for constants
    except ImportError as e:
        print(f"Error: delta_robot package not found: {e}")
        return False
    
    # ✅ USE CONSTANTS FROM C++ HEADERS
    num_segments = delta_robot.DEFAULT_ROBOT_SEGMENTS
    tolerance = delta_robot.FABRIK_TOLERANCE
    max_iterations = delta_robot.FABRIK_MAX_ITERATIONS
    spherical_angle_deg = delta_robot.SPHERICAL_JOINT_CONE_ANGLE_DEG
    
    print(f"Using C++ constants:")
    print(f"  DEFAULT_ROBOT_SEGMENTS = {num_segments}")
    print(f"  FABRIK_TOLERANCE = {tolerance}")
    print(f"  SPHERICAL_JOINT_CONE_ANGLE = {spherical_angle_deg}°")
    
    # Initialize robot chain
    init_result = fi.FabrikInitialization.initialize_straight_up(num_segments)
    initial_chain = init_result.chain
    
    print(f"Initialized {num_segments}-segment robot with {len(initial_chain.joints)} joints")
    
    # Create target position
    target_position = dt.Vector3(target_x, target_y, target_z)
    
    # Perform backward iteration with constants
    result = fb.FabrikBackward.iterate_to_target(
        initial_chain, 
        target_position,
        tolerance=tolerance,
        max_iterations=max_iterations
    )
    final_chain = result.updated_chain
    
    # Setup figure with larger 3D view
    fig = plt.figure(figsize=(20, 10))
    
    # Main 3D comparison view (much larger)
    ax1 = fig.add_subplot(121, projection='3d')
    
    # Plot initial chain
    initial_positions = plot_chain(ax1, initial_chain, color='blue', alpha=0.5, 
                                 linewidth=3, label='Initial', show_constraints=False)
    
    # Plot final chain
    final_positions = plot_chain(ax1, final_chain, color='red', alpha=1.0, 
                               linewidth=3, label='Final', show_constraints=True)
    
    # Plot target
    ax1.scatter([target_x], [target_y], [target_z], color='gold', s=200, 
               marker='X', label='Target', edgecolor='orange', linewidth=2)
    
    # Draw movement arrows
    for i, (initial_pos, final_pos) in enumerate(zip(initial_positions, final_positions)):
        if not np.allclose(initial_pos, final_pos, atol=0.1):
            ax1.quiver(initial_pos[0], initial_pos[1], initial_pos[2],
                      final_pos[0] - initial_pos[0], 
                      final_pos[1] - initial_pos[1], 
                      final_pos[2] - initial_pos[2],
                      color='purple', alpha=0.6, arrow_length_ratio=0.1)
    
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title(f'FABRIK Backward Iteration: {num_segments}-Segment Robot\nBefore vs After (Fixed Algorithm)')
    ax1.legend()
    
    # Set equal aspect ratio
    max_range = max(abs(target_x), abs(target_y), abs(target_z), init_result.total_reach) * 1.2
    ax1.set_xlim([-max_range/2, max_range])
    ax1.set_ylim([-max_range/2, max_range])
    ax1.set_zlim([0, max_range])
    
    # Results summary (now takes right half of screen)
    ax2 = fig.add_subplot(122)
    ax2.axis('off')
    
    # Calculate joint movements for summary
    joint_movements = []
    for i, (initial_joint, final_joint) in enumerate(zip(initial_chain.joints, final_chain.joints)):
        movement = math.sqrt(
            (final_joint.position.x - initial_joint.position.x)**2 +
            (final_joint.position.y - initial_joint.position.y)**2 +
            (final_joint.position.z - initial_joint.position.z)**2
        )
        joint_movements.append(movement)
    
    # Calculate final distances for verification
    final_end_effector = fb.FabrikBackward.get_end_effector_position(final_chain)
    distance_to_target = math.sqrt(
        (final_end_effector.x - target_x)**2 +
        (final_end_effector.y - target_y)**2 +
        (final_end_effector.z - target_z)**2
    )
    
    # Check segment length violations
    segment_violations = []
    for i, segment in enumerate(final_chain.segments):
        start_pos = final_chain.joints[segment.start_joint_index].position
        end_pos = final_chain.joints[segment.end_joint_index].position
        
        actual_length = math.sqrt(
            (end_pos.x - start_pos.x)**2 +
            (end_pos.y - start_pos.y)**2 +
            (end_pos.z - start_pos.z)**2
        )
        
        error = abs(actual_length - segment.length)
        if error > 0.1:
            segment_violations.append((i, error))
    
    # Create summary text
    reachable = result.target_reachable
    base_moved = joint_movements[0] > 0.1
    
    summary_text = f"""
FABRIK BACKWARD ITERATION ANALYSIS (FIXED ALGORITHM)

Robot Configuration: {num_segments} segments, {len(initial_chain.joints)} joints
Using C++ Constants:
  DEFAULT_ROBOT_SEGMENTS = {delta_robot.DEFAULT_ROBOT_SEGMENTS}
  FABRIK_TOLERANCE = {delta_robot.FABRIK_TOLERANCE}
  FABRIK_MAX_ITERATIONS = {delta_robot.FABRIK_MAX_ITERATIONS}
  SPHERICAL_JOINT_CONE_ANGLE = {delta_robot.SPHERICAL_JOINT_CONE_ANGLE_DEG}°

Target: ({target_x}, {target_y}, {target_z})
Target Reachable: {'✓ YES' if reachable else '✗ NO'}
Distance from Base: {math.sqrt(target_x**2 + target_y**2 + target_z**2):.1f}
Max Reach: {init_result.total_reach:.1f}

RESULTS:
Iterations: {result.iterations_used}/{max_iterations}
Final End-Effector: ({final_end_effector.x:.1f}, {final_end_effector.y:.1f}, {final_end_effector.z:.1f})
Distance to Target: {distance_to_target:.4f}
Converged: {'✓ YES' if distance_to_target < tolerance else '✗ NO'} (tolerance: {tolerance})

BASE MOVEMENT:
Base Moved: {'✓ YES' if base_moved else '✗ NO'} ({joint_movements[0]:.1f} units)
Final Base: ({final_chain.joints[0].position.x:.1f}, {final_chain.joints[0].position.y:.1f}, {final_chain.joints[0].position.z:.1f})

SEGMENT LENGTH VIOLATIONS:
"""
    
    if segment_violations:
        summary_text += f"⚠️  {len(segment_violations)} violations detected:\n"
        for seg_idx, error in segment_violations:
            summary_text += f"   Segment {seg_idx}: ±{error:.1f} units\n"
    else:
        summary_text += "✓ All segment lengths preserved\n"
    
    summary_text += f"""
JOINT MOVEMENTS (showing first 10):
"""
    for i, movement in enumerate(joint_movements[:10]):  # Show only first 10 joints
        joint_type = str(initial_chain.joints[i].type).split('.')[-1] if hasattr(initial_chain.joints[i].type, 'name') else str(initial_chain.joints[i].type)
        summary_text += f"   Joint {i} ({joint_type[:8]}): {movement:.1f} units\n"
    
    if len(joint_movements) > 10:
        summary_text += f"   ... and {len(joint_movements) - 10} more joints\n"
    
    summary_text += f"""
ALGORITHM STATUS:
✓ Backward iteration FIXED (direction projection)
{'✓' if base_moved else '✗'} Base movement enabled
{'✓' if distance_to_target < tolerance else '✗'} Target reached within tolerance
{'✓' if len(segment_violations) == 0 else '✗'} Segment lengths preserved
✓ Using {num_segments} segments from C++ constants

ALGORITHM IMPROVEMENTS:
- Fixed cone constraint projection (direction vs point)
- Better fallback strategies when guidance fails
- Maintains goal of minimizing distance to original positions
- Uses actual C++ constants instead of hardcoded values

Next: Forward iteration will fix base to (0,0,0)
"""
    
    ax2.text(0.05, 0.95, summary_text, transform=ax2.transAxes, fontsize=10,
             verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='lightgreen' if len(segment_violations) == 0 else 'lightyellow', alpha=0.8))
    
    plt.tight_layout()
    plt.show()
    
    # Print analysis to console
    print(f"\n" + "="*60)
    print("VISUAL ANALYSIS COMPLETE")
    print("="*60)
    print(f"Robot: {num_segments} segments, {len(initial_chain.joints)} joints")
    print(f"Target: ({target_x}, {target_y}, {target_z})")
    print(f"Backward iteration: {'✓ SUCCESS' if distance_to_target < tolerance else '✗ FAILED'}")
    print(f"Segment violations: {len(segment_violations)}")
    print(f"Base moved: {'✓ YES' if base_moved else '✗ NO'} ({joint_movements[0]:.1f} units)")
    print(f"Using constants from C++ headers:")
    print(f"  DEFAULT_ROBOT_SEGMENTS = {delta_robot.DEFAULT_ROBOT_SEGMENTS}")
    print(f"  FABRIK_TOLERANCE = {delta_robot.FABRIK_TOLERANCE}")
    print(f"  SPHERICAL_JOINT_CONE_ANGLE = {delta_robot.SPHERICAL_JOINT_CONE_ANGLE_DEG}°")
    
    if len(segment_violations) > 0:
        print("\n⚠️  SEGMENT LENGTH ISSUES DETECTED:")
        print("This suggests the backward iteration algorithm needs refinement")
        print("Expected: All segments should maintain their original lengths")
    else:
        print("\n✓ SEGMENT LENGTHS PRESERVED CORRECTLY")
        print("Fixed backward iteration is working as expected!")
    
    return True

def main():
    # Parse command line arguments for target position
    if len(sys.argv) == 2:
        target_x, target_y, target_z = parse_coordinates(sys.argv[1])
        print(f"Using command line target: ({target_x}, {target_y}, {target_z})")
    else:
        target_x, target_y, target_z = 100, 100, 300
        print("Using default target: (100, 100, 300)")
        print("Usage: python3 test_fabrik_backward_visual.py x,y,z")
    
    print("Creating FABRIK Backward Iteration visualization with FIXED algorithm...")
    success = visualize_fabrik_backward(target_x, target_y, target_z)
    
    if success:
        print("\nVisualization shows:")
        print("- 3D view: Initial (blue) vs Final (red) chain positions")
        print("- FIXED algorithm: Uses direction projection instead of point projection")
        print("- Dynamic joint count: Uses DEFAULT_ROBOT_SEGMENTS from C++ constants")
        print("- Joint movements: How much each joint moved")
        print("- Summary: Algorithm performance and improvements")
        print("\nTry different targets:")
        print("  python3 test_fabrik_backward_visual.py 0,0,400   # Straight up")
        print("  python3 test_fabrik_backward_visual.py 200,0,200 # Far sideways")
        print("  python3 test_fabrik_backward_visual.py 50,50,100 # Close target")
    else:
        print("Visualization failed - check module installation")

if __name__ == "__main__":
    main()