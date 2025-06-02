#!/usr/bin/env python3
"""
Enhanced visual test for FABRIK Forward - Shows roll/pitch angles and target vectors
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

def calculate_chain_debug_info(chain, name, original_lengths=None, new_lengths=None):
    """Calculate debug information for a chain"""
    
    # Joint positions
    joints_info = []
    for i, joint in enumerate(chain.joints):
        joint_type = str(joint.type).split('.')[-1] if '.' in str(joint.type) else str(joint.type)
        joints_info.append(f"  [{i}] {joint_type[:8]}: ({joint.position.x:.1f}, {joint.position.y:.1f}, {joint.position.z:.1f})")
    
    # Segment lengths
    segments_info = []
    actual_lengths = []
    for i, segment in enumerate(chain.segments):
        start_pos = chain.joints[segment.start_joint_index].position
        end_pos = chain.joints[segment.end_joint_index].position
        
        actual_length = math.sqrt(
            (end_pos.x - start_pos.x)**2 +
            (end_pos.y - start_pos.y)**2 +
            (end_pos.z - start_pos.z)**2
        )
        actual_lengths.append(actual_length)
        
        expected = new_lengths[i] if new_lengths else segment.length
        error = abs(actual_length - expected)
        segments_info.append(f"  [{i}] Expected: {expected:.1f}, Actual: {actual_length:.1f}, Error: {error:.2f}")
    
    return joints_info, segments_info, actual_lengths

def extract_enhanced_fabrik_calculations(backward_chain, target_position):
    """Extract enhanced calculations with roll/pitch and target vectors"""
    
    try:
        import delta_robot.fabrik_forward as ff
        import delta_robot.fermat_module as fermat
        import delta_robot.delta_types as dt
    except ImportError:
        return [], [], [], [], [], [], []
    
    # Step 1: Extract direction pairs (same as in fabrik_forward.cpp)
    direction_pairs = []
    target_vectors = []  # Store the actual target vectors used
    num_segments = backward_chain.num_robot_segments
    
    for seg in range(num_segments):
        ref_start = seg
        ref_end = seg + 1
        target_start = seg + 1
        target_end = seg + 2
        
        # Ensure indices are valid (same logic as C++)
        if target_end >= len(backward_chain.joints):
            target_start = ref_start
            target_end = ref_end
        
        ref_pos_start = backward_chain.joints[ref_start].position
        ref_pos_end = backward_chain.joints[ref_end].position
        target_pos_start = backward_chain.joints[target_start].position
        target_pos_end = backward_chain.joints[target_end].position
        
        # Calculate reference direction
        ref_dir = dt.Vector3(
            ref_pos_end.x - ref_pos_start.x,
            ref_pos_end.y - ref_pos_start.y,
            ref_pos_end.z - ref_pos_start.z
        ).normalized()
        
        # Calculate target direction
        target_dir = dt.Vector3(
            target_pos_end.x - target_pos_start.x,
            target_pos_end.y - target_pos_start.y,
            target_pos_end.z - target_pos_start.z
        ).normalized()
        
        direction_pairs.append((ref_dir, target_dir, seg))
        target_vectors.append(target_dir)
    
    # Step 2: Calculate segment properties with enhanced info
    transformed_directions = []
    prismatic_lengths = []
    h_to_g_distances = []
    roll_angles = []
    pitch_angles = []
    fermat_points = []
    
    for i, (ref_dir, target_dir, seg_idx) in enumerate(direction_pairs):
        # Transform target direction to Z+ reference coordinate system
        transformed_dir = transform_to_z_reference(ref_dir, target_dir)
        transformed_directions.append(transformed_dir)
        
        # Calculate roll and pitch of the transformed direction
        roll, pitch, magnitude = vector_to_spherical(transformed_dir)
        roll_angles.append(roll)
        pitch_angles.append(pitch)
        
        # Calculate prismatic length using KinematicsModule (which handles half-angle correctly!)
        kinematics_result = None
        try:
            import delta_robot.kinematics_module as kinematics
            kinematics_result = kinematics.KinematicsModule.calculate(transformed_dir)
            prismatic_length = kinematics_result.prismatic_joint_length
        except ImportError:
            # Fallback to manual calculation if kinematics module not available
            fermat_result = fermat.FermatModule.calculate(transformed_dir)
            prismatic_length = 2.0 * fermat_result.fermat_point.z
        
        prismatic_lengths.append(prismatic_length)
        
        # Store the exact vector that was passed to the solver
        print(f"DEBUG: Segment {i} - Solver input: ({transformed_dir.x:.6f}, {transformed_dir.y:.6f}, {transformed_dir.z:.6f}) → Prismatic: {prismatic_length:.6f}")
        if kinematics_result:
            print(f"       Half-angle vector: ({kinematics_result.transformed_vector.x:.6f}, {kinematics_result.transformed_vector.y:.6f}, {kinematics_result.transformed_vector.z:.6f})")
        
        # Calculate H→G distance
        h_to_g_distance = 101.0 + 2.0 * 11.0 + prismatic_length  # MIN_HEIGHT + 2*MOTOR_LIMIT + prismatic
        h_to_g_distances.append(h_to_g_distance)
    
    return (direction_pairs, target_vectors, transformed_directions, 
            prismatic_lengths, h_to_g_distances, roll_angles, pitch_angles)

def transform_to_z_reference(reference_direction, target_direction):
    """Transform direction to Z+ reference (simplified version of C++ implementation)"""
    import delta_robot.delta_types as dt
    import numpy as np
    
    ref_norm = np.array([reference_direction.x, reference_direction.y, reference_direction.z])
    target_norm = np.array([target_direction.x, target_direction.y, target_direction.z])
    z_axis = np.array([0, 0, 1])
    
    ref_norm = ref_norm / np.linalg.norm(ref_norm)
    target_norm = target_norm / np.linalg.norm(target_norm)
    
    # If reference is already Z+, no transformation needed
    if np.linalg.norm(ref_norm - z_axis) < 1e-6:
        return dt.Vector3(target_norm[0], target_norm[1], target_norm[2])
    
    # If reference is -Z, simple flip
    if np.linalg.norm(ref_norm + z_axis) < 1e-6:
        return dt.Vector3(-target_norm[0], -target_norm[1], -target_norm[2])
    
    # Calculate rotation axis (cross product)
    rotation_axis = np.cross(ref_norm, z_axis)
    rotation_axis_length = np.linalg.norm(rotation_axis)
    
    if rotation_axis_length < 1e-6:
        return dt.Vector3(target_norm[0], target_norm[1], target_norm[2])
    
    rotation_axis = rotation_axis / rotation_axis_length
    
    # Calculate rotation angle
    cos_angle = np.clip(np.dot(ref_norm, z_axis), -1.0, 1.0)
    angle = np.arccos(cos_angle)
    
    # Apply Rodrigues rotation to target direction
    cos_angle_rot = np.cos(angle)
    sin_angle_rot = np.sin(angle)
    
    v_parallel = rotation_axis * np.dot(rotation_axis, target_norm)
    v_perpendicular = target_norm - v_parallel
    w = np.cross(rotation_axis, v_perpendicular)
    
    rotated = v_parallel + v_perpendicular * cos_angle_rot + w * sin_angle_rot
    
    return dt.Vector3(rotated[0], rotated[1], rotated[2])

def visualize_fabrik_enhanced(target_x, target_y, target_z):
    """Create enhanced visualization with roll/pitch/target vector info"""
    
    try:
        import delta_robot.fabrik_initialization as fi
        import delta_robot.fabrik_backward as fb
        import delta_robot.fabrik_forward as ff
        import delta_robot.delta_types as dt
    except ImportError as e:
        print(f"Error: delta_robot package not found: {e}")
        return False
    
    # Initialize robot chain
    init_result = fi.FabrikInitialization.initialize_straight_up(3)
    initial_chain = init_result.chain
    original_lengths = [seg.length for seg in initial_chain.segments]
    
    # Create target position
    target_position = dt.Vector3(target_x, target_y, target_z)
    
    # Calculate target vector properties
    target_roll, target_pitch, target_magnitude = vector_to_spherical(target_position)
    
    # Perform backward iteration
    backward_result = fb.FabrikBackward.iterate_to_target(initial_chain, target_position)
    backward_chain = backward_result.updated_chain
    
    # Calculate new segment lengths
    new_lengths = ff.FabrikForward.calculate_new_segment_lengths(backward_chain)
    
    # Perform forward iteration
    forward_result = ff.FabrikForward.iterate_from_base(backward_chain)
    forward_chain = forward_result.updated_chain
    
    # Extract enhanced FABRIK calculations
    (direction_pairs, target_vectors, transformed_directions, 
     prismatic_lengths, h_to_g_distances, roll_angles, pitch_angles) = extract_enhanced_fabrik_calculations(backward_chain, target_position)
    
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
    ax.set_title('FABRIK Complete Cycle with Target Analysis', fontsize=14, weight='bold')
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
    
    debug_text = f"""TARGET VECTOR ANALYSIS:
Target Position: ({target_x}, {target_y}, {target_z})
Target Magnitude: {target_magnitude:.1f}
Target Roll (azimuth): {target_roll:.1f}°
Target Pitch (elevation): {target_pitch:.1f}°
Distance from origin: {target_magnitude:.1f}
Max robot reach: {init_result.total_reach:.1f}

BACKWARD ITERATION:
Iterations: {backward_result.iterations_used}
Target reached: {'YES' if final_dist_to_target < 10 else 'NO'}
Base moved to: ({backward_chain.joints[0].position.x:.1f}, {backward_chain.joints[0].position.y:.1f}, {backward_chain.joints[0].position.z:.1f})

SEGMENT LENGTH RECALCULATION:
Original → Recalculated:"""

    for i, (orig, new) in enumerate(zip(original_lengths, new_lengths)):
        change = new - orig
        debug_text += f"\n  Seg {i}: {orig:.1f} → {new:.1f} ({change:+.1f})"

    debug_text += f"""

ENHANCED DIRECTION & ANGLE ANALYSIS:"""
    for i, (ref_dir, target_dir, seg_idx) in enumerate(direction_pairs):
        debug_text += f"\n  Segment {i}:"
        debug_text += f"\n    Ref Dir:    ({ref_dir.x:.3f}, {ref_dir.y:.3f}, {ref_dir.z:.3f})"
        
        # Target direction angles
        target_roll, target_pitch, target_mag = vector_to_spherical(target_dir)
        debug_text += f"\n    Target Dir: ({target_dir.x:.3f}, {target_dir.y:.3f}, {target_dir.z:.3f})"
        debug_text += f"\n    Target Roll: {target_roll:.1f}°, Pitch: {target_pitch:.1f}°"
        
        if i < len(transformed_directions):
            trans_dir = transformed_directions[i]
            debug_text += f"\n    Trans Dir:  ({trans_dir.x:.3f}, {trans_dir.y:.3f}, {trans_dir.z:.3f})"
            debug_text += f"\n    Trans Roll: {roll_angles[i]:.1f}°, Pitch: {pitch_angles[i]:.1f}°"

    debug_text += f"""

KINEMATIC SOLVER INPUT & PRISMATIC RESULTS:"""
    for i, (pris, roll, pitch) in enumerate(zip(prismatic_lengths, roll_angles, pitch_angles)):
        trans_dir = transformed_directions[i]
        debug_text += f"\n  Segment {i}:"
        debug_text += f"\n    Solver Input Vector: ({trans_dir.x:.3f}, {trans_dir.y:.3f}, {trans_dir.z:.3f})"
        debug_text += f"\n    Vector Roll: {roll:.1f}°, Pitch: {pitch:.1f}°"
        debug_text += f"\n    → Prismatic Length: {pris:.3f}"
        debug_text += f"\n    → H→G Distance: {h_to_g_distances[i]:.3f}"

    debug_text += f"""

FORWARD ITERATION:
Iterations: {forward_result.iterations_used}
Base fixed: {'YES' if base_error < 0.1 else 'NO'} (error: {base_error:.3f})
Lengths preserved: {'YES' if forward_result.constraints_satisfied else 'NO'}

FINAL RESULT:
End-effector: ({final_ee.x:.1f}, {final_ee.y:.1f}, {final_ee.z:.1f})
Distance to target: {final_dist_to_target:.2f}
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
    
    # Add debug text with scrollable box
    ax_text.text(0.05, 0.95, debug_text, transform=ax_text.transAxes, fontsize=8,
                verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round,pad=0.5', facecolor='lightgreen', alpha=0.8))
    
    plt.tight_layout()
    plt.show()
    
    # Enhanced console output
    print(f"\n{'='*60}")
    print(f"KINEMATIC SOLVER ANALYSIS")
    print(f"{'='*60}")
    print(f"Target: ({target_x}, {target_y}, {target_z})")
    print(f"Target Roll: {target_roll:.1f}°, Pitch: {target_pitch:.1f}°")
    print(f"\nKINEMATIC SOLVER INPUTS & OUTPUTS:")
    for i, (trans_dir, pris, roll, pitch) in enumerate(zip(transformed_directions, prismatic_lengths, roll_angles, pitch_angles)):
        print(f"  Segment {i}:")
        print(f"    Input Vector: ({trans_dir.x:.6f}, {trans_dir.y:.6f}, {trans_dir.z:.6f})")
        print(f"    Roll: {roll:.1f}°, Pitch: {pitch:.1f}°")
        print(f"    → Prismatic Result: {pris:.6f}")
    print(f"\nFinal distance to target: {final_dist_to_target:.2f}")
    print(f"Final roll: {vector_to_spherical(final_ee)[0]:.1f}°, pitch: {vector_to_spherical(final_ee)[1]:.1f}°")
    print(f"Angular errors - Roll: {abs(vector_to_spherical(final_ee)[0] - target_roll):.1f}°, Pitch: {abs(vector_to_spherical(final_ee)[1] - target_pitch):.1f}°")
    print(f"Base error: {base_error:.4f}")
    print(f"Total iterations: {backward_result.iterations_used + forward_result.iterations_used}")
    
    return True

def main():
    if len(sys.argv) == 2:
        target_x, target_y, target_z = parse_coordinates(sys.argv[1])
    else:
        target_x, target_y, target_z = 100, 100, 300
        print("Usage: python3 test_fabrik_forward_visual.py x,y,z")
    
    print(f"Enhanced FABRIK visualization for target: ({target_x}, {target_y}, {target_z})")
    visualize_fabrik_enhanced(target_x, target_y, target_z)

if __name__ == "__main__":
    main()