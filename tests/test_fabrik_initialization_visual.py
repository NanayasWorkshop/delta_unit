#!/usr/bin/env python3
"""
Visual test for FABRIK Initialization module using matplotlib
"""

import sys
import os
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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

def get_joint_color(joint_type):
    """Get color for different joint types"""
    type_colors = {
        'FIXED_BASE': 'red',
        'SPHERICAL_120': 'blue', 
        'END_EFFECTOR': 'green'
    }
    
    # Handle different ways the joint type might be represented
    type_str = str(joint_type)
    if 'FIXED' in type_str or 'BASE' in type_str:
        return type_colors['FIXED_BASE']
    elif 'SPHERICAL' in type_str:
        return type_colors['SPHERICAL_120']
    elif 'END_EFFECTOR' in type_str or 'END' in type_str:
        return type_colors['END_EFFECTOR']
    else:
        return 'gray'

def get_joint_marker(joint_type):
    """Get marker style for different joint types"""
    type_markers = {
        'FIXED_BASE': 's',      # square
        'SPHERICAL_120': 'o',   # circle
        'END_EFFECTOR': '^'     # triangle
    }
    
    type_str = str(joint_type)
    if 'FIXED' in type_str or 'BASE' in type_str:
        return type_markers['FIXED_BASE']
    elif 'SPHERICAL' in type_str:
        return type_markers['SPHERICAL_120']
    elif 'END_EFFECTOR' in type_str or 'END' in type_str:
        return type_markers['END_EFFECTOR']
    else:
        return 'o'

def plot_robot_chain_3d(result, num_segments):
    """Create 3D visualization of the robot chain"""
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Extract joint positions
    positions = []
    colors = []
    markers = []
    labels = []
    
    for i, joint in enumerate(result.chain.joints):
        positions.append([joint.position.x, joint.position.y, joint.position.z])
        colors.append(get_joint_color(joint.type))
        markers.append(get_joint_marker(joint.type))
        
        # Create labels
        type_str = str(joint.type)
        if 'FIXED' in type_str or 'BASE' in type_str:
            labels.append(f"Base")
        elif 'SPHERICAL' in type_str:
            segment_num = i  # Joint i corresponds to segment i
            labels.append(f"Joint {segment_num}")
        elif 'END_EFFECTOR' in type_str:
            labels.append("End Effector")
        else:
            labels.append(f"Joint {i}")
    
    positions = np.array(positions)
    
    # Plot joints with different markers and colors
    unique_types = list(set([str(j.type) for j in result.chain.joints]))
    
    for joint_type in unique_types:
        mask = [str(j.type) == joint_type for j in result.chain.joints]
        joint_positions = positions[mask]
        
        if len(joint_positions) > 0:
            color = get_joint_color(joint_type)
            marker = get_joint_marker(joint_type)
            
            # Clean up label
            if 'FIXED' in joint_type or 'BASE' in joint_type:
                label = 'Base (Fixed)'
            elif 'SPHERICAL' in joint_type:
                label = 'Spherical Joints (120°)'
            elif 'END_EFFECTOR' in joint_type:
                label = 'End Effector'
            else:
                label = joint_type
            
            ax.scatter(joint_positions[:, 0], joint_positions[:, 1], joint_positions[:, 2],
                      c=color, marker=marker, s=100, label=label, alpha=0.8)
    
    # Draw segments (connecting lines)
    for i, segment in enumerate(result.chain.segments):
        start_pos = positions[segment.start_joint_index]
        end_pos = positions[segment.end_joint_index]
        
        ax.plot([start_pos[0], end_pos[0]], 
                [start_pos[1], end_pos[1]], 
                [start_pos[2], end_pos[2]], 
                'k-', linewidth=2, alpha=0.6)
        
        # Add segment length annotation at midpoint
        mid_pos = (start_pos + end_pos) / 2
        ax.text(mid_pos[0], mid_pos[1], mid_pos[2], 
                f'L={segment.length:.1f}', fontsize=8, alpha=0.7)
    
    # Add joint position labels
    for i, (pos, label) in enumerate(zip(positions, labels)):
        ax.text(pos[0]+2, pos[1]+2, pos[2]+2, 
                f'{label}\n({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f})', 
                fontsize=8, alpha=0.8)
    
    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'{num_segments}-Segment Delta Robot Chain\n(FABRIK Initialization - Straight Up)')
    
    # Add legend
    ax.legend()
    
    # Set equal aspect ratio
    max_range = np.array([positions[:,0].max()-positions[:,0].min(),
                         positions[:,1].max()-positions[:,1].min(),
                         positions[:,2].max()-positions[:,2].min()]).max() / 2.0
    
    mid_x = (positions[:,0].max()+positions[:,0].min()) * 0.5
    mid_y = (positions[:,1].max()+positions[:,1].min()) * 0.5
    mid_z = (positions[:,2].max()+positions[:,2].min()) * 0.5
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(0, positions[:,2].max() + 20)
    
    # Add grid
    ax.grid(True, alpha=0.3)
    
    return fig, ax

# Removed - only keeping 3D visualization

def test_fabrik_initialization_visual():
    """Main visual test function"""
    # Import modules first to get constants
    
    try:
        import delta_robot.fabrik_initialization as fi
        import delta_robot.delta_types as dt
        import delta_robot
    except ImportError as e:
        print(f"Error: Module not found: {e}")
        print("Build the fabrik_initialization module first.")
        return False

    # Parse command line arguments - USE THE CONSTANT FROM HPP!
    if len(sys.argv) == 2:
        num_segments = parse_segments(sys.argv[1])
        print(f"Using command line segments: {num_segments}")
    else:
        num_segments = delta_robot.DEFAULT_ROBOT_SEGMENTS  # ✅ USE THE CONSTANT!
        print(f"Using default segments from C++ constant: {num_segments}")
        print("Usage: python3 test_fabrik_initialization_visual.py <num_segments>")
    
    try:
        import delta_robot.fabrik_initialization as fi
        import delta_robot.delta_types as dt
        import delta_robot
    except ImportError as e:
        print(f"Error: Module not found: {e}")
        print("Build the fabrik_initialization module first.")
        return False
    
    print("Visual Testing FABRIK Initialization Module")
    print("=" * 50)
    
    # Initialize the robot
    print(f"Initializing {num_segments}-segment robot chain (straight up)")
    result = fi.FabrikInitialization.initialize_straight_up(num_segments)
    
    # Print basic info
    print(f"Chain created with {len(result.chain.joints)} joints and {len(result.chain.segments)} segments")
    print(f"End-effector at: ({result.final_end_effector.x:.1f}, {result.final_end_effector.y:.1f}, {result.final_end_effector.z:.1f})")
    print(f"Total reach: {result.total_reach:.1f}")
    
    # Create 3D visualization only
    print("Creating 3D visualization...")
    
    fig, ax = plot_robot_chain_3d(result, num_segments)
    
    # Show the plot
    plt.show()
    
    print("\n" + "="*50)
    print("✓ 3D visualization completed!")
    print("✓ FABRIK initialization structure displayed")
    print("\nNext steps:")
    print("  - Implement FABRIK backward iteration")
    print("  - Add target position visualization")
    print("  - Test complete FABRIK algorithm")
    
    return True

if __name__ == "__main__":
    try:
        test_fabrik_initialization_visual()
    except ImportError:
        print("Error: matplotlib not found. Install with:")
        print("  pip install matplotlib")
        sys.exit(1)
    except Exception as e:
        print(f"Error during visualization: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)