#!/usr/bin/env python3
"""
Simple FABRIK Visual Debugger
Shows FABRIK chain (red) vs Actual Segment End-Effectors (green)
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

def calculate_actual_segment_end_effectors(result):
    """Calculate the actual physical segment end-effector positions"""
    if not result.segment_end_effectors:
        return []
    
    # Use the segment end-effector data from the solver result
    actual_positions = [(0, 0, 0)]  # Start from base
    
    for seg_data in result.segment_end_effectors:
        pos = seg_data.end_effector_position
        actual_positions.append((pos.x, pos.y, pos.z))
    
    return actual_positions

def extract_fabrik_chain_positions(chain):
    """Extract FABRIK chain joint positions"""
    positions = []
    for joint in chain.joints:
        positions.append((joint.position.x, joint.position.y, joint.position.z))
    return positions

def visualize_fabrik_vs_actual(target_x, target_y, target_z):
    """Visualize FABRIK chain vs actual segment end-effectors"""
    
    try:
        import delta_robot.fabrik_solver as fs
        import delta_robot.fabrik_initialization as fi
        import delta_robot.delta_types as dt
    except ImportError as e:
        print(f"Error: delta_robot package not found: {e}")
        return False
    
    # Initialize and solve
    print(f"Solving for target ({target_x}, {target_y}, {target_z})...")
    
    init_result = fi.FabrikInitialization.initialize_straight_up(3)
    target_position = dt.Vector3(target_x, target_y, target_z)
    
    # Solve with default configuration
    result = fs.FabrikSolver.solve(init_result.chain, target_position)
    
    print(f"Converged: {'Yes' if result.converged else 'No'}")
    print(f"Final error: {result.final_error:.6f}")
    print(f"Segment end-effectors found: {len(result.segment_end_effectors)}")
    
    # Extract positions
    fabrik_positions = extract_fabrik_chain_positions(result.final_chain)
    actual_positions = calculate_actual_segment_end_effectors(result)
    
    print(f"\nFABRIK Chain Positions:")
    for i, pos in enumerate(fabrik_positions):
        print(f"  Joint {i}: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
    
    print(f"\nActual Segment End-Effector Positions:")
    for i, pos in enumerate(actual_positions):
        if i == 0:
            print(f"  Base: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
        else:
            print(f"  Segment {i} EE: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
    
    # Create visualization
    fig = plt.figure(figsize=(15, 5))
    
    # 3D view
    ax_3d = fig.add_subplot(131, projection='3d')
    
    # Plot target
    ax_3d.scatter([target_x], [target_y], [target_z], color='gold', s=300, 
                  marker='X', label='Target', edgecolor='black', linewidth=2)
    
    # Plot FABRIK chain (RED)
    if fabrik_positions:
        fabrik_array = np.array(fabrik_positions)
        ax_3d.plot(fabrik_array[:, 0], fabrik_array[:, 1], fabrik_array[:, 2], 
                  'r-o', linewidth=3, markersize=8, label='FABRIK Chain', alpha=0.8)
        
        # Mark joints
        for i, pos in enumerate(fabrik_positions):
            if i == 0:
                ax_3d.scatter(*pos, color='black', s=150, marker='s', 
                             edgecolor='white', linewidth=2, label='Base')
            elif i == len(fabrik_positions) - 1:
                ax_3d.scatter(*pos, color='red', s=150, marker='*', 
                             edgecolor='white', linewidth=2, label='FABRIK End-Effector')
    
    # Plot Actual Segment End-Effectors (GREEN)
    if actual_positions and len(actual_positions) > 1:
        actual_array = np.array(actual_positions)
        ax_3d.plot(actual_array[:, 0], actual_array[:, 1], actual_array[:, 2], 
                  'g-D', linewidth=3, markersize=8, label='Actual Segment EEs', alpha=0.8)
        
        # Mark segment end-effectors
        for i, pos in enumerate(actual_positions[1:], 1):  # Skip base
            ax_3d.scatter(*pos, color='green', s=100, marker='D', 
                         edgecolor='white', linewidth=1)
            ax_3d.text(pos[0], pos[1], pos[2] + 10, f'S{i}', 
                      fontsize=9, ha='center', va='bottom', color='green', weight='bold')
    
    # Set 3D properties
    max_range = max(abs(target_x), abs(target_y), abs(target_z), 400) * 1.1
    ax_3d.set_xlim([0, max_range])
    ax_3d.set_ylim([0, max_range])
    ax_3d.set_zlim([0, max_range])
    ax_3d.set_xlabel('X')
    ax_3d.set_ylabel('Y')
    ax_3d.set_zlabel('Z')
    ax_3d.set_title('FABRIK vs Actual Segment End-Effectors')
    ax_3d.legend()
    
    # XY projection
    ax_xy = fig.add_subplot(132)
    ax_xy.scatter([target_x], [target_y], color='gold', s=300, marker='X', 
                 label='Target', edgecolor='black', linewidth=2)
    
    if fabrik_positions:
        fabrik_array = np.array(fabrik_positions)
        ax_xy.plot(fabrik_array[:, 0], fabrik_array[:, 1], 'r-o', 
                  linewidth=2, markersize=6, label='FABRIK Chain', alpha=0.8)
    
    if actual_positions and len(actual_positions) > 1:
        actual_array = np.array(actual_positions)
        ax_xy.plot(actual_array[:, 0], actual_array[:, 1], 'g-D', 
                  linewidth=2, markersize=6, label='Actual Segment EEs', alpha=0.8)
    
    ax_xy.set_xlabel('X')
    ax_xy.set_ylabel('Y')
    ax_xy.set_title('XY Projection')
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend()
    ax_xy.axis('equal')
    
    # XZ projection
    ax_xz = fig.add_subplot(133)
    ax_xz.scatter([target_x], [target_z], color='gold', s=300, marker='X', 
                 label='Target', edgecolor='black', linewidth=2)
    
    if fabrik_positions:
        fabrik_array = np.array(fabrik_positions)
        ax_xz.plot(fabrik_array[:, 0], fabrik_array[:, 2], 'r-o', 
                  linewidth=2, markersize=6, label='FABRIK Chain', alpha=0.8)
    
    if actual_positions and len(actual_positions) > 1:
        actual_array = np.array(actual_positions)
        ax_xz.plot(actual_array[:, 0], actual_array[:, 2], 'g-D', 
                  linewidth=2, markersize=6, label='Actual Segment EEs', alpha=0.8)
    
    ax_xz.set_xlabel('X')
    ax_xz.set_ylabel('Z')
    ax_xz.set_title('XZ Projection')
    ax_xz.grid(True, alpha=0.3)
    ax_xz.legend()
    
    plt.tight_layout()
    
    # Print comparison
    print(f"\n=== COMPARISON ===")
    if fabrik_positions and actual_positions:
        fabrik_final = fabrik_positions[-1]
        actual_final = actual_positions[-1] if len(actual_positions) > 1 else (0, 0, 0)
        
        print(f"FABRIK final end-effector: ({fabrik_final[0]:.3f}, {fabrik_final[1]:.3f}, {fabrik_final[2]:.3f})")
        print(f"Actual final segment EE:   ({actual_final[0]:.3f}, {actual_final[1]:.3f}, {actual_final[2]:.3f})")
        print(f"Target position:           ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
        
        # Calculate distances to target
        fabrik_dist = math.sqrt((fabrik_final[0] - target_x)**2 + 
                               (fabrik_final[1] - target_y)**2 + 
                               (fabrik_final[2] - target_z)**2)
        actual_dist = math.sqrt((actual_final[0] - target_x)**2 + 
                               (actual_final[1] - target_y)**2 + 
                               (actual_final[2] - target_z)**2)
        
        print(f"\nDistance to target:")
        print(f"  FABRIK: {fabrik_dist:.6f}")
        print(f"  Actual: {actual_dist:.6f}")
        print(f"  Difference: {abs(fabrik_dist - actual_dist):.6f}")
        
        if actual_dist > 50:  # Arbitrary threshold
            print(f"\n⚠️  WARNING: Actual segment end-effector is far from target!")
            print(f"This indicates an issue with segment position calculation.")
    
    plt.show()
    return True

def main():
    if len(sys.argv) == 2:
        target_x, target_y, target_z = parse_coordinates(sys.argv[1])
    else:
        target_x, target_y, target_z = 100, 100, 300
        print("Usage: python3 visual_fabrik_debugger.py x,y,z")
        print(f"Using default target: ({target_x}, {target_y}, {target_z})")
    
    visualize_fabrik_vs_actual(target_x, target_y, target_z)

if __name__ == "__main__":
    main()