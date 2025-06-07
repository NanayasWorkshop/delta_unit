#!/usr/bin/env python3
"""
Test Collision Pill Visualization
Shows FABRIK joint positions and collision pills in 3D
"""
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def plot_pill_as_cylinder(ax, start_point, end_point, radius, color='red', alpha=0.3):
    """Plot a pill as a cylinder with spherical caps"""
    # Convert to numpy arrays if they're not already
    start = np.array(start_point)
    end = np.array(end_point)
    
    # Calculate cylinder properties
    direction = end - start
    length = np.linalg.norm(direction)
    
    if length < 1e-6:
        return  # Skip zero-length pills
    
    direction_norm = direction / length
    
    # Create cylinder
    theta = np.linspace(0, 2*np.pi, 20)
    
    # Find two perpendicular vectors to the cylinder axis
    if abs(direction_norm[2]) < 0.9:
        perp1 = np.cross(direction_norm, [0, 0, 1])
    else:
        perp1 = np.cross(direction_norm, [1, 0, 0])
    perp1 = perp1 / np.linalg.norm(perp1)
    perp2 = np.cross(direction_norm, perp1)
    
    # Generate cylinder surface points
    cylinder_x = []
    cylinder_y = []
    cylinder_z = []
    
    for t in [0, 1]:  # Start and end circles
        circle_center = start + t * direction
        for angle in theta:
            point = circle_center + radius * (np.cos(angle) * perp1 + np.sin(angle) * perp2)
            cylinder_x.append(point[0])
            cylinder_y.append(point[1])
            cylinder_z.append(point[2])
    
    # Plot cylinder wireframe
    n_theta = len(theta)
    for i in range(n_theta):
        # Connect corresponding points on start and end circles
        ax.plot([cylinder_x[i], cylinder_x[i + n_theta]], 
                [cylinder_y[i], cylinder_y[i + n_theta]], 
                [cylinder_z[i], cylinder_z[i + n_theta]], 
                color=color, alpha=alpha)
    
    # Plot end circles
    for t in [0, 1]:
        circle_x = cylinder_x[t*n_theta:(t+1)*n_theta]
        circle_y = cylinder_y[t*n_theta:(t+1)*n_theta]
        circle_z = cylinder_z[t*n_theta:(t+1)*n_theta]
        ax.plot(circle_x + [circle_x[0]], circle_y + [circle_y[0]], circle_z + [circle_z[0]], 
                color=color, alpha=alpha)

def plot_joint_chain(ax, joint_positions, color='blue', marker='o', linewidth=2):
    """Plot joint positions connected by lines"""
    positions = np.array(joint_positions)
    
    # Plot joints as points
    ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], 
              c=color, s=50, marker=marker, alpha=0.8)
    
    # Plot connections between joints
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
            color=color, linewidth=linewidth, alpha=0.6)
    
    # Label joints
    for i, pos in enumerate(positions):
        ax.text(pos[0], pos[1], pos[2], f'J{i}', fontsize=8)

def extract_joint_positions(motor_result):
    """Extract joint positions from motor result"""
    joint_positions = []
    for joint_pos in motor_result.fabrik_joint_positions:
        if hasattr(joint_pos, 'shape'):  # numpy array
            joint_positions.append([joint_pos[0], joint_pos[1], joint_pos[2]])
        else:
            # Handle Eigen vector case
            joint_positions.append([joint_pos.x(), joint_pos.y(), joint_pos.z()])
    return joint_positions

def main():
    try:
        import delta_robot
        
        # Test coordinates
        target_x, target_y, target_z = 100, 50, 300
        print(f"Testing collision pill visualization for target: ({target_x}, {target_y}, {target_z})")
        
        # Run FABRIK solver
        print("Running FABRIK solver...")
        result = delta_robot.calculate_motors(target_x, target_y, target_z)
        
        if result is None:
            print("No result from motor calculation")
            return
        
        # Get collision pills directly
        print("Getting collision pills...")
        pills = delta_robot.delta_robot_complete.get_collision_pills()
        
        print(f"Found {len(pills)} collision pills")
        
        # Extract joint positions
        joint_positions = extract_joint_positions(result)
        print(f"Found {len(joint_positions)} joint positions")
        
        # Create 3D plot
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot joint chain
        plot_joint_chain(ax, joint_positions, color='blue', marker='o')
        
        # Plot collision pills
        colors = ['red', 'orange', 'yellow', 'green', 'cyan', 'purple', 'pink']
        for i, pill in enumerate(pills):
            color = colors[i % len(colors)]
            
            # Get pill points
            start_point = [pill.start_point[0], pill.start_point[1], pill.start_point[2]]
            end_point = [pill.end_point[0], pill.end_point[1], pill.end_point[2]]
            
            print(f"Pill {i}: Joint {pill.associated_joint_index}, "
                  f"Start: ({start_point[0]:.1f}, {start_point[1]:.1f}, {start_point[2]:.1f}), "
                  f"End: ({end_point[0]:.1f}, {end_point[1]:.1f}, {end_point[2]:.1f}), "
                  f"Radius: {pill.radius}")
            
            # Plot pill
            plot_pill_as_cylinder(ax, start_point, end_point, pill.radius, color=color, alpha=0.3)
            
            # Mark pill center
            center = pill.get_center()
            ax.scatter([center[0]], [center[1]], [center[2]], 
                      c=color, s=20, marker='x', alpha=0.8)
        
        # Plot target point
        ax.scatter([target_x], [target_y], [target_z], 
                  c='red', s=100, marker='*', alpha=1.0, label='Target')
        
        # Set labels and title
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.set_title(f'FABRIK Solution with Collision Pills\nTarget: ({target_x}, {target_y}, {target_z})')
        
        # Set equal aspect ratio
        ax.set_box_aspect([1,1,1])
        
        # Add legend
        ax.legend()
        
        # Show plot
        plt.tight_layout()
        plt.show()
        
        # Print pill summary
        print(f"\n--- Collision Pill Summary ---")
        print(f"Total pills: {len(pills)}")
        print(f"Pill radius: {delta_robot.delta_robot_complete.COLLISION_PILL_RADIUS} mm")
        print(f"FABRIK converged: {result.fabrik_converged}")
        print(f"FABRIK error: {result.fabrik_error:.4f}")
        
    except ImportError as e:
        print(f"ImportError: {e}")
        print("Please ensure the compiled C++ module is in the Python path.")
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()