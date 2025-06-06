#!/usr/bin/env python3
"""
Collision Data Viewer - Visualizes collision world from JSON
No Isaac Sim dependencies - works with any JSON collision data
"""

import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import argparse
import os

class CollisionViewer:
    def __init__(self, figsize=(12, 8)):
        self.fig = plt.figure(figsize=figsize)
        self.ax = self.fig.add_subplot(111, projection='3d')
        
    def load_collision_data(self, json_file):
        """Load collision data from JSON file"""
        try:
            with open(json_file, 'r') as f:
                data = json.load(f)
            print(f"Loaded collision data from: {json_file}")
            
            metadata = data.get('metadata', {})
            print(f"  Total primitives: {metadata.get('total_primitives', 0)}")
            print(f"  Spheres: {metadata.get('sphere_count', 0)}")
            print(f"  Boxes: {metadata.get('box_count', 0)}")
            print(f"  Cylinders: {metadata.get('cylinder_count', 0)}")
            
            return data
        except FileNotFoundError:
            print(f"Error: File '{json_file}' not found")
            return None
        except json.JSONDecodeError as e:
            print(f"Error: Invalid JSON in '{json_file}': {e}")
            return None
    
    def visualize_collision_world(self, collision_data, title="Collision World"):
        """Visualize complete collision world"""
        self.ax.clear()
        
        if not collision_data:
            print("No collision data to visualize")
            return
        
        primitive_count = 0
        
        # Draw spheres
        for i, sphere in enumerate(collision_data.get('spheres', [])):
            self.draw_sphere(sphere, color='blue', alpha=0.6, label=f'Sphere {i}')
            primitive_count += 1
        
        # Draw boxes
        for i, box in enumerate(collision_data.get('boxes', [])):
            color = 'green' if box.get('is_axis_aligned', True) else 'orange'
            label = f'Box {i} ({"AABB" if box.get("is_axis_aligned", True) else "OBB"})'
            self.draw_box(box, color=color, alpha=0.4, label=label)
            primitive_count += 1
        
        # Draw cylinders
        for i, cylinder in enumerate(collision_data.get('cylinders', [])):
            self.draw_cylinder(cylinder, color='red', alpha=0.5, label=f'Cylinder {i}')
            primitive_count += 1
        
        self.setup_3d_scene()
        self.ax.set_title(f"{title} ({primitive_count} primitives)")
        
        if primitive_count > 0:
            self.ax.legend()
        
        plt.tight_layout()
        plt.show()
    
    def draw_sphere(self, sphere_data, color='blue', alpha=0.6, label=None):
        """Draw a sphere from JSON data"""
        center = sphere_data['center']
        radius = sphere_data['radius']
        
        # Create sphere mesh
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        x = radius * np.outer(np.cos(u), np.sin(v)) + center[0]
        y = radius * np.outer(np.sin(u), np.sin(v)) + center[1]
        z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + center[2]
        
        self.ax.plot_surface(x, y, z, color=color, alpha=alpha, label=label)
        
        # Add center point
        self.ax.scatter(*center, color='black', s=20)
    
    def draw_box(self, box_data, color='green', alpha=0.4, label=None):
        """Draw a box from JSON data (supports both AABB and OBB)"""
        center = np.array(box_data['center'])
        size = np.array(box_data['size'])
        
        # Create box vertices (local coordinates)
        vertices = np.array([
            [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],  # Bottom face
            [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]      # Top face
        ]) * (size / 2)
        
        # Apply rotation if it's an OBB
        if not box_data.get('is_axis_aligned', True):
            # Convert quaternion to rotation matrix (simplified)
            quat = box_data['quaternion']  # [x, y, z, w]
            rotation_matrix = self.quaternion_to_rotation_matrix(quat)
            vertices = vertices @ rotation_matrix.T
        
        # Translate to center
        vertices += center
        
        # Define box faces
        faces = [
            [vertices[0], vertices[1], vertices[2], vertices[3]],  # Bottom
            [vertices[4], vertices[5], vertices[6], vertices[7]],  # Top
            [vertices[0], vertices[1], vertices[5], vertices[4]],  # Front
            [vertices[2], vertices[3], vertices[7], vertices[6]],  # Back
            [vertices[1], vertices[2], vertices[6], vertices[5]],  # Right
            [vertices[4], vertices[7], vertices[3], vertices[0]]   # Left
        ]
        
        # Add to plot
        poly3d = [[list(vertex) for vertex in face] for face in faces]
        collection = Poly3DCollection(poly3d, alpha=alpha, facecolor=color, edgecolor='black', linewidth=0.5)
        self.ax.add_collection3d(collection)
        
        # Add center point
        self.ax.scatter(*center, color='black', s=20)
    
    def draw_cylinder(self, cylinder_data, color='red', alpha=0.5, label=None):
        """Draw a cylinder from JSON data"""
        center = np.array(cylinder_data['center'])
        radius = cylinder_data['radius']
        height = cylinder_data['height']
        
        # Create cylinder mesh
        theta = np.linspace(0, 2*np.pi, 20)
        z_vals = np.array([center[2] - height/2, center[2] + height/2])
        
        # Cylinder surface
        for z in z_vals:
            x_circle = radius * np.cos(theta) + center[0]
            y_circle = radius * np.sin(theta) + center[1]
            z_circle = np.full_like(x_circle, z)
            self.ax.plot(x_circle, y_circle, z_circle, color=color, alpha=alpha)
        
        # Cylinder sides (simplified)
        for i in range(0, len(theta), 4):  # Draw some vertical lines
            x_line = [radius * np.cos(theta[i]) + center[0]] * 2
            y_line = [radius * np.sin(theta[i]) + center[1]] * 2
            z_line = [center[2] - height/2, center[2] + height/2]
            self.ax.plot(x_line, y_line, z_line, color=color, alpha=alpha)
        
        # Add center point
        self.ax.scatter(*center, color='black', s=20)
    
    def quaternion_to_rotation_matrix(self, quat):
        """Convert quaternion [x, y, z, w] to rotation matrix"""
        x, y, z, w = quat
        
        # Simplified rotation matrix from quaternion
        R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ])
        
        return R
    
    def setup_3d_scene(self):
        """Setup 3D scene with proper axes and labels"""
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        
        # Force equal scaling by setting limits based on data
        # Get current axis limits
        x_limits = self.ax.get_xlim()
        y_limits = self.ax.get_ylim()
        z_limits = self.ax.get_zlim()
        
        # Calculate the range for each axis
        x_range = x_limits[1] - x_limits[0]
        y_range = y_limits[1] - y_limits[0]
        z_range = z_limits[1] - z_limits[0]
        
        # Use the maximum range for all axes
        max_range = max(x_range, y_range, z_range)
        
        # Calculate centers
        x_center = (x_limits[1] + x_limits[0]) / 2
        y_center = (y_limits[1] + y_limits[0]) / 2
        z_center = (z_limits[1] + z_limits[0]) / 2
        
        # Set equal limits
        half_range = max_range / 2
        self.ax.set_xlim(x_center - half_range, x_center + half_range)
        self.ax.set_ylim(y_center - half_range, y_center + half_range)
        self.ax.set_zlim(z_center - half_range, z_center + half_range)
        
        # Set equal aspect ratio
        self.ax.set_box_aspect([1,1,1])
        
        # Grid
        self.ax.grid(True, alpha=0.3)
        
    def save_plot(self, filename):
        """Save current plot to file"""
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"Plot saved to: {filename}")

def create_test_json():
    """Create a test JSON file for development"""
    test_data = {
        "metadata": {
            "total_primitives": 4,
            "sphere_count": 1,
            "box_count": 2,
            "cylinder_count": 1
        },
        "spheres": [
            {
                "center": [100.0, 50.0, 200.0],
                "radius": 75.0,
                "quaternion": [0.0, 0.0, 0.0, 1.0],
                "is_uniform_scale": True
            }
        ],
        "boxes": [
            {
                "center": [-150.0, 100.0, 300.0],
                "size": [100.0, 50.0, 200.0],
                "quaternion": [0.0, 0.0, 0.0, 1.0],
                "is_axis_aligned": True
            },
            {
                "center": [200.0, -100.0, 250.0],
                "size": [80.0, 80.0, 150.0],
                "quaternion": [0.0, 0.0, 0.383, 0.924],
                "is_axis_aligned": False
            }
        ],
        "cylinders": [
            {
                "center": [0.0, -200.0, 180.0],
                "radius": 60.0,
                "height": 120.0,
                "quaternion": [0.0, 0.0, 0.0, 1.0],
                "is_circular": True,
                "is_axis_aligned": True
            }
        ]
    }
    
    filename = "test_collision_world.json"
    with open(filename, 'w') as f:
        json.dump(test_data, f, indent=2)
    
    print(f"Created test JSON file: {filename}")
    return filename

def main():
    parser = argparse.ArgumentParser(description='Visualize collision world from JSON')
    parser.add_argument('json_file', nargs='?', help='JSON file with collision data')
    parser.add_argument('--create-test', action='store_true', help='Create test JSON file')
    parser.add_argument('--save', help='Save plot to file')
    
    args = parser.parse_args()
    
    # Create test file if requested
    if args.create_test:
        test_file = create_test_json()
        if not args.json_file:
            args.json_file = test_file
    
    # Use test file if no file specified
    if not args.json_file:
        print("No JSON file specified. Use --create-test to create a test file.")
        print("Usage: python collision_viewer.py <json_file>")
        return
    
    # Check if file exists
    if not os.path.exists(args.json_file):
        print(f"File '{args.json_file}' not found.")
        print("Use --create-test to create a test file.")
        return
    
    # Create viewer and visualize
    viewer = CollisionViewer()
    collision_data = viewer.load_collision_data(args.json_file)
    
    if collision_data:
        viewer.visualize_collision_world(collision_data, 
                                       title=f"Collision World - {os.path.basename(args.json_file)}")
        
        # Save plot if requested
        if args.save:
            viewer.save_plot(args.save)

if __name__ == "__main__":
    main()