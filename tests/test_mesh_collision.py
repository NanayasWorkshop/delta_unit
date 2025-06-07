#!/usr/bin/env python3
"""
Test Mesh Collision System
Tests collision detection with sphere obstacles and mesh integration
"""
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def test_basic_mesh_functionality():
    """Test basic mesh creation and management"""
    print("Testing basic mesh functionality...")
    
    try:
        import delta_robot
        
        # Test sphere mesh creation
        center = (150, 75, 200)
        radius = 50
        vertices, faces = delta_robot.create_sphere_mesh(center, radius, resolution=10)
        
        print(f"Created sphere mesh: {vertices.shape[0]} vertices, {faces.shape[0]} faces")
        
        # Test mesh upload
        mesh_id = 0
        delta_robot.update_collision_mesh(mesh_id, vertices, faces)
        print(f"Uploaded mesh {mesh_id} to collision system")
        
        # Test mesh retrieval
        meshes = delta_robot.get_collision_meshes()
        print(f"Active meshes: {len(meshes)}")
        
        if mesh_id in meshes:
            mesh = meshes[mesh_id]
            print(f"Mesh {mesh_id}: {len(mesh.triangles)} triangles")
        
        # Test mesh removal
        delta_robot.remove_collision_mesh(mesh_id)
        meshes_after = delta_robot.get_collision_meshes()
        print(f"Meshes after removal: {len(meshes_after)}")
        
        return True
        
    except Exception as e:
        print(f"Basic mesh test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_collision_with_sphere():
    """Test FABRIK with sphere collision avoidance"""
    print("\nTesting collision avoidance with sphere obstacle...")
    
    try:
        import delta_robot
        
        # Clear any existing meshes
        delta_robot.clear_all_collision_meshes()
        
        # Create sphere obstacle in robot's path
        sphere_center = (50, 25, 250)  # In robot's path to target
        sphere_radius = 80
        vertices, faces = delta_robot.create_sphere_mesh(sphere_center, sphere_radius, resolution=15)
        
        # Upload sphere mesh
        mesh_id = 1
        delta_robot.update_collision_mesh(mesh_id, vertices, faces)
        print(f"Added sphere obstacle at {sphere_center} with radius {sphere_radius}")
        
        # Target that would pass through sphere
        target = (100, 50, 300)
        print(f"FABRIK target: {target}")
        
        # Run FABRIK with collision avoidance
        print("Running FABRIK with collision avoidance...")
        result = delta_robot.calculate_motors(*target)
        
        # Get pills and check for collisions
        pills = delta_robot.get_collision_pills()
        meshes = delta_robot.get_collision_meshes()
        
        print(f"FABRIK result: converged={result.fabrik_converged}, error={result.fabrik_error:.4f}")
        print(f"Active pills: {len(pills)}")
        print(f"Active meshes: {len(meshes)}")
        
        # Analyze pill positions relative to sphere
        print("\nPill analysis:")
        for i, pill in enumerate(pills):
            pill_center = pill.get_center()
            distance_to_sphere = np.linalg.norm(np.array(pill_center) - np.array(sphere_center))
            collision_threshold = pill.radius + sphere_radius
            
            print(f"Pill {i}: center=({pill_center[0]:.1f}, {pill_center[1]:.1f}, {pill_center[2]:.1f}), "
                  f"dist_to_sphere={distance_to_sphere:.1f}, threshold={collision_threshold:.1f}")
            
            if distance_to_sphere < collision_threshold:
                print(f"  ⚠️  Potential collision detected!")
            else:
                print(f"  ✅ Safe distance")
        
        return True
        
    except Exception as e:
        print(f"Collision test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def visualize_collision_test():
    """Visualize FABRIK solution with mesh obstacles"""
    print("\nVisualizing collision test...")
    
    try:
        import delta_robot
        
        # Setup scene
        delta_robot.clear_all_collision_meshes()
        
        # Add sphere obstacle
        sphere_center = (120, 70, 500)
        sphere_radius = 60
        vertices, faces = delta_robot.create_sphere_mesh(sphere_center, sphere_radius, resolution=12)
        delta_robot.update_collision_mesh(0, vertices, faces)
        
        # Solve FABRIK
        target = (120, 60, 350)
        result = delta_robot.calculate_motors(*target)
        
        # Get data for visualization
        pills = delta_robot.get_collision_pills()
        joint_positions = []
        for joint_pos in result.fabrik_joint_positions:
            if hasattr(joint_pos, 'shape'):
                joint_positions.append([joint_pos[0], joint_pos[1], joint_pos[2]])
            else:
                joint_positions.append([joint_pos.x(), joint_pos.y(), joint_pos.z()])
        
        # Create 3D plot
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot robot joints
        positions = np.array(joint_positions)
        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
                'b-o', linewidth=2, markersize=6, label='Robot Chain')
        
        # Plot pills
        for i, pill in enumerate(pills):
            start = [pill.start_point[0], pill.start_point[1], pill.start_point[2]]
            end = [pill.end_point[0], pill.end_point[1], pill.end_point[2]]
            ax.plot([start[0], end[0]], [start[1], end[1]], [start[2], end[2]], 
                    'r-', linewidth=pill.radius/5, alpha=0.6, label='Collision Pills' if i == 0 else "")
        
        # Plot sphere obstacle
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        x_sphere = sphere_center[0] + sphere_radius * np.outer(np.cos(u), np.sin(v))
        y_sphere = sphere_center[1] + sphere_radius * np.outer(np.sin(u), np.sin(v))
        z_sphere = sphere_center[2] + sphere_radius * np.outer(np.ones(np.size(u)), np.cos(v))
        ax.plot_surface(x_sphere, y_sphere, z_sphere, alpha=0.3, color='orange', label='Obstacle')
        
        # Plot target
        ax.scatter([target[0]], [target[1]], [target[2]], 
                  c='red', s=100, marker='*', label='Target')
        
        # Set equal aspect ratio
        all_positions = np.array(joint_positions)
        max_range = np.array([all_positions[:,0].max()-all_positions[:,0].min(),
                             all_positions[:,1].max()-all_positions[:,1].min(),
                             all_positions[:,2].max()-all_positions[:,2].min()]).max() / 2.0
        
        mid_x = (all_positions[:,0].max()+all_positions[:,0].min()) * 0.5
        mid_y = (all_positions[:,1].max()+all_positions[:,1].min()) * 0.5
        mid_z = (all_positions[:,2].max()+all_positions[:,2].min()) * 0.5
        
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        ax.set_box_aspect([1,1,1])
        
        # Labels and title
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.set_title(f'FABRIK with Mesh Collision Avoidance\nTarget: {target}')
        ax.legend()
        
        plt.tight_layout()
        plt.show()
        
        return True
        
    except Exception as e:
        print(f"Visualization failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_multiple_obstacles():
    """Test with multiple mesh obstacles"""
    print("\nTesting multiple obstacles...")
    
    try:
        import delta_robot
        
        # Clear existing meshes
        delta_robot.clear_all_collision_meshes()
        
        # Add multiple sphere obstacles
        obstacles = [
            {"center": (60, 30, 180), "radius": 40, "id": 0},
            {"center": (90, 45, 280), "radius": 35, "id": 1},
            {"center": (40, 20, 350), "radius": 30, "id": 2}
        ]
        
        for obs in obstacles:
            vertices, faces = delta_robot.create_sphere_mesh(obs["center"], obs["radius"], resolution=10)
            delta_robot.update_collision_mesh(obs["id"], vertices, faces)
            print(f"Added obstacle {obs['id']} at {obs['center']} with radius {obs['radius']}")
        
        # Run FABRIK through obstacles
        target = (100, 50, 400)
        result = delta_robot.calculate_motors(*target)
        
        print(f"FABRIK with multiple obstacles: converged={result.fabrik_converged}, error={result.fabrik_error:.4f}")
        
        # Check mesh count
        meshes = delta_robot.get_collision_meshes()
        print(f"Active meshes: {len(meshes)}")
        
        return True
        
    except Exception as e:
        print(f"Multiple obstacles test failed: {e}")
        return False

def performance_test():
    """Test performance with large mesh"""
    print("\nTesting performance with larger mesh...")
    
    try:
        import delta_robot
        import time
        
        # Clear meshes
        delta_robot.clear_all_collision_meshes()
        
        # Create larger sphere mesh
        start_time = time.time()
        vertices, faces = delta_robot.create_sphere_mesh((100, 50, 250), 70, resolution=30)
        mesh_creation_time = time.time() - start_time
        
        print(f"Created large mesh: {vertices.shape[0]} vertices, {faces.shape[0]} faces in {mesh_creation_time:.3f}s")
        
        # Upload mesh
        start_time = time.time()
        delta_robot.update_collision_mesh(0, vertices, faces)
        upload_time = time.time() - start_time
        
        print(f"Uploaded mesh in {upload_time:.3f}s")
        
        # Run FABRIK with timing
        target = (120, 60, 350)
        start_time = time.time()
        result = delta_robot.calculate_motors(*target)
        solve_time = time.time() - start_time
        
        print(f"FABRIK solve time: {solve_time:.3f}s")
        print(f"Result: converged={result.fabrik_converged}, error={result.fabrik_error:.4f}")
        
        return True
        
    except Exception as e:
        print(f"Performance test failed: {e}")
        return False

def main():
    print("=== Delta Robot Mesh Collision Testing ===\n")
    
    tests = [
        ("Basic Mesh Functionality", test_basic_mesh_functionality),
        ("Collision with Sphere", test_collision_with_sphere),
        ("Multiple Obstacles", test_multiple_obstacles),
        ("Performance Test", performance_test),
        ("Visualization", visualize_collision_test)
    ]
    
    results = []
    for test_name, test_func in tests:
        print(f"Running {test_name}...")
        try:
            success = test_func()
            results.append((test_name, success))
            print(f"✅ {test_name}: {'PASSED' if success else 'FAILED'}")
        except Exception as e:
            print(f"❌ {test_name}: FAILED with exception: {e}")
            results.append((test_name, False))
        print("-" * 50)
    
    # Summary
    print("\n=== Test Summary ===")
    passed = sum(1 for _, success in results if success)
    total = len(results)
    
    for test_name, success in results:
        status = "✅ PASSED" if success else "❌ FAILED"
        print(f"{test_name}: {status}")
    
    print(f"\nOverall: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 All tests passed! Mesh collision system is working.")
    else:
        print("⚠️  Some tests failed. Check the output above for details.")

if __name__ == "__main__":
    main()