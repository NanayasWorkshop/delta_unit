#!/usr/bin/env python3
"""
Test Complete Collision-Aware Solver with Advanced Visualization (FIXED VERSION)
Tests the main CollisionAwareSolver orchestrator that combines all pipeline phases
"""
import sys
import os
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def create_obstacle_objects(obstacle_data):
    """Create obstacle objects from (center, radius) tuples"""
    try:
        import delta_robot
        obstacles = []
        for center, radius in obstacle_data:
            obstacles.append(delta_robot.collision.Obstacle(center, radius))
        return obstacles
    except Exception as e:
        print(f"Error creating obstacle objects: {e}")
        return []

def test_basic_collision_pipeline():
    """Test the basic collision detection pipeline step by step"""
    try:
        import delta_robot
        
        print("=== TESTING BASIC COLLISION PIPELINE ===")
        
        # Step 1: Create a simple FABRIK solution
        print("1. Creating FABRIK solution...")
        target = np.array([100, 0, 300])
        fabrik_result = delta_robot.fabrik.solve_delta_robot(7, target, delta_robot.FABRIK_TOLERANCE)
        print(f"   FABRIK converged: {fabrik_result.converged}")
        
        # Step 2: Extract U points
        print("2. Extracting U points...")
        u_points = delta_robot.collision.UPointsExtractor.extract_u_points(fabrik_result.final_chain)
        print(f"   Extracted {len(u_points)} U points")
        
        # Step 3: Create obstacles
        print("3. Creating obstacles...")
        obstacles = delta_robot.collision.CollisionDetector.create_test_obstacles()
        print(f"   Created {len(obstacles)} obstacles")
        
        # Step 4: Test collision detection
        print("4. Testing collision detection...")
        collision_result = delta_robot.collision.CollisionDetector.check_and_avoid(
            u_points, obstacles, delta_robot.DEFAULT_SPLINE_DIAMETER
        )
        print(f"   Has collision: {collision_result.has_collision}")
        print(f"   Min distance: {collision_result.min_distance:.2f}mm")
        print(f"   Computation time: {collision_result.computation_time:.2f}ms")
        print(f"   Got {len(collision_result.waypoints)} waypoints")
        
        # Step 5: Test waypoint conversion
        if len(collision_result.waypoints) > 0:
            print("5. Testing waypoint conversion...")
            conversion_result = delta_robot.collision.WaypointConverter.convert_waypoints_to_joints(
                collision_result.waypoints
            )
            print(f"   Conversion successful: {conversion_result.conversion_successful}")
            if conversion_result.conversion_successful:
                print(f"   Converted to {len(conversion_result.joint_positions)} joint positions")
        
        return True
        
    except Exception as e:
        print(f"Basic pipeline test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_collision_aware_solver_simple():
    """Test collision-aware solver with simple scenarios"""
    try:
        import delta_robot
        
        print("\n=== TESTING COLLISION-AWARE SOLVER ===")
        
        # Test Scenario: With test obstacles
        print("Testing with standard test obstacles")
        target = np.array([100, 50, 300])
        obstacles = delta_robot.collision.CollisionDetector.create_test_obstacles()
        
        result = delta_robot.collision.CollisionAwareSolver.solve(target, obstacles, 3)
        print(f"   Collision-free: {result.collision_free}")
        print(f"   FABRIK converged: {result.fabrik_result.converged}")
        print(f"   Collision iterations: {result.collision_iterations}")
        print(f"   Total collision time: {result.total_collision_time_ms:.2f}ms")
        
        return True
        
    except Exception as e:
        print(f"Collision-aware solver test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def create_detailed_pipeline_visualization():
    """Create detailed visualization of the complete collision-aware pipeline"""
    try:
        import delta_robot
        
        print("\n=== CREATING DETAILED PIPELINE VISUALIZATION ===")
        
        # Setup test scenario
        target = np.array([100, 50, 300])
        obstacles = delta_robot.collision.CollisionDetector.create_test_obstacles()
        
        print("Running complete collision-aware pipeline...")
        
        # Step 1: Initial FABRIK solution
        print("Step 1: Initial FABRIK solution")
        initial_fabrik = delta_robot.fabrik.solve_delta_robot(7, target, delta_robot.FABRIK_TOLERANCE)
        
        # Step 2: Extract U points from initial FABRIK
        print("Step 2: Extracting U points")
        u_points = delta_robot.collision.UPointsExtractor.extract_u_points(initial_fabrik.final_chain)
        
        # Step 3: Run collision detection and get swarm-optimized waypoints
        print("Step 3: Collision detection and swarm optimization")
        collision_result = delta_robot.collision.CollisionDetector.check_and_avoid(
            u_points, obstacles, delta_robot.DEFAULT_SPLINE_DIAMETER
        )
        
        # Step 4: Convert waypoints back to joint positions
        print("Step 4: Converting waypoints to joint positions")
        conversion_result = delta_robot.collision.WaypointConverter.convert_waypoints_to_joints(
            collision_result.waypoints
        )
        
        # Step 5: Final FABRIK solution with collision-free initial positions
        print("Step 5: Final FABRIK solution")
        if conversion_result.conversion_successful:
            final_fabrik_chain = delta_robot.collision.WaypointConverter.create_fabrik_chain_from_waypoints(
                collision_result.waypoints, 7
            )
            final_fabrik = delta_robot.fabrik.FabrikSolver.solve(final_fabrik_chain, target)
        else:
            final_fabrik = initial_fabrik
        
        # Create visualization
        print("Creating visualization...")
        fig = go.Figure()
        
        # Add obstacles
        for i, obs in enumerate(obstacles):
            center = obs.center
            radius = obs.radius
            
            # Create sphere
            u = np.linspace(0, 2 * np.pi, 15)
            v = np.linspace(0, np.pi, 15)
            x = radius * np.outer(np.cos(u), np.sin(v)) + center[0]
            y = radius * np.outer(np.sin(u), np.sin(v)) + center[1]
            z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + center[2]
            
            fig.add_trace(go.Surface(
                x=x, y=y, z=z, 
                opacity=0.5, 
                colorscale='Reds', 
                showscale=False, 
                name=f'Obstacle {i+1}'
            ))
        
        # Step 1: Initial FABRIK solution (blue)
        initial_joints = []
        for joint in initial_fabrik.final_chain.joints:
            pos = joint.position
            initial_joints.append([pos[0], pos[1], pos[2]])
        initial_joints = np.array(initial_joints)
        
        fig.add_trace(go.Scatter3d(
            x=initial_joints[:, 0], 
            y=initial_joints[:, 1], 
            z=initial_joints[:, 2],
            mode='markers+lines',
            marker=dict(size=5, color='blue'),
            line=dict(color='blue', width=3, dash='dash'),
            name='1. Initial FABRIK'
        ))
        
        # Step 2: U points (red)
        u_points_array = []
        for u_point in u_points:
            u_points_array.append([u_point[0], u_point[1], u_point[2]])
        u_points_array = np.array(u_points_array)
        
        fig.add_trace(go.Scatter3d(
            x=u_points_array[:, 0], 
            y=u_points_array[:, 1], 
            z=u_points_array[:, 2],
            mode='markers+lines',
            marker=dict(size=7, color='red'),
            line=dict(color='red', width=4),
            name='2. U Points'
        ))
        
        # Step 3: Swarm-optimized waypoints (orange)
        if len(collision_result.waypoints) > 0:
            waypoints_array = []
            for waypoint in collision_result.waypoints:
                waypoints_array.append([waypoint[0], waypoint[1], waypoint[2]])
            waypoints_array = np.array(waypoints_array)
            
            fig.add_trace(go.Scatter3d(
                x=waypoints_array[:, 0], 
                y=waypoints_array[:, 1], 
                z=waypoints_array[:, 2],
                mode='markers+lines',
                marker=dict(size=8, color='orange'),
                line=dict(color='orange', width=5),
                name='3. Swarm Waypoints'
            ))
        
        # Step 4: Converted joint positions (purple) 
        if conversion_result.conversion_successful:
            converted_joints = []
            for joint_pos in conversion_result.joint_positions:
                converted_joints.append([joint_pos[0], joint_pos[1], joint_pos[2]])
            converted_joints = np.array(converted_joints)
            
            fig.add_trace(go.Scatter3d(
                x=converted_joints[:, 0], 
                y=converted_joints[:, 1], 
                z=converted_joints[:, 2],
                mode='markers+lines',
                marker=dict(size=6, color='purple'),
                line=dict(color='purple', width=4, dash='dot'),
                name='4. Converted Joints'
            ))
        
        # Step 5: Final FABRIK solution (green)
        final_joints = []
        for joint in final_fabrik.final_chain.joints:
            pos = joint.position
            final_joints.append([pos[0], pos[1], pos[2]])
        final_joints = np.array(final_joints)
        
        fig.add_trace(go.Scatter3d(
            x=final_joints[:, 0], 
            y=final_joints[:, 1], 
            z=final_joints[:, 2],
            mode='markers+lines',
            marker=dict(size=7, color='green'),
            line=dict(color='green', width=6),
            name='5. Final FABRIK'
        ))
        
        # Add target (gold diamond)
        fig.add_trace(go.Scatter3d(
            x=[target[0]], y=[target[1]], z=[target[2]],
            mode='markers',
            marker=dict(size=15, color='gold', symbol='diamond'),
            name='Target'
        ))
        
        # Update layout
        status = "SUCCESS" if not collision_result.has_collision else "OPTIMIZED"
        fig.update_layout(
            title=f'Complete Collision-Aware Pipeline - {status}<br>Steps: FABRIK → U Points → Swarm → Convert → Final FABRIK<br>Time: {collision_result.computation_time:.1f}ms',
            scene=dict(
                xaxis_title='X (mm)',
                yaxis_title='Y (mm)',
                zaxis_title='Z (mm)',
                aspectmode='data',
                camera=dict(
                    eye=dict(x=1.2, y=1.2, z=1.2)
                )
            ),
            height=800,
            legend=dict(
                x=0.02,
                y=0.98,
                bgcolor='rgba(255,255,255,0.8)'
            )
        )
        
        fig.show()
        
        # Print detailed analysis
        print(f"\nPipeline Analysis:")
        print(f"Initial FABRIK error: {initial_fabrik.final_error:.3f}mm")
        print(f"Final FABRIK error: {final_fabrik.final_error:.3f}mm")
        print(f"Collision detected: {collision_result.has_collision}")
        print(f"Min distance to obstacles: {collision_result.min_distance:.2f}mm")
        print(f"Swarm optimization time: {collision_result.computation_time:.2f}ms")
        print(f"Waypoint conversion successful: {conversion_result.conversion_successful}")
        if conversion_result.conversion_successful:
            print(f"Total reach after conversion: {conversion_result.total_reach:.1f}mm")
        
        return True
        
    except Exception as e:
        print(f"Pipeline visualization failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Main test function"""
    print("=== COLLISION-AWARE SOLVER TEST (FIXED VERSION) ===\n")
    
    try:
        import delta_robot
        
        # Step 1: Test basic pipeline components
        if not test_basic_collision_pipeline():
            print("❌ Basic pipeline test failed!")
            return False
        
        print("Basic pipeline test passed!")
        
        # Step 2: Test collision-aware solver
        if not test_collision_aware_solver_simple():
            print("Collision-aware solver test failed!")
            return False
        
        print("Collision-aware solver test passed!")
        
        # Step 3: Create detailed pipeline visualization
        if not create_detailed_pipeline_visualization():
            print("Pipeline visualization failed!")
            return False
        
        print("Pipeline visualization created!")
        
        print("\n=== ALL TESTS PASSED ===")
        print("The collision-aware solver is working correctly!")
        
        return True
        
    except ImportError as e:
        print(f"Import error: {e}")
        print("Please ensure the delta_robot module is properly compiled and installed.")
        return False
    except Exception as e:
        print(f"Test error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)