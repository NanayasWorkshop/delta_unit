#!/usr/bin/env python3
"""
Test U Points Extractor - Visualize J points vs U points
"""
import sys
import os
import numpy as np
import plotly.graph_objects as go

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def test_u_points_extraction():
    try:
        import delta_robot
        
        # Test target
        target = np.array([100, 50, 300])
        print(f"Testing U Points extraction for target: {target}")
        
        # Solve FABRIK to get joint positions
        fabrik_result = delta_robot.fabrik.solve_delta_robot(7, target, delta_robot.FABRIK_TOLERANCE)
        
        if not fabrik_result.converged:
            print("FABRIK did not converge!")
            return
        
        # Extract joint positions from FABRIK result
        joint_positions = []
        for joint in fabrik_result.final_chain.joints:
            joint_positions.append([joint.position[0], joint.position[1], joint.position[2]])
        
        print(f"FABRIK found {len(joint_positions)} joint positions")
        
        # Extract U points using our new extractor
        u_points_raw = delta_robot.delta_robot_complete.UPointsExtractor.extract_u_points_from_positions(
            [np.array(pos) for pos in joint_positions]
        )
        
        # Convert to regular Python lists for plotting
        u_points = []
        for u_point in u_points_raw:
            u_points.append([u_point[0], u_point[1], u_point[2]])
        
        print(f"Extracted {len(u_points)} U points")
        
        # Create visualization
        fig = go.Figure()
        
        # Plot J points (FABRIK joints) in blue
        j_x = [pos[0] for pos in joint_positions]
        j_y = [pos[1] for pos in joint_positions]
        j_z = [pos[2] for pos in joint_positions]
        
        fig.add_trace(go.Scatter3d(
            x=j_x, y=j_y, z=j_z,
            mode='markers+lines',
            marker=dict(size=8, color='blue'),
            line=dict(color='blue', width=4),
            name='J Points (FABRIK Joints)'
        ))
        
        # Plot U points (collision detection points) in red
        u_x = [pos[0] for pos in u_points]
        u_y = [pos[1] for pos in u_points]
        u_z = [pos[2] for pos in u_points]
        
        fig.add_trace(go.Scatter3d(
            x=u_x, y=u_y, z=u_z,
            mode='markers+lines',
            marker=dict(size=6, color='red'),
            line=dict(color='red', width=2),
            name='U Points (Collision Detection)'
        ))
        
        # Plot target point in green
        fig.add_trace(go.Scatter3d(
            x=[target[0]], y=[target[1]], z=[target[2]],
            mode='markers',
            marker=dict(size=12, color='green'),
            name='Target'
        ))
        
        # Set layout
        fig.update_layout(
            title='FABRIK J Points vs U Points Visualization',
            scene=dict(
                xaxis_title='X (mm)',
                yaxis_title='Y (mm)',
                zaxis_title='Z (mm)',
                aspectmode='data'
            )
        )
        
        # Show the plot
        fig.show()
        
        # Print comparison
        print("\nJ Points vs U Points comparison:")
        print(f"J Points: {len(joint_positions)}")
        print(f"U Points: {len(u_points)}")
        
        for i, (j_pos, u_pos) in enumerate(zip(joint_positions, u_points[:len(joint_positions)])):
            distance = np.linalg.norm(np.array(j_pos) - np.array(u_pos))
            print(f"  J{i} vs U{i+1}: distance = {distance:.3f}mm")
        
        return True
        
    except ImportError as e:
        print(f"ImportError: {e}")
        print("Make sure to compile the module first: python setup.py build_ext --inplace")
        return False
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    test_u_points_extraction()