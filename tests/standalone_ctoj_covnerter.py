#!/usr/bin/env python3
"""
Standalone Joint Calculation Visualizer
Analyzes P points → J points conversion with matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize_scalar

class StandaloneJointConverter:
    """
    Standalone version of joint converter for analysis and visualization
    """
    def __init__(self):
        self.points = []
        self.joints = []
        self.optimization_history = []  # Track optimization steps
        
    def load_points_from_coordinates(self, coordinates):
        """Load P points from coordinate list"""
        self.points = []
        
        for i, (x, y, z) in enumerate(coordinates):
            point_array = np.array([x, y, z])
            self.points.append(point_array)
            print(f"P{i+1}: ({x:.6f}, {y:.6f}, {z:.6f})")
        
        print(f"Loaded {len(self.points)} P points")
        return len(self.points)
    
    def angle_between_vectors(self, v1, v2):
        """Calculate angle between two vectors in degrees"""
        dot_product = np.dot(v1, v2)
        norms = np.linalg.norm(v1) * np.linalg.norm(v2)
        if norms == 0:
            return 0
        angle_rad = np.arccos(np.clip(dot_product / norms, -1, 1))
        return angle_rad * 180 / np.pi
    
    def optimize_first_joint(self, bounds=(-0.5, 0.5), visualize_optimization=False):
        """Optimize J1 position on Z-axis for first triangle (P1-J1-P2)"""
        if len(self.points) < 2:
            raise ValueError("Need at least 2 points to optimize first joint")
            
        p1, p2 = self.points[0], self.points[1]
        
        # Store optimization steps
        z_values = []
        angle_diffs = []
        
        def angle_difference(z):
            j1 = np.array([0, 0, z])
            
            # Vectors for angle calculation
            p1_j1 = j1 - p1
            p1_p2 = p2 - p1
            p2_j1 = j1 - p2
            p2_p1 = p1 - p2
            
            angle1 = self.angle_between_vectors(p1_j1, p1_p2)
            angle2 = self.angle_between_vectors(p2_j1, p2_p1)
            
            diff = abs(angle1 - angle2)
            
            if visualize_optimization:
                z_values.append(z)
                angle_diffs.append(diff)
            
            return diff
        
        result = minimize_scalar(angle_difference, bounds=bounds, method='bounded')
        j1 = np.array([0, 0, result.x])
        self.joints.append(j1)
        
        if visualize_optimization:
            self.optimization_history.append({
                'joint_num': 1,
                'z_values': z_values,
                'angle_diffs': angle_diffs,
                'optimal_z': result.x,
                'optimal_angle_diff': result.fun
            })
        
        return j1
    
    def optimize_joint(self, joint_index, bounds=(-2, 2), visualize_optimization=False):
        """Optimize joint position along the line from previous joint through current pivot point"""
        if joint_index == 0:
            return self.optimize_first_joint(visualize_optimization=visualize_optimization)
            
        if len(self.points) < joint_index + 2:
            raise ValueError(f"Need at least {joint_index + 2} points to optimize joint {joint_index + 1}")
        
        # Get the relevant points
        p_prev = self.points[joint_index]      # Previous point (pivot of current triangle)
        p_curr = self.points[joint_index + 1]  # Current point (base vertex 1)
        j_prev = self.joints[joint_index - 1]  # Previous joint
        
        # Direction from previous joint through previous point
        direction = p_prev - j_prev
        direction_norm = np.linalg.norm(direction)
        if direction_norm == 0:
            print(f"Warning: Zero direction vector for joint {joint_index + 1}")
            # Use default direction
            direction_unit = np.array([0, 0, 1])
        else:
            direction_unit = direction / direction_norm
        
        # Store optimization steps
        t_values = []
        angle_diffs = []
        
        def angle_difference(t):
            # Joint position along the line
            j_curr = j_prev + t * direction_unit
            
            # Vectors from current point
            p_curr_j = j_curr - p_curr
            p_curr_p_prev = p_prev - p_curr
            
            # Vectors from previous point  
            p_prev_j = j_curr - p_prev
            p_prev_p_curr = p_curr - p_prev
            
            angle1 = self.angle_between_vectors(p_curr_j, p_curr_p_prev)
            angle2 = self.angle_between_vectors(p_prev_j, p_prev_p_curr)
            
            diff = abs(angle1 - angle2)
            
            if visualize_optimization:
                t_values.append(t)
                angle_diffs.append(diff)
            
            return diff
        
        result = minimize_scalar(angle_difference, bounds=bounds, method='bounded')
        j_curr = j_prev + result.x * direction_unit
        self.joints.append(j_curr)
        
        if visualize_optimization:
            self.optimization_history.append({
                'joint_num': joint_index + 1,
                't_values': t_values,
                'angle_diffs': angle_diffs,
                'optimal_t': result.x,
                'optimal_angle_diff': result.fun,
                'direction': direction_unit,
                'j_prev': j_prev.copy(),
                'p_prev': p_prev.copy(),
                'p_curr': p_curr.copy()
            })
        
        return j_curr
    
    def calculate_j_points_with_analysis(self, visualize_optimization=True):
        """Calculate J points with detailed analysis"""
        if len(self.points) < 2:
            print("Need at least 2 P points to calculate J points")
            return []
        
        self.joints = []
        self.optimization_history = []
        
        print(f"\n=== Calculating J points from {len(self.points)} P points ===")
        
        # Calculate (n-1) joints for n points using optimization
        for i in range(len(self.points) - 1):
            joint = self.optimize_joint(i, visualize_optimization=visualize_optimization)
            print(f"J{i+1} position: ({joint[0]:.6f}, {joint[1]:.6f}, {joint[2]:.6f})")
        
        # Add final J point equal to last P point (end effector)
        final_j_point = self.points[-1].copy()  # J_n = P_n
        self.joints.append(final_j_point)
        print(f"J{len(self.joints)} position: ({final_j_point[0]:.6f}, {final_j_point[1]:.6f}, {final_j_point[2]:.6f}) [= P{len(self.points)}]")
        
        print(f"\nCalculated {len(self.joints)} J points (including end effector)")
        return self.joints.copy()
    
    def analyze_problems(self):
        """Analyze potential problems in the P point configuration"""
        problems = []
        
        # Check for points too close together
        for i in range(len(self.points) - 1):
            dist = np.linalg.norm(self.points[i+1] - self.points[i])
            if dist < 0.01:  # Less than 1cm
                problems.append(f"P{i+1} and P{i+2} are very close: {dist:.6f}m")
        
        # Check for backtracking (points going backward)
        for i in range(len(self.points) - 2):
            v1 = self.points[i+1] - self.points[i]
            v2 = self.points[i+2] - self.points[i+1]
            dot = np.dot(v1, v2)
            if dot < 0:
                problems.append(f"Backtracking detected between P{i+1}→P{i+2}→P{i+3}")
        
        # Check for extreme coordinate jumps
        for i in range(len(self.points) - 1):
            diff = self.points[i+1] - self.points[i]
            if np.max(np.abs(diff)) > 0.5:  # 50cm jump in any direction
                problems.append(f"Large jump from P{i+1} to P{i+2}: {diff}")
        
        # Check Z-direction consistency
        z_values = [p[2] for p in self.points]
        if not all(z_values[i] <= z_values[i+1] for i in range(len(z_values)-1)):
            problems.append("Z-coordinates are not monotonically increasing")
        
        return problems

def visualize_complete_analysis(p_points_coords):
    """Complete visualization and analysis of P→J conversion"""
    
    # Initialize converter
    converter = StandaloneJointConverter()
    converter.load_points_from_coordinates(p_points_coords)
    
    # Analyze problems
    print("\n=== PROBLEM ANALYSIS ===")
    problems = converter.analyze_problems()
    if problems:
        for problem in problems:
            print(f"⚠️  {problem}")
    else:
        print("✅ No obvious problems detected")
    
    # Calculate J points with optimization tracking
    j_points = converter.calculate_j_points_with_analysis(visualize_optimization=True)
    
    # Create comprehensive visualization
    fig = plt.figure(figsize=(20, 12))
    
    # 1. 3D Plot of P and J points
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    
    # Extract coordinates
    p_coords = np.array(converter.points)
    j_coords = np.array(converter.joints)
    
    # Plot P points (connection points)
    ax1.scatter(p_coords[:, 0], p_coords[:, 1], p_coords[:, 2], 
               c='red', s=100, alpha=0.7, label='P points (Connection)')
    
    # Plot J points (optimized joints)
    ax1.scatter(j_coords[:, 0], j_coords[:, 1], j_coords[:, 2], 
               c='blue', s=100, alpha=0.7, label='J points (Joints)')
    
    # Connect P points with lines
    ax1.plot(p_coords[:, 0], p_coords[:, 1], p_coords[:, 2], 
             'r--', alpha=0.5, linewidth=2, label='P chain')
    
    # Connect J points with lines
    ax1.plot(j_coords[:, 0], j_coords[:, 1], j_coords[:, 2], 
             'b-', alpha=0.7, linewidth=3, label='J chain')
    
    # Add point labels
    for i, (p, j) in enumerate(zip(converter.points, converter.joints)):
        ax1.text(p[0], p[1], p[2], f'P{i+1}', fontsize=8)
        ax1.text(j[0], j[1], j[2], f'J{i+1}', fontsize=8)
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('P Points vs J Points (3D)')
    ax1.legend()
    ax1.grid(True)
    
    # 2. X-Y Projection
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.scatter(p_coords[:, 0], p_coords[:, 1], c='red', s=100, alpha=0.7, label='P points')
    ax2.scatter(j_coords[:, 0], j_coords[:, 1], c='blue', s=100, alpha=0.7, label='J points')
    ax2.plot(p_coords[:, 0], p_coords[:, 1], 'r--', alpha=0.5, linewidth=2)
    ax2.plot(j_coords[:, 0], j_coords[:, 1], 'b-', alpha=0.7, linewidth=3)
    
    for i, (p, j) in enumerate(zip(converter.points, converter.joints)):
        ax2.text(p[0], p[1], f'P{i+1}', fontsize=8)
        ax2.text(j[0], j[1], f'J{i+1}', fontsize=8)
    
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('X-Y Projection')
    ax2.legend()
    ax2.grid(True)
    ax2.axis('equal')
    
    # 3. Z progression
    ax3 = fig.add_subplot(2, 3, 3)
    points_range = range(1, len(converter.points) + 1)
    ax3.plot(points_range, p_coords[:, 2], 'ro-', label='P points Z', linewidth=2, markersize=8)
    ax3.plot(points_range, j_coords[:, 2], 'bo-', label='J points Z', linewidth=2, markersize=8)
    ax3.set_xlabel('Point Number')
    ax3.set_ylabel('Z Coordinate (m)')
    ax3.set_title('Z Progression')
    ax3.legend()
    ax3.grid(True)
    
    # 4. Distances between consecutive points
    ax4 = fig.add_subplot(2, 3, 4)
    p_distances = [np.linalg.norm(p_coords[i+1] - p_coords[i]) for i in range(len(p_coords)-1)]
    j_distances = [np.linalg.norm(j_coords[i+1] - j_coords[i]) for i in range(len(j_coords)-1)]
    
    segment_labels = [f'P{i+1}→P{i+2}' for i in range(len(p_distances))]
    x_pos = range(len(p_distances))
    
    ax4.bar([x - 0.2 for x in x_pos], p_distances, 0.4, label='P point distances', alpha=0.7)
    ax4.bar([x + 0.2 for x in x_pos], j_distances, 0.4, label='J point distances', alpha=0.7)
    ax4.set_xlabel('Segment')
    ax4.set_ylabel('Distance (m)')
    ax4.set_title('Inter-Point Distances')
    ax4.set_xticks(x_pos)
    ax4.set_xticklabels(segment_labels, rotation=45)
    ax4.legend()
    ax4.grid(True)
    
    # 5. Optimization convergence (if available)
    ax5 = fig.add_subplot(2, 3, 5)
    if converter.optimization_history:
        for hist in converter.optimization_history:
            joint_num = hist['joint_num']
            if 'z_values' in hist:  # First joint
                ax5.plot(hist['z_values'], hist['angle_diffs'], 
                        label=f'J{joint_num} (Z-axis)', alpha=0.7)
            elif 't_values' in hist:  # Other joints
                ax5.plot(hist['t_values'], hist['angle_diffs'], 
                        label=f'J{joint_num} (t-param)', alpha=0.7)
    
    ax5.set_xlabel('Parameter Value')
    ax5.set_ylabel('Angle Difference (degrees)')
    ax5.set_title('Optimization Convergence')
    ax5.legend()
    ax5.grid(True)
    
    # 6. Problem summary text
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.axis('off')
    
    summary_text = f"""
ANALYSIS SUMMARY

P Points: {len(converter.points)}
J Points: {len(converter.joints)}

DETECTED PROBLEMS:
"""
    
    if problems:
        for i, problem in enumerate(problems):
            summary_text += f"{i+1}. {problem}\n"
    else:
        summary_text += "✅ No problems detected"
    
    summary_text += f"""

STATISTICS:
Total P chain length: {sum(p_distances):.3f}m
Total J chain length: {sum(j_distances):.3f}m
Max P distance: {max(p_distances):.3f}m
Max J distance: {max(j_distances):.3f}m
Z range: {min(p_coords[:, 2]):.3f} to {max(p_coords[:, 2]):.3f}m
"""
    
    ax6.text(0.05, 0.95, summary_text, transform=ax6.transAxes, 
             fontsize=10, verticalalignment='top', fontfamily='monospace')
    
    plt.tight_layout()
    plt.show()
    
    return converter

# Your actual P points data
if __name__ == "__main__":
    # P points from your robot
    your_p_points = [
        (0.000000, 0.000000, -0.011500),
        (-0.001866, -0.003316, 0.139039),
        (-0.019779, -0.027667, 0.282547),
        (-0.019552, -0.030537, 0.410814),
        (0.043792, 0.040967, 0.491715),
        (0.125436, 0.135485, 0.449693),
        (0.178752, 0.200311, 0.333211),
        (0.215341, 0.251606, 0.217538)
    ]
    
    print("=== ANALYZING YOUR P POINTS ===")
    converter = visualize_complete_analysis(your_p_points)
    
    # Print detailed results
    print("\n=== DETAILED RESULTS ===")
    print("P Points:")
    for i, p in enumerate(converter.points):
        print(f"  P{i+1}: ({p[0]:.6f}, {p[1]:.6f}, {p[2]:.6f})")
    
    print("\nJ Points:")
    for i, j in enumerate(converter.joints):
        print(f"  J{i+1}: ({j[0]:.6f}, {j[1]:.6f}, {j[2]:.6f})")