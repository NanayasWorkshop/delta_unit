#!/usr/bin/env python3
"""
Comprehensive 3D Visualization Test for Delta Robot Modules
Shows all 4 modules working together: Fermat -> Joint State -> Kinematics -> Orientation
"""

import sys
import os
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def parse_coordinates(coord_str):
    """Parse coordinate string like '5,4,7' into x,y,z values"""
    try:
        coords = [float(x.strip()) for x in coord_str.split(',')]
        if len(coords) != 3:
            raise ValueError("Expected 3 coordinates")
        return coords
    except ValueError as e:
        print(f"Error parsing coordinates '{coord_str}': {e}")
        print("Expected format: x,y,z (e.g., 5,4,7)")
        sys.exit(1)

class DeltaVisualization:
    """3D Visualization of complete delta robot calculation pipeline"""
    
    def __init__(self):
        # Import all modules
        try:
            import fermat_module as fm
            import joint_state_module as jsm
            import kinematics_module as km
            import orientation_module as om
            
            self.fermat = fm
            self.joint_state = jsm
            self.kinematics = km
            self.orientation = om
            
            print("✓ All modules imported successfully")
        except ImportError as e:
            print(f"✗ Error importing modules: {e}")
            print("Build all modules first with: python3 setup.py build_ext --inplace")
            sys.exit(1)
        
        # Robot constants (from your constants.hpp)
        self.ROBOT_RADIUS = 24.8
        self.MIN_HEIGHT = 101.0
        self.WORKING_HEIGHT = 11.5
        self.MOTOR_LIMIT = 11.0
        
        # Base positions
        self.base_A = self.fermat.FermatModule.get_base_A()
        self.base_B = self.fermat.FermatModule.get_base_B()
        self.base_C = self.fermat.FermatModule.get_base_C()
        
    def calculate_all_modules(self, input_vector):
        """Calculate results from all 4 modules"""
        print(f"Calculating all modules for input: {input_vector}")
        
        # Step 1: Fermat calculation
        fermat_result = self.fermat.FermatModule.calculate(*input_vector)
        print(f"✓ Fermat: z_A={fermat_result.z_A:.2f}, z_B={fermat_result.z_B:.2f}, z_C={fermat_result.z_C:.2f}")
        
        # Step 2: Joint state calculation  
        direction_vec = self.fermat.Vector3(*input_vector)
        joint_result = self.joint_state.JointStateModule.calculate_from_fermat(direction_vec, fermat_result)
        print(f"✓ Joint State: Prismatic={joint_result.prismatic_joint:.2f}, Roll={math.degrees(joint_result.roll_joint):.1f}°, Pitch={math.degrees(joint_result.pitch_joint):.1f}°")
        
        # Step 3: Kinematics calculation
        kinematics_result = self.kinematics.KinematicsModule.calculate(*input_vector)
        print(f"✓ Kinematics: End-effector=({kinematics_result.end_effector_position.x:.1f}, {kinematics_result.end_effector_position.y:.1f}, {kinematics_result.end_effector_position.z:.1f})")
        
        # Step 4: Orientation calculation
        orientation_result = self.orientation.OrientationModule.calculate(*input_vector)
        print(f"✓ Orientation: Transform matrix created")
        
        return {
            'input': input_vector,
            'fermat': fermat_result,
            'joint_state': joint_result,
            'kinematics': kinematics_result,
            'orientation': orientation_result
        }
    
    def create_comprehensive_visualization(self, input_vector, save_path=None):
        """Create comprehensive 3D visualization with 4 subplots"""
        
        # Calculate all module results
        results = self.calculate_all_modules(input_vector)
        
        # Create figure with 2x2 subplot layout
        fig = plt.figure(figsize=(20, 16))
        fig.suptitle(f'Delta Robot Complete Pipeline - Input Vector: ({input_vector[0]}, {input_vector[1]}, {input_vector[2]})', 
                     fontsize=16, fontweight='bold')
        
        # Subplot 1: Fermat Calculation (top left)
        ax1 = fig.add_subplot(2, 2, 1, projection='3d')
        self.plot_fermat_calculation(ax1, results)
        
        # Subplot 2: Kinematics & End-Effector (top right)
        ax2 = fig.add_subplot(2, 2, 2, projection='3d')
        self.plot_kinematics_calculation(ax2, results)
        
        # Subplot 3: Complete Robot Geometry (bottom left)
        ax3 = fig.add_subplot(2, 2, 3, projection='3d')
        self.plot_complete_robot(ax3, results)
        
        # Subplot 4: Coordinate Systems & Transformation (bottom right)
        ax4 = fig.add_subplot(2, 2, 4, projection='3d')
        self.plot_coordinate_systems(ax4, results)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Visualization saved to {save_path}")
        
        plt.show()
        return results
    
    def plot_fermat_calculation(self, ax, results):
        """Plot Step 1: Fermat calculation with triangle ABC"""
        fermat_result = results['fermat']
        
        # Create 3D points A, B, C with calculated Z values
        A_point = np.array([self.base_A.x, self.base_A.y, fermat_result.z_A])
        B_point = np.array([self.base_B.x, self.base_B.y, fermat_result.z_B])
        C_point = np.array([self.base_C.x, self.base_C.y, fermat_result.z_C])
        fermat_point = np.array([fermat_result.fermat_point.x, fermat_result.fermat_point.y, fermat_result.fermat_point.z])
        
        # Plot base positions (XY plane)
        ax.scatter(self.base_A.x, self.base_A.y, 0, color='red', s=100, marker='o', label='Base A')
        ax.scatter(self.base_B.x, self.base_B.y, 0, color='green', s=100, marker='o', label='Base B')
        ax.scatter(self.base_C.x, self.base_C.y, 0, color='blue', s=100, marker='o', label='Base C')
        
        # Plot 3D triangle points
        ax.scatter(*A_point, color='red', s=150, marker='^', label=f'A (z={fermat_result.z_A:.1f})')
        ax.scatter(*B_point, color='green', s=150, marker='^', label=f'B (z={fermat_result.z_B:.1f})')
        ax.scatter(*C_point, color='blue', s=150, marker='^', label=f'C (z={fermat_result.z_C:.1f})')
        
        # Plot Fermat point
        ax.scatter(*fermat_point, color='magenta', s=200, marker='*', label='Fermat Point')
        
        # Plot triangle edges
        triangle_points = np.array([A_point, B_point, C_point, A_point])
        ax.plot(triangle_points[:, 0], triangle_points[:, 1], triangle_points[:, 2], 'k-', linewidth=2, alpha=0.7)
        
        # Plot lines from bases to 3D points
        for base, point3d, color in [(self.base_A, A_point, 'red'), (self.base_B, B_point, 'green'), (self.base_C, C_point, 'blue')]:
            ax.plot([base.x, point3d[0]], [base.y, point3d[1]], [0, point3d[2]], 
                   color=color, linestyle='--', alpha=0.5)
        
        # Plot lines from Fermat to triangle vertices
        for point, color in [(A_point, 'red'), (B_point, 'green'), (C_point, 'blue')]:
            ax.plot([fermat_point[0], point[0]], [fermat_point[1], point[1]], [fermat_point[2], point[2]], 
                   color=color, linestyle=':', linewidth=2, alpha=0.8)
        
        # Plot direction vector
        direction = np.array(results['input'])
        direction_normalized = direction / np.linalg.norm(direction) * 20
        ax.quiver(0, 0, 0, direction_normalized[0], direction_normalized[1], direction_normalized[2], 
                 color='orange', arrow_length_ratio=0.1, linewidth=3, label='Direction Vector')
        
        ax.set_title('Step 1: Fermat Point Calculation\n(Triangle ABC on inclined plane)')
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    def plot_kinematics_calculation(self, ax, results):
        """Plot Step 3: Kinematics calculation showing end-effector path"""
        kinematics_result = results['kinematics']
        joint_result = results['joint_state']
        
        # Key points
        joint_H = np.array([0, 0, self.WORKING_HEIGHT])
        end_effector = np.array([kinematics_result.end_effector_position.x, 
                                kinematics_result.end_effector_position.y, 
                                kinematics_result.end_effector_position.z])
        
        # Calculate Point G
        vector_length = self.MIN_HEIGHT + 2 * self.MOTOR_LIMIT + joint_result.prismatic_joint
        original_input = np.array(results['input'])
        normalized_direction = original_input / np.linalg.norm(original_input)
        point_G = joint_H + normalized_direction * vector_length
        
        # Plot key points
        ax.scatter(0, 0, 0, color='black', s=100, marker='o', label='Origin')
        ax.scatter(*joint_H, color='purple', s=150, marker='s', label=f'Joint H (z={self.WORKING_HEIGHT})')
        ax.scatter(*point_G, color='orange', s=150, marker='^', label='Point G')
        ax.scatter(*end_effector, color='red', s=200, marker='*', label='End-Effector')
        
        # Plot prismatic joint extension
        ax.plot([joint_H[0], point_G[0]], [joint_H[1], point_G[1]], [joint_H[2], point_G[2]], 
               'purple', linewidth=4, label=f'Prismatic Joint ({joint_result.prismatic_joint:.1f}mm)')
        
        # Plot mirror line (midpoint between H and G)
        midpoint = (joint_H + point_G) / 2
        ax.scatter(*midpoint, color='cyan', s=100, marker='o', label='Mirror Plane Center')
        
        # Plot mirror plane normal (direction vector)
        ax.quiver(midpoint[0], midpoint[1], midpoint[2], 
                 normalized_direction[0]*10, normalized_direction[1]*10, normalized_direction[2]*10,
                 color='cyan', arrow_length_ratio=0.1, linewidth=2, label='Mirror Normal')
        
        # Plot mirror reflection
        ax.plot([0, midpoint[0]], [0, midpoint[1]], [0, midpoint[2]], 'gray', linestyle='--', alpha=0.5, label='Base to Mirror')
        ax.plot([midpoint[0], end_effector[0]], [midpoint[1], end_effector[1]], [midpoint[2], end_effector[2]], 
               'red', linestyle='--', alpha=0.8, label='Mirror to End-Effector')
        
        # Show transformed vector
        transformed = np.array([kinematics_result.transformed_vector.x, 
                               kinematics_result.transformed_vector.y, 
                               kinematics_result.transformed_vector.z])
        ax.quiver(0, 0, 0, transformed[0]*15, transformed[1]*15, transformed[2]*15,
                 color='green', arrow_length_ratio=0.1, linewidth=2, label='Half-Angle Vector')
        
        ax.set_title(f'Step 3: Kinematics Calculation\n(Half-angle: {math.degrees(kinematics_result.input_angle_from_z/2):.1f}°)')
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    def plot_complete_robot(self, ax, results):
        """Plot complete robot geometry with all components"""
        
        # Robot base circle
        theta = np.linspace(0, 2*np.pi, 100)
        base_x = self.ROBOT_RADIUS * np.cos(theta)
        base_y = self.ROBOT_RADIUS * np.sin(theta)
        base_z = np.zeros_like(theta)
        ax.plot(base_x, base_y, base_z, 'k-', linewidth=2, alpha=0.5, label='Robot Base')
        
        # All key points
        fermat_point = np.array([results['fermat'].fermat_point.x, results['fermat'].fermat_point.y, results['fermat'].fermat_point.z])
        joint_H = np.array([0, 0, self.WORKING_HEIGHT])
        end_effector = np.array([results['kinematics'].end_effector_position.x, 
                                results['kinematics'].end_effector_position.y, 
                                results['kinematics'].end_effector_position.z])
        
        # Triangle ABC
        A_point = np.array([self.base_A.x, self.base_A.y, results['fermat'].z_A])
        B_point = np.array([self.base_B.x, self.base_B.y, results['fermat'].z_B])
        C_point = np.array([self.base_C.x, self.base_C.y, results['fermat'].z_C])
        
        # Plot all points
        ax.scatter(0, 0, 0, color='black', s=100, marker='o', label='Origin')
        ax.scatter(*joint_H, color='purple', s=100, marker='s', label='Joint H')
        ax.scatter(*fermat_point, color='magenta', s=150, marker='*', label='Fermat Point')
        ax.scatter(*end_effector, color='red', s=200, marker='*', label='End-Effector')
        
        # Triangle
        ax.scatter(*A_point, color='red', s=100, marker='^')
        ax.scatter(*B_point, color='green', s=100, marker='^')
        ax.scatter(*C_point, color='blue', s=100, marker='^')
        triangle_points = np.array([A_point, B_point, C_point, A_point])
        ax.plot(triangle_points[:, 0], triangle_points[:, 1], triangle_points[:, 2], 'k-', linewidth=2, alpha=0.7)
        
        # Main kinematic chain
        ax.plot([0, joint_H[0]], [0, joint_H[1]], [0, joint_H[2]], 'purple', linewidth=3, label='Fixed Link')
        
        # Calculate Point G for complete chain
        vector_length = self.MIN_HEIGHT + 2 * self.MOTOR_LIMIT + results['joint_state'].prismatic_joint
        original_input = np.array(results['input'])
        normalized_direction = original_input / np.linalg.norm(original_input)
        point_G = joint_H + normalized_direction * vector_length
        
        ax.plot([joint_H[0], point_G[0]], [joint_H[1], point_G[1]], [joint_H[2], point_G[2]], 
               'orange', linewidth=3, label='Prismatic Joint')
        ax.scatter(*point_G, color='orange', s=100, marker='^')
        
        # Show workspace cone
        cone_height = 50
        cone_radius = cone_height * math.tan(self.ROBOT_RADIUS / 180 * math.pi)  # Approximate
        phi = np.linspace(0, 2*np.pi, 20)
        cone_x = cone_radius * np.cos(phi)
        cone_y = cone_radius * np.sin(phi)
        cone_z = np.full_like(phi, cone_height)
        
        for i in range(len(phi)):
            ax.plot([0, cone_x[i]], [0, cone_y[i]], [0, cone_z[i]], 'gray', alpha=0.2, linewidth=0.5)
        
        ax.set_title('Complete Robot Geometry\n(All components integrated)')
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.legend()
    
    def plot_coordinate_systems(self, ax, results):
        """Plot Step 4: Coordinate systems and orientation"""
        orientation_result = results['orientation']
        
        # End-effector position
        end_eff_pos = np.array([orientation_result.end_effector_position.x,
                               orientation_result.end_effector_position.y,
                               orientation_result.end_effector_position.z])
        
        # UVW frame at Fermat point
        fermat_frame = orientation_result.UVW_at_fermat
        fermat_origin = np.array([fermat_frame.origin.x, fermat_frame.origin.y, fermat_frame.origin.z])
        fermat_u = np.array([fermat_frame.u_axis.x, fermat_frame.u_axis.y, fermat_frame.u_axis.z])
        fermat_v = np.array([fermat_frame.v_axis.x, fermat_frame.v_axis.y, fermat_frame.v_axis.z])
        fermat_w = np.array([fermat_frame.w_axis.x, fermat_frame.w_axis.y, fermat_frame.w_axis.z])
        
        # Final frame at end-effector
        final_frame = orientation_result.final_frame
        final_u = np.array([final_frame.u_axis.x, final_frame.u_axis.y, final_frame.u_axis.z])
        final_v = np.array([final_frame.v_axis.x, final_frame.v_axis.y, final_frame.v_axis.z])
        final_w = np.array([final_frame.w_axis.x, final_frame.w_axis.y, final_frame.w_axis.z])
        
        # Plot coordinate systems
        axis_length = 15
        
        # World coordinate system (XYZ)
        ax.quiver(0, 0, 0, axis_length, 0, 0, color='red', arrow_length_ratio=0.1, linewidth=2, label='World X')
        ax.quiver(0, 0, 0, 0, axis_length, 0, color='green', arrow_length_ratio=0.1, linewidth=2, label='World Y')
        ax.quiver(0, 0, 0, 0, 0, axis_length, color='blue', arrow_length_ratio=0.1, linewidth=2, label='World Z')
        
        # UVW frame at Fermat
        ax.scatter(*fermat_origin, color='magenta', s=150, marker='*', label='Fermat Point')
        ax.quiver(fermat_origin[0], fermat_origin[1], fermat_origin[2], 
                 fermat_u[0]*axis_length, fermat_u[1]*axis_length, fermat_u[2]*axis_length,
                 color='red', arrow_length_ratio=0.1, linewidth=2, alpha=0.7, linestyle='--')
        ax.quiver(fermat_origin[0], fermat_origin[1], fermat_origin[2], 
                 fermat_v[0]*axis_length, fermat_v[1]*axis_length, fermat_v[2]*axis_length,
                 color='green', arrow_length_ratio=0.1, linewidth=2, alpha=0.7, linestyle='--')
        ax.quiver(fermat_origin[0], fermat_origin[1], fermat_origin[2], 
                 fermat_w[0]*axis_length, fermat_w[1]*axis_length, fermat_w[2]*axis_length,
                 color='blue', arrow_length_ratio=0.1, linewidth=2, alpha=0.7, linestyle='--')
        
        # Final UVW frame at end-effector
        ax.scatter(*end_eff_pos, color='red', s=200, marker='*', label='End-Effector')
        ax.quiver(end_eff_pos[0], end_eff_pos[1], end_eff_pos[2], 
                 final_u[0]*axis_length, final_u[1]*axis_length, final_u[2]*axis_length,
                 color='red', arrow_length_ratio=0.1, linewidth=3, label="End-Eff U''")
        ax.quiver(end_eff_pos[0], end_eff_pos[1], end_eff_pos[2], 
                 final_v[0]*axis_length, final_v[1]*axis_length, final_v[2]*axis_length,
                 color='green', arrow_length_ratio=0.1, linewidth=3, label="End-Eff V''")
        ax.quiver(end_eff_pos[0], end_eff_pos[1], end_eff_pos[2], 
                 final_w[0]*axis_length, final_w[1]*axis_length, final_w[2]*axis_length,
                 color='blue', arrow_length_ratio=0.1, linewidth=3, label="End-Eff W''")
        
        # Show transformation matrix info
        matrix = orientation_result.transformation_matrix
        ax.text2D(0.02, 0.98, f'Transformation Matrix:\n[{matrix[0,0]:.2f} {matrix[0,1]:.2f} {matrix[0,2]:.2f} {matrix[0,3]:.1f}]\n[{matrix[1,0]:.2f} {matrix[1,1]:.2f} {matrix[1,2]:.2f} {matrix[1,3]:.1f}]\n[{matrix[2,0]:.2f} {matrix[2,1]:.2f} {matrix[2,2]:.2f} {matrix[2,3]:.1f}]\n[{matrix[3,0]:.2f} {matrix[3,1]:.2f} {matrix[3,2]:.2f} {matrix[3,3]:.2f}]', 
                 transform=ax.transAxes, fontsize=8, verticalalignment='top',
                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        ax.set_title('Step 4: Coordinate Systems & Orientation\n(UVW frames and transformation matrix)')
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')

def main():
    # Parse command line arguments
    if len(sys.argv) == 2:
        input_x, input_y, input_z = parse_coordinates(sys.argv[1])
        print(f"Using command line input: ({input_x}, {input_y}, {input_z})")
    else:
        input_x, input_y, input_z = 5, 4, 7
        print("Using default input: (5, 4, 7)")
        print("Usage: python3 test_delta_visual.py x,y,z")
    
    # Create visualizer
    print("\n" + "="*80)
    print("DELTA ROBOT COMPLETE PIPELINE VISUALIZATION")
    print("="*80)
    
    viz = DeltaVisualization()
    
    # Create comprehensive visualization
    input_vector = [input_x, input_y, input_z]
    save_path = f"delta_complete_pipeline_{input_x}_{input_y}_{input_z}.png"
    
    print(f"\nGenerating complete visualization for input vector: {input_vector}")
    results = viz.create_comprehensive_visualization(input_vector, save_path)
    
    if results:
        print(f"\n✓ Visualization complete!")
        print(f"✓ All 4 modules calculated successfully")
        print(f"✓ Saved to: {save_path}")
        
        # Print summary
        print(f"\n" + "="*50)
        print("PIPELINE SUMMARY:")
        print(f"Input Vector: {input_vector}")
        print(f"Fermat Point: ({results['fermat'].fermat_point.x:.2f}, {results['fermat'].fermat_point.y:.2f}, {results['fermat'].fermat_point.z:.2f})")
        print(f"Prismatic Joint: {results['joint_state'].prismatic_joint:.2f} mm")
        print(f"End-Effector: ({results['kinematics'].end_effector_position.x:.2f}, {results['kinematics'].end_effector_position.y:.2f}, {results['kinematics'].end_effector_position.z:.2f})")
        print(f"Transform Matrix: 4x4 homogeneous transformation created")
        print(f"="*50)
    
    return results

if __name__ == "__main__":
    main()