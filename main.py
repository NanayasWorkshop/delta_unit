#!/usr/bin/env python3
"""
Compact Dynamic Motor Module Test with --current joint position support
"""
import sys
import os
import numpy as np

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def parse_coordinates(coord_str):
    """Parse coordinate string like '100,50,300' into x,y,z values"""
    try:
        coords = [float(x.strip()) for x in coord_str.split(',')]
        if len(coords) != 3:
            raise ValueError("Expected 3 coordinates")
        return coords
    except ValueError as e:
        print(f"Error parsing coordinates '{coord_str}': {e}")
        print("Expected format: x,y,z (e.g., 100,50,300)")
        sys.exit(1)

def parse_current_positions(current_str, expected_segments=8):
    """Parse current joint positions string"""
    try:
        # Parse: "(-20.627,-10.314,142.314),(-8.943,-4.472,145.316),..."
        # OR: "0.000,0.000,0.000:0.000,0.000,73.025:..."
        
        if ':' in current_str:
            # Colon-separated format
            position_strs = current_str.split(':')
        else:
            # Comma-separated format with parentheses (support legacy format)
            current_str = current_str.replace('(', '').replace(')', '')
            position_strs = current_str.split(',')
            # Group every 3 values
            position_strs = [','.join(position_strs[i:i+3]) for i in range(0, len(position_strs), 3)]
        
        positions = []
        for pos_str in position_strs:
            coords = [float(x.strip()) for x in pos_str.split(',')]
            if len(coords) != 3:
                raise ValueError(f"Expected 3 coordinates per position, got {len(coords)}")
            positions.append(np.array([coords[0], coords[1], coords[2]]))
        
        expected_joints = expected_segments + 2  # base + segments + end-effector
        if len(positions) != expected_joints:
            print(f"Warning: Expected {expected_joints} joint positions, got {len(positions)}. Using straight-up fallback.")
            return None
            
        return positions
    except Exception as e:
        print(f"Error parsing current positions '{current_str}': {e}")
        print("Using straight-up fallback.")
        return None

def format_joint_positions(motor_result):
    """Extract joint positions from motor result and format as --current parameter"""
    try:
        joint_positions = []
        for joint_pos in motor_result.fabrik_joint_positions:
            if hasattr(joint_pos, 'shape'):  # numpy array
                joint_positions.append(f"{joint_pos[0]:.3f},{joint_pos[1]:.3f},{joint_pos[2]:.3f}")
            else:
                # Handle Eigen vector case (fallback)
                joint_positions.append(f"{joint_pos.x():.3f},{joint_pos.y():.3f},{joint_pos.z():.3f}")
        
        return ":".join(joint_positions)
    
    except Exception as e:
        print(f"Warning: Could not extract joint positions: {e}")
        return "0,0,0:0,0,0:0,0,0:0,0,0"  # fallback with default number of joints

def main():
    # Parse command line arguments
    if len(sys.argv) < 2:
        target_x, target_y, target_z = 100, 50, 300
        print(f"Usage: python3 {os.path.basename(__file__)} x,y,z [--current \"joint_positions\"]")
    else:
        target_x, target_y, target_z = parse_coordinates(sys.argv[1])
    
    # Parse optional current positions
    current_positions = None
    if len(sys.argv) >= 4 and sys.argv[2] == "--current":
        current_positions = parse_current_positions(sys.argv[3])
    
    print(f"Target: ({target_x}, {target_y}, {target_z})")
    if current_positions is not None:
        print(f"Using provided joint positions ({len(current_positions)} joints)")
    else:
        print("Using straight-up initialization")
    
    try:
        import delta_robot
        
        # Call appropriate method based on whether current positions are provided
        if current_positions is not None:
            result = delta_robot.motor.MotorModule.calculate_motors(target_x, target_y, target_z, current_positions)
        else:
            result = delta_robot.calculate_motors(target_x, target_y, target_z)
        
        if result is None:
            print("Motor module did not return a result.")
            return
        
        if not result.levels:
            print("No transformation levels processed.")
            return
        
        # Print compact segment information
        for level_idx, level_data in enumerate(result.levels):
            base_original_seg_num = result.original_segment_numbers[level_idx]
            
            print(f"--- Segment {base_original_seg_num} ---")
            
            # Handle Eigen vector access - it should now return a numpy array
            pos = level_data.base_segment_position
            if hasattr(pos, 'shape'):  # It's a numpy array
                print(f"  Position: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
            else:  # Fallback if it's still Eigen
                print(f"  Position: Eigen vector (conversion issue)")
            
            print(f"  Motors: z_A={level_data.z_A:.3f}, z_B={level_data.z_B:.3f}, z_C={level_data.z_C:.3f}")
            print(f"  Joints: prismatic={level_data.prismatic_joint:.3f}, roll={level_data.roll_joint:.1f}°, pitch={level_data.pitch_joint:.1f}°")
        
        # Check if timing information is available
        if hasattr(result, 'solve_time_ms'):
            print(f"\nFABRIK converged: {result.fabrik_converged}, Error: {result.fabrik_error:.4f}, Time: {result.solve_time_ms:.2f}ms")
        else:
            print(f"\nFABRIK converged: {result.fabrik_converged}, Error: {result.fabrik_error:.4f}")
            print("Note: Timing information not available - needs C++ motor module update")
        
        # NEW: Add FABRIK joint positions in --current format (efficient - no double solving!)
        joint_positions_str = format_joint_positions(result)
        print(f'--current "{joint_positions_str}"')
        
    except ImportError as e:
        print(f"ImportError: {e}")
        print("Please ensure the compiled C++ module is in the Python path.")
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()