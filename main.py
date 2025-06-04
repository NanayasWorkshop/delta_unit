#!/usr/bin/env python3
"""
Compact Dynamic Motor Module Test
"""
import sys
import os

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

def main():
    if len(sys.argv) == 2:
        target_x, target_y, target_z = parse_coordinates(sys.argv[1])
    else:
        target_x, target_y, target_z = 100, 50, 300
        print(f"Usage: python3 {os.path.basename(__file__)} x,y,z")
    
    print(f"Target: ({target_x}, {target_y}, {target_z})")
    
    try:
        import delta_robot
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
            print(f"  Position: ({level_data.base_segment_position.x:.3f}, {level_data.base_segment_position.y:.3f}, {level_data.base_segment_position.z:.3f})")
            print(f"  Motors: z_A={level_data.z_A:.3f}, z_B={level_data.z_B:.3f}, z_C={level_data.z_C:.3f}")
            print(f"  Joints: prismatic={level_data.prismatic_joint:.3f}, roll={level_data.roll_joint:.1f}°, pitch={level_data.pitch_joint:.1f}°")
        
        # Check if timing information is available
        if hasattr(result, 'solve_time_ms'):
            print(f"\nFABRIK converged: {result.fabrik_converged}, Error: {result.fabrik_error:.4f}, Time: {result.solve_time_ms:.2f}ms")
        else:
            print(f"\nFABRIK converged: {result.fabrik_converged}, Error: {result.fabrik_error:.4f}")
            print("Note: Timing information not available - needs C++ motor module update")
        
    except ImportError as e:
        print(f"ImportError: {e}")
        print("Please ensure the compiled C++ module is in the Python path.")
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()