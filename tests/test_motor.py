#!/usr/bin/env python3
"""
Dynamic Motor Module Test with UVW to XYZ transformation for multiple levels
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
        print(f"Target: ({target_x}, {target_y}, {target_z})")
    else:
        target_x, target_y, target_z = 100, 50, 300
        print(f"Default target: ({target_x}, {target_y}, {target_z})")
        print(f"Usage: python3 {os.path.basename(__file__)} x,y,z")
    
    print("=" * 50)
    
    try:
        import delta_robot
        result = delta_robot.calculate_motors(target_x, target_y, target_z)
        
        if result is None:
            print("Motor module did not return a result.")
            return

        print("INITIAL FABRIK SOLVER RESULT:")
        print(f"Target Position: ({result.target_position.x:.3f}, {result.target_position.y:.3f}, {result.target_position.z:.3f})")
        print(f"FABRIK Converged: {result.fabrik_converged}, Final Error: {result.fabrik_error:.6f}")

        print("\nORIGINAL SEGMENT POSITIONS (Global Frame):")
        if not result.original_segment_positions:
            print("No original segment positions found.")
        for i, (seg_num, pos) in enumerate(zip(result.original_segment_numbers, result.original_segment_positions)):
            print(f"Segment {seg_num}: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
        
        if not result.levels:
            print("\nNo transformation levels processed.")
            return

        for level_idx, level_data in enumerate(result.levels):
            # Determine the original segment number that forms the base of this level
            # Level 0 is based on original_segment_numbers[0] (Segment 1)
            # Level 1 is based on original_segment_numbers[1] (Segment 2)
            base_original_seg_num = result.original_segment_numbers[level_idx]
            
            level_primes = "'" * level_idx # S1, S2', S3'', ...
            
            print(f"\n--- LEVEL {level_idx} ---")
            print(f"BASE SEGMENT FOR THIS LEVEL: Original Segment {base_original_seg_num}{level_primes}")
            print(f"KINEMATICS (for Seg {base_original_seg_num}{level_primes}):")
            print(f"  Position: ({level_data.base_segment_position.x:.3f}, {level_data.base_segment_position.y:.3f}, {level_data.base_segment_position.z:.3f})")
            print(f"  Motors: z_A={level_data.z_A:.3f}, z_B={level_data.z_B:.3f}, z_C={level_data.z_C:.3f}")
            print(f"  Joints: prismatic={level_data.prismatic_joint:.3f}, roll={level_data.roll_joint:.1f}°, pitch={level_data.pitch_joint:.1f}°")
            
            print(f"\nORIENTATION (for Seg {base_original_seg_num}{level_primes}'s UVW frame):")
            print(f"  UVW Origin: ({level_data.uvw_origin.x:.3f}, {level_data.uvw_origin.y:.3f}, {level_data.uvw_origin.z:.3f})")
            print(f"  U-axis: ({level_data.uvw_u_axis.x:.3f}, {level_data.uvw_u_axis.y:.3f}, {level_data.uvw_u_axis.z:.3f})")
            print(f"  V-axis: ({level_data.uvw_v_axis.x:.3f}, {level_data.uvw_v_axis.y:.3f}, {level_data.uvw_v_axis.z:.3f})")
            print(f"  W-axis: ({level_data.uvw_w_axis.x:.3f}, {level_data.uvw_w_axis.y:.3f}, {level_data.uvw_w_axis.z:.3f})")

            if level_data.transformed_segment_positions:
                transformed_primes = "'" * (level_idx + 1) # S2', S3'', S4'''
                print(f"\nTRANSFORMED SEGMENTS (relative to Seg {base_original_seg_num}{level_primes}'s UVW aligned to XYZ):")
                print(f"  (Translated so Seg {base_original_seg_num}{level_primes} is at origin, rotated so its UVW axes align with XYZ)")
                for i, (orig_seg_num, pos) in enumerate(zip(level_data.transformed_segment_original_numbers, level_data.transformed_segment_positions)):
                    print(f"  Segment {orig_seg_num}{transformed_primes}: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
            else:
                if level_idx < len(result.original_segment_numbers) - 1:
                    print(f"\n  No further segments transformed from this level.")
        
        print(f"\n✓ Motor module processing complete for {len(result.levels)} levels.")
        
    except ImportError as e:
        print(f"ImportError: {e}")
        print("Please ensure the compiled C++ module is in the Python path.")
        print("Also ensure dependent types like Vector3 are correctly bound and accessible.")
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()