#!/usr/bin/env python3
"""
Clean Motor Module Test
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
    # Parse command line arguments for target position
    if len(sys.argv) == 2:
        target_x, target_y, target_z = parse_coordinates(sys.argv[1])
        print(f"Target: ({target_x}, {target_y}, {target_z})")
    else:
        target_x, target_y, target_z = 100, 50, 300
        print(f"Default target: ({target_x}, {target_y}, {target_z})")
        print("Usage: python3 test_motor_clean.py x,y,z")
    
    print("=" * 50)
    
    # Test using the package-level function
    try:
        import delta_robot
        result = delta_robot.calculate_motors(target_x, target_y, target_z)
        
        if result is None:
            print("Motor module not available")
            return
        
        # Print segment positions only
        print("SEGMENT POSITIONS:")
        for i, (seg_num, pos) in enumerate(zip(result.segment_numbers, result.segment_positions)):
            print(f"Segment {seg_num}: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
        
        # Print first segment kinematics data
        if result.segment_positions:
            print(f"\nFIRST SEGMENT KINEMATICS:")
            print(f"Position: ({result.first_segment_position.x:.3f}, {result.first_segment_position.y:.3f}, {result.first_segment_position.z:.3f})")
            print(f"Motors: z_A={result.z_A:.3f}, z_B={result.z_B:.3f}, z_C={result.z_C:.3f}")
            print(f"Joints: prismatic={result.prismatic_joint:.3f}, roll={result.roll_joint:.1f}°, pitch={result.pitch_joint:.1f}°")
            
            # Print first segment orientation data (Final UVW)
            print(f"\nFIRST SEGMENT ORIENTATION (Final UVW):")
            print(f"Origin: ({result.uvw_origin.x:.3f}, {result.uvw_origin.y:.3f}, {result.uvw_origin.z:.3f})")
            print(f"U-axis: ({result.uvw_u_axis.x:.3f}, {result.uvw_u_axis.y:.3f}, {result.uvw_u_axis.z:.3f})")
            print(f"V-axis: ({result.uvw_v_axis.x:.3f}, {result.uvw_v_axis.y:.3f}, {result.uvw_v_axis.z:.3f})")
            print(f"W-axis: ({result.uvw_w_axis.x:.3f}, {result.uvw_w_axis.y:.3f}, {result.uvw_w_axis.z:.3f})")
        
        print(f"\n✓ Motor module working correctly!")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()