#!/usr/bin/env python3
"""
Motor Module Position Verification Test - Check if we're using correct positions
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
    
    print("=" * 70)
    
    try:
        import delta_robot
        result = delta_robot.calculate_motors(target_x, target_y, target_z)
        
        if result is None:
            print("Motor module did not return a result.")
            return

        print("POSITION VERIFICATION ANALYSIS")
        print("=" * 70)
        
        print("\nORIGINAL SEGMENT POSITIONS (from FABRIK):")
        for i, (seg_num, pos) in enumerate(zip(result.original_segment_numbers, result.original_segment_positions)):
            print(f"  Segment {seg_num}: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
        
        print(f"\nANALYZING {len(result.levels)} LEVELS:")
        print("=" * 70)
        
        for level_idx, level_data in enumerate(result.levels):
            base_original_seg_num = result.original_segment_numbers[level_idx]
            level_primes = "'" * level_idx
            
            print(f"\n--- LEVEL {level_idx} ---")
            print(f"Analyzing Segment {base_original_seg_num}{level_primes}")
            
            # Show what position was used for kinematics/orientation analysis
            analyzed_pos = level_data.base_segment_position
            print(f"Position sent to Kinematics/Orientation: ({analyzed_pos.x:.3f}, {analyzed_pos.y:.3f}, {analyzed_pos.z:.3f})")
            
            # Compare with original position
            original_pos = result.original_segment_positions[level_idx]
            print(f"Original position of Segment {base_original_seg_num}:     ({original_pos.x:.3f}, {original_pos.y:.3f}, {original_pos.z:.3f})")
            
            # Check if they match (for level 0 they should, for others they shouldn't)
            pos_diff = ((analyzed_pos.x - original_pos.x)**2 + 
                       (analyzed_pos.y - original_pos.y)**2 + 
                       (analyzed_pos.z - original_pos.z)**2)**0.5
            
            if level_idx == 0:
                if pos_diff < 0.001:
                    print("✅ CORRECT: Level 0 using original position")
                else:
                    print(f"❌ ERROR: Level 0 should use original position! Diff: {pos_diff:.6f}")
            else:
                if pos_diff > 0.001:
                    print(f"✅ CORRECT: Level {level_idx} using transformed position (diff: {pos_diff:.3f})")
                else:
                    print(f"❌ ERROR: Level {level_idx} should use transformed position, but matches original!")
            
            # Show what position this would be in the transformed coordinate system
            if level_idx > 0:
                # Find this segment in the previous level's transformed segments
                prev_level = result.levels[level_idx - 1]
                found_in_prev = False
                for j, (transformed_seg_num, transformed_pos) in enumerate(zip(
                    prev_level.transformed_segment_original_numbers, 
                    prev_level.transformed_segment_positions)):
                    if transformed_seg_num == base_original_seg_num:
                        print(f"Expected position from Level {level_idx-1} transform: ({transformed_pos.x:.3f}, {transformed_pos.y:.3f}, {transformed_pos.z:.3f})")
                        
                        # Check if they match
                        expected_diff = ((analyzed_pos.x - transformed_pos.x)**2 + 
                                       (analyzed_pos.y - transformed_pos.y)**2 + 
                                       (analyzed_pos.z - transformed_pos.z)**2)**0.5
                        
                        if expected_diff < 0.001:
                            print("✅ MATCHES: Using correct transformed position from previous level")
                        else:
                            print(f"❌ MISMATCH: Expected vs analyzed diff: {expected_diff:.6f}")
                        found_in_prev = True
                        break
                
                if not found_in_prev:
                    print("❓ Could not find this segment in previous level's transformed segments")
            
            print(f"Kinematics results: z_A={level_data.z_A:.3f}, z_B={level_data.z_B:.3f}, z_C={level_data.z_C:.3f}")
            print(f"UVW U-axis: ({level_data.uvw_u_axis.x:.3f}, {level_data.uvw_u_axis.y:.3f}, {level_data.uvw_u_axis.z:.3f})")
            
            # Check for problematic axis orientations
            warnings = []
            if level_data.uvw_u_axis.x < 0:
                warnings.append("U-axis pointing in -X direction")
            if level_data.uvw_v_axis.y < 0:
                warnings.append("V-axis pointing in -Y direction")  
            if level_data.uvw_w_axis.z < 0:
                warnings.append("W-axis pointing in -Z direction")
            
            if warnings:
                print("⚠️  AXIS WARNINGS:")
                for warning in warnings:
                    print(f"   - {warning}")
            else:
                print("✅ All UVW axes pointing in positive XYZ directions")
        
        print(f"\n" + "=" * 70)
        print("SUMMARY:")
        print("- Level 0 should use original position ✓")
        print("- Level 1+ should use transformed positions from previous level")
        print("- UVW axes should point in positive XYZ directions for proper alignment")
        print("- Any axis warnings above indicate potential flipping issues")
        
    except ImportError as e:
        print(f"ImportError: {e}")
        print("Please ensure the compiled C++ module is in the Python path.")
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()