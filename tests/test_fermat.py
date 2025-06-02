#!/usr/bin/env python3
"""
Test the clean Fermat module interface
"""

import sys
import os
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

def test_fermat_module():
    # Parse command line arguments for direction vector
    if len(sys.argv) == 2:
        x, y, z = parse_coordinates(sys.argv[1])
        print(f"Using command line direction: ({x}, {y}, {z})")
    else:
        x, y, z = 5, 4, 7
        print("Using default direction: (5, 4, 7)")
        print("Usage: python3 test_fermat.py x,y,z")
    
    try:
        import fermat_module as fm
    except ImportError:
        print("Error: fermat_module not found. Build first.")
        return False
    
    print("Testing Fermat Module")
    print("=" * 30)
    
    # Test with coordinates
    result = fm.FermatModule.calculate(x, y, z)
    
    print(f"Input: ({x}, {y}, {z})")
    print(f"Z positions:")
    print(f"  A: {result.z_A:.4f}")
    print(f"  B: {result.z_B:.4f}")
    print(f"  C: {result.z_C:.4f}")
    print(f"Fermat point: ({result.fermat_point.x:.4f}, {result.fermat_point.y:.4f}, {result.fermat_point.z:.4f})")
    
    print("\nBase positions:")
    base_A = fm.FermatModule.get_base_A()
    base_B = fm.FermatModule.get_base_B()
    base_C = fm.FermatModule.get_base_C()
    print(f"  A: {base_A}")
    print(f"  B: {base_B}")
    print(f"  C: {base_C}")
    
    print("\nModule working correctly!")
    return True

if __name__ == "__main__":
    test_fermat_module()