#!/usr/bin/env python3
"""
Delta robot Fermat point calculation
Usage: python3 calculate.py x,y,z
Example: python3 calculate.py 5,4,7
"""

import sys
import math

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

def print_vector(name, vec, indent=""):
    """Print vector in formatted way"""
    print(f"{indent}{name}: ({vec.x:.4f}, {vec.y:.4f}, {vec.z:.4f})")

def print_section(title):
    """Print section header"""
    print(f"\n{title}")
    print("-" * len(title))

def main():
    print("Starting calculate.py...")
    print(f"Arguments: {sys.argv}")
    
    if len(sys.argv) != 2:
        print("Usage: python3 calculate.py x,y,z")
        print("Example: python3 calculate.py 5,4,7")
        sys.exit(1)
    
    # Parse input coordinates
    print(f"Parsing coordinates: {sys.argv[1]}")
    x, y, z = parse_coordinates(sys.argv[1])
    print(f"Parsed: x={x}, y={y}, z={z}")
    
    try:
        print("Importing delta_math module...")
        import delta_math as dm
        print("Module imported successfully")
    except ImportError:
        print("Error: delta_math module not found")
        print("Build the module first with: python3 setup.py build_ext --inplace")
        sys.exit(1)
    
    print("Delta Robot Fermat Point Calculation")
    print("=" * 40)
    
    print_section("Input")
    print(f"Direction vector: ({x}, {y}, {z})")
    
    # Create direction vector and normalize
    direction = dm.Vector3(x, y, z)
    normalized = direction.normalized()
    print(f"Normalized:       ({normalized.x:.4f}, {normalized.y:.4f}, {normalized.z:.4f})")
    print(f"Magnitude:        {direction.norm():.4f}")
    
    print_section("Base Positions")
    base_A = dm.get_base_position_A()
    base_B = dm.get_base_position_B()
    base_C = dm.get_base_position_C()
    
    print_vector("Base A (XY)", base_A, "  ")
    print_vector("Base B (XY)", base_B, "  ")
    print_vector("Base C (XY)", base_C, "  ")
    
    print_section("Fermat Calculation")
    calc = dm.FermatCalculation(direction)
    
    print("3D Points after Z intersection:")
    print_vector("Point A", calc.A_point, "  ")
    print_vector("Point B", calc.B_point, "  ")
    print_vector("Point C", calc.C_point, "  ")
    
    print(f"\nSide lengths:")
    print(f"  a (BC): {calc.side_a:.4f}")
    print(f"  b (CA): {calc.side_b:.4f}")
    print(f"  c (AB): {calc.side_c:.4f}")
    
    print(f"\nAngles:")
    print(f"  Alpha: {math.degrees(calc.alpha):.2f}°")
    print(f"  Beta:  {math.degrees(calc.beta):.2f}°")
    print(f"  Gamma: {math.degrees(calc.gamma):.2f}°")
    print(f"  Sum:   {math.degrees(calc.alpha + calc.beta + calc.gamma):.2f}°")
    
    print(f"\nLambda values:")
    print(f"  Lambda A: {calc.lambda_A:.4f}")
    print(f"  Lambda B: {calc.lambda_B:.4f}")
    print(f"  Lambda C: {calc.lambda_C:.4f}")
    
    print_section("Result")
    print_vector("Fermat Point", calc.fermat_point)
    
    # Verify triangle validity
    total_angle = calc.alpha + calc.beta + calc.gamma
    if abs(total_angle - math.pi) < 0.01:
        print("Triangle validation: Valid")
    else:
        print(f"Triangle validation: Warning - angles sum to {math.degrees(total_angle):.2f}°")

if __name__ == "__main__":
    main()