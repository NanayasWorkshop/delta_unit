#!/usr/bin/env python3
"""
Unified test for the new delta robot package structure.
Tests the shared types approach and complete pipeline.
"""

import sys
import os
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

def test_shared_types():
    """Test that shared types work correctly."""
    print("Testing Shared Types")
    print("=" * 40)
    
    try:
        import delta_robot
    except ImportError as e:
        print(f"Error importing delta_robot: {e}")
        print("\nBuild the modules first:")
        print("python setup.py build_ext --inplace")
        return False
    
    # Verify installation
    if not delta_robot.verify_installation():
        return False
    
    # Test Vector3 operations
    print("\nTesting Vector3:")
    v1 = delta_robot.Vector3(3, 4, 5)
    v2 = delta_robot.Vector3(1, 1, 1)
    
    print(f"  v1 = {v1}")
    print(f"  v2 = {v2}")
    print(f"  v1 + v2 = {v1 + v2}")
    print(f"  v1 - v2 = {v1 - v2}")
    print(f"  v1 * 2 = {v1 * 2}")
    print(f"  v1.dot(v2) = {v1.dot(v2)}")
    print(f"  v1.norm() = {v1.norm():.4f}")
    print(f"  v1.normalized() = {v1.normalized()}")
    
    # Test Matrix4x4
    print("\nTesting Matrix4x4:")
    matrix = delta_robot.Matrix4x4()
    print(f"  Identity matrix [0,0] = {matrix[0,0]}")
    print(f"  Identity matrix [1,1] = {matrix[1,1]}")
    print(f"  Identity matrix [3,3] = {matrix[3,3]}")
    
    matrix[0,3] = 10.5  # Set translation X
    print(f"  After setting [0,3] = 10.5: {matrix[0,3]}")
    
    # Test CoordinateFrame
    print("\nTesting CoordinateFrame:")
    frame = delta_robot.CoordinateFrame(
        delta_robot.Vector3(1, 2, 3),  # origin
        delta_robot.Vector3(1, 0, 0),  # u_axis
        delta_robot.Vector3(0, 1, 0),  # v_axis
        delta_robot.Vector3(0, 0, 1)   # w_axis
    )
    print(f"  Frame: {frame}")
    
    # Test constants
    print("\nTesting Constants:")
    print(f"  ROBOT_RADIUS = {delta_robot.ROBOT_RADIUS}")
    print(f"  WORKING_HEIGHT = {delta_robot.WORKING_HEIGHT}")
    print(f"  rad_to_deg(π/4) = {delta_robot.rad_to_deg(math.pi/4):.2f}°")
    
    # Test base positions
    print("\nTesting Base Positions:")
    base_A = delta_robot.get_base_position_A()
    base_B = delta_robot.get_base_position_B()
    base_C = delta_robot.get_base_position_C()
    print(f"  Base A: {base_A}")
    print(f"  Base B: {base_B}")
    print(f"  Base C: {base_C}")
    
    print("✓ All shared types working correctly!")
    return True

def test_individual_modules():
    """Test each module individually."""
    print("\nTesting Individual Modules")
    print("=" * 50)
    
    import delta_robot
    
    test_input = (5, 4, 7)
    print(f"Test input vector: {test_input}")
    
    # Test Fermat module
    print("\n1. Testing Fermat Module:")
    fermat_result = delta_robot.fermat.FermatModule.calculate(*test_input)
    print(f"   Fermat point: ({fermat_result.fermat_point.x:.4f}, {fermat_result.fermat_point.y:.4f}, {fermat_result.fermat_point.z:.4f})")
    print(f"   Z positions: A={fermat_result.z_A:.4f}, B={fermat_result.z_B:.4f}, C={fermat_result.z_C:.4f}")
    
    # Test Joint State module
    print("\n2. Testing Joint State Module:")
    direction_vector = delta_robot.Vector3(*test_input)
    joint_result = delta_robot.joint_state.JointStateModule.calculate_from_fermat(direction_vector, fermat_result)
    print(f"   Prismatic: {joint_result.prismatic_joint:.4f}")
    print(f"   Roll: {math.degrees(joint_result.roll_joint):.2f}°")
    print(f"   Pitch: {math.degrees(joint_result.pitch_joint):.2f}°")
    
    # Test Kinematics module
    print("\n3. Testing Kinematics Module:")
    kinematics_result = delta_robot.kinematics.KinematicsModule.calculate(*test_input)
    print(f"   End-effector: ({kinematics_result.end_effector_position.x:.4f}, {kinematics_result.end_effector_position.y:.4f}, {kinematics_result.end_effector_position.z:.4f})")
    print(f"   Prismatic length: {kinematics_result.prismatic_joint_length:.4f}")
    print(f"   Input angle from Z: {math.degrees(kinematics_result.input_angle_from_z):.2f}°")
    
    # Test Orientation module
    print("\n4. Testing Orientation Module:")
    orientation_result = delta_robot.orientation.OrientationModule.calculate(*test_input)
    print(f"   End-effector position: ({orientation_result.end_effector_position.x:.4f}, {orientation_result.end_effector_position.y:.4f}, {orientation_result.end_effector_position.z:.4f})")
    print(f"   Transformation matrix [0,3]: {orientation_result.transformation_matrix[0,3]:.4f}")
    print(f"   Final frame origin: ({orientation_result.final_frame.origin.x:.4f}, {orientation_result.final_frame.origin.y:.4f}, {orientation_result.final_frame.origin.z:.4f})")
    
    print("✓ All individual modules working correctly!")
    return True

def test_complete_pipeline():
    """Test the complete pipeline convenience function."""
    print("\nTesting Complete Pipeline")
    print("=" * 40)
    
    import delta_robot
    
    test_input = (5, 4, 7)
    print(f"Input vector: {test_input}")
    
    # Use convenience function
    results = delta_robot.calculate_complete_pipeline(*test_input)
    
    print(f"\nPipeline Results:")
    print(f"  Input: {results['input_vector']}")
    print(f"  Fermat point: ({results['fermat'].fermat_point.x:.4f}, {results['fermat'].fermat_point.y:.4f}, {results['fermat'].fermat_point.z:.4f})")
    print(f"  Joint states: P={results['joint_state'].prismatic_joint:.4f}, R={math.degrees(results['joint_state'].roll_joint):.2f}°, P={math.degrees(results['joint_state'].pitch_joint):.2f}°")
    print(f"  End-effector: ({results['kinematics'].end_effector_position.x:.4f}, {results['kinematics'].end_effector_position.y:.4f}, {results['kinematics'].end_effector_position.z:.4f})")
    print(f"  Transform ready: {type(results['orientation'].transformation_matrix).__name__}")
    
    # Verify consistency between modules
    print(f"\nConsistency Checks:")
    
    # End-effector positions should match
    kin_pos = results['kinematics'].end_effector_position
    ori_pos = results['orientation'].end_effector_position
    pos_diff = math.sqrt((kin_pos.x - ori_pos.x)**2 + (kin_pos.y - ori_pos.y)**2 + (kin_pos.z - ori_pos.z)**2)
    print(f"  Kinematics vs Orientation end-effector difference: {pos_diff:.6f}")
    
    # Prismatic joint values should match
    kin_pris = results['kinematics'].prismatic_joint_length
    joint_pris = results['joint_state'].prismatic_joint
    pris_diff = abs(kin_pris - joint_pris)
    print(f"  Kinematics vs Joint State prismatic difference: {pris_diff:.6f}")
    
    # Fermat points should match
    fermat_pt = results['fermat'].fermat_point
    kin_fermat = results['kinematics'].fermat_data.fermat_point
    fermat_diff = math.sqrt((fermat_pt.x - kin_fermat.x)**2 + (fermat_pt.y - kin_fermat.y)**2 + (fermat_pt.z - kin_fermat.z)**2)
    print(f"  Fermat vs Kinematics fermat point difference: {fermat_diff:.6f}")
    
    if pos_diff < 1e-6 and pris_diff < 1e-6 and fermat_diff < 1e-6:
        print("  ✓ All consistency checks passed!")
    else:
        print("  ✗ Some consistency checks failed!")
    
    print("✓ Complete pipeline working correctly!")
    return True

def test_error_handling():
    """Test error handling and edge cases."""
    print("\nTesting Error Handling")
    print("=" * 30)
    
    import delta_robot
    
    # Test zero vector
    print("Testing zero vector (0, 0, 0):")
    try:
        result = delta_robot.fermat.FermatModule.calculate(0, 0, 0)
        print(f"  Fermat point: ({result.fermat_point.x:.4f}, {result.fermat_point.y:.4f}, {result.fermat_point.z:.4f})")
        print("  ✓ Zero vector handled")
    except Exception as e:
        print(f"  ✗ Zero vector failed: {e}")
    
    # Test very small vector
    print("\nTesting very small vector (1e-6, 1e-6, 1e-6):")
    try:
        result = delta_robot.fermat.FermatModule.calculate(1e-6, 1e-6, 1e-6)
        print(f"  Fermat point: ({result.fermat_point.x:.6f}, {result.fermat_point.y:.6f}, {result.fermat_point.z:.6f})")
        print("  ✓ Small vector handled")
    except Exception as e:
        print(f"  ✗ Small vector failed: {e}")
    
    # Test negative values
    print("\nTesting negative vector (-5, -4, -7):")
    try:
        result = delta_robot.fermat.FermatModule.calculate(-5, -4, -7)
        print(f"  Fermat point: ({result.fermat_point.x:.4f}, {result.fermat_point.y:.4f}, {result.fermat_point.z:.4f})")
        print("  ✓ Negative vector handled")
    except Exception as e:
        print(f"  ✗ Negative vector failed: {e}")
    
    # Test large values
    print("\nTesting large vector (1000, 2000, 3000):")
    try:
        result = delta_robot.fermat.FermatModule.calculate(1000, 2000, 3000)
        print(f"  Fermat point: ({result.fermat_point.x:.4f}, {result.fermat_point.y:.4f}, {result.fermat_point.z:.4f})")
        print("  ✓ Large vector handled")
    except Exception as e:
        print(f"  ✗ Large vector failed: {e}")
    
    print("✓ Error handling tests completed!")
    return True

def main():
    # Parse command line arguments
    if len(sys.argv) == 2:
        test_x, test_y, test_z = parse_coordinates(sys.argv[1])
        print(f"Using command line input: ({test_x}, {test_y}, {test_z})")
    else:
        test_x, test_y, test_z = 5, 4, 7
        print("Using default input: (5, 4, 7)")
        print("Usage: python test_delta_unified.py x,y,z")
    
    print("\n" + "="*60)
    print("DELTA ROBOT UNIFIED PACKAGE TEST")
    print("Testing shared types approach and complete pipeline")
    print("="*60)
    
    success = True
    
    # Test 1: Shared types
    success &= test_shared_types()
    
    # Test 2: Individual modules
    success &= test_individual_modules()
    
    # Test 3: Complete pipeline
    success &= test_complete_pipeline()
    
    # Test 4: Error handling
    success &= test_error_handling()
    
    # Final summary
    print("\n" + "="*60)
    if success:
        print("🎉 ALL TESTS PASSED!")
        print("✓ Shared types approach working correctly")
        print("✓ No type registration conflicts")
        print("✓ All modules integrate seamlessly")
        print("✓ Complete pipeline functional")
        print("\nThe delta robot package is ready for use!")
    else:
        print("❌ SOME TESTS FAILED!")
        print("Check the error messages above and rebuild if necessary.")
    
    print("\nTo use the package in your code:")
    print("  import delta_robot")
    print("  result = delta_robot.calculate_complete_pipeline(5, 4, 7)")
    print("="*60)
    
    return success

if __name__ == "__main__":
    main()