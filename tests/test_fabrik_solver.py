#!/usr/bin/env python3
"""
Test the complete FABRIK Solver module
"""

import sys
import os
import math
import time
# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def parse_coordinates(coord_str):
    """Parse coordinate string like '100,50,200' into x,y,z values"""
    try:
        coords = [float(x.strip()) for x in coord_str.split(',')]
        if len(coords) != 3:
            raise ValueError("Expected 3 coordinates")
        return coords
    except ValueError as e:
        print(f"Error parsing coordinates '{coord_str}': {e}")
        print("Expected format: x,y,z (e.g., 100,50,200)")
        sys.exit(1)

def calculate_distance(pos1, pos2):
    """Calculate Euclidean distance between two positions"""
    return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z - pos2.z)**2)

def print_solution_summary(result, test_name="FABRIK Solution"):
    """Print comprehensive solution summary"""
    print(f"\n=== {test_name.upper()} SUMMARY ===")
    print(f"Target: ({result.target_position.x:.2f}, {result.target_position.y:.2f}, {result.target_position.z:.2f})")
    print(f"Achieved: ({result.achieved_position.x:.2f}, {result.achieved_position.y:.2f}, {result.achieved_position.z:.2f})")
    print(f"Converged: {'✓ YES' if result.converged else '✗ NO'}")
    print(f"Final error: {result.final_error:.6f}")
    print(f"Total iterations: {result.total_iterations}")
    print(f"  - Backward: {result.backward_iterations}")
    print(f"  - Forward: {result.forward_iterations}")
    print(f"Solve time: {result.solve_time_ms:.2f} ms")
    
    if result.final_error <= 0.01:
        print("✓ EXCELLENT accuracy achieved!")
    elif result.final_error <= 0.1:
        print("✓ Good accuracy achieved")
    elif result.final_error <= 1.0:
        print("⚠️  Moderate accuracy")
    else:
        print("✗ Poor accuracy")

def print_convergence_history(result, max_show=10):
    """Print convergence history if available"""
    if not result.convergence_history:
        print("No convergence history tracked")
        return
    
    print(f"\n=== CONVERGENCE HISTORY ===")
    history = result.convergence_history
    target = result.target_position
    
    show_count = min(len(history), max_show)
    if len(history) > max_show:
        print(f"Showing first and last {max_show//2} iterations (total: {len(history)})...")
        
        # Show first half
        for i in range(max_show//2):
            pos = history[i]
            error = calculate_distance(pos, target)
            print(f"  Cycle {i}: Error = {error:.6f}")
        
        print("  ...")
        
        # Show last half
        for i in range(len(history) - max_show//2, len(history)):
            pos = history[i]
            error = calculate_distance(pos, target)
            print(f"  Cycle {i}: Error = {error:.6f}")
    else:
        for i, pos in enumerate(history):
            error = calculate_distance(pos, target)
            print(f"  Cycle {i}: Error = {error:.6f}")

def test_multiple_targets():
    """Test multiple target positions"""
    print(f"\n=== MULTIPLE TARGETS TEST ===")
    
    try:
        import delta_robot.fabrik_solver as fs
        import delta_robot.fabrik_initialization as fi
        import delta_robot.delta_types as dt
    except ImportError as e:
        print(f"Error: Module not found: {e}")
        return False
    
    # Initialize chain
    init_result = fi.FabrikInitialization.initialize_straight_up(3)
    
    # Define test targets
    test_targets = [
        (0, 0, 200),      # Straight up (easy)
        (100, 0, 300),    # Side reach
        (50, 50, 250),    # Diagonal
        (0, 100, 350),    # Forward reach
        (80, 80, 400),    # Near max reach
    ]
    
    config = fs.FabrikSolver.create_fast_config()
    
    for i, (x, y, z) in enumerate(test_targets):
        target = dt.Vector3(x, y, z)
        print(f"\nTarget {i+1}: ({x}, {y}, {z})")
        
        # Check reachability first
        reachable = fs.is_target_reachable(init_result.chain, target)
        print(f"  Reachable: {'✓ YES' if reachable else '✗ NO'}")
        
        if reachable:
            result = fs.FabrikSolver.solve_to_target(init_result.chain, target, config)
            print(f"  Converged: {'✓' if result.converged else '✗'}")
            print(f"  Error: {result.final_error:.4f}")
            print(f"  Time: {result.solve_time_ms:.1f}ms")
        else:
            print(f"  Skipped (unreachable)")

def test_different_configurations():
    """Test different solver configurations"""
    print(f"\n=== CONFIGURATION COMPARISON ===")
    
    try:
        import delta_robot.fabrik_solver as fs
        import delta_robot.fabrik_initialization as fi
        import delta_robot.delta_types as dt
    except ImportError as e:
        print(f"Error: Module not found: {e}")
        return False
    
    # Initialize chain
    init_result = fi.FabrikInitialization.initialize_straight_up(3)
    target = dt.Vector3(100, 100, 300)
    
    # Test different configurations
    configs = [
        ("Fast", fs.FabrikSolver.create_fast_config()),
        ("Precise", fs.FabrikSolver.create_precise_config()),
        ("Debug", fs.FabrikSolver.create_debug_config()),
    ]
    
    for config_name, config in configs:
        print(f"\n{config_name} Configuration:")
        print(f"  Tolerance: {config.tolerance}")
        print(f"  Max cycles: {config.max_backward_forward_cycles}")
        print(f"  Verbose: {config.verbose_logging}")
        
        start_time = time.time()
        result = fs.FabrikSolver.solve_to_target(init_result.chain, target, config)
        end_time = time.time()
        
        print(f"  Result: {'✓ Converged' if result.converged else '✗ No convergence'}")
        print(f"  Error: {result.final_error:.6f}")
        print(f"  Iterations: {result.total_iterations}")
        print(f"  Python time: {(end_time - start_time)*1000:.1f}ms")
        print(f"  C++ time: {result.solve_time_ms:.1f}ms")

def test_workspace_analysis():
    """Test workspace analysis functions"""
    print(f"\n=== WORKSPACE ANALYSIS ===")
    
    try:
        import delta_robot.fabrik_solver as fs
        import delta_robot.fabrik_initialization as fi
        import delta_robot.delta_types as dt
    except ImportError as e:
        print(f"Error: Module not found: {e}")
        return False
    
    # Initialize chain
    init_result = fi.FabrikInitialization.initialize_straight_up(3)
    
    # Test max reach in different directions
    directions = [
        (0, 0, 1),   # Up
        (1, 0, 0),   # Right
        (0, 1, 0),   # Forward
        (1, 1, 0),   # Diagonal
        (1, 1, 1),   # 3D diagonal
    ]
    
    print("Maximum reach analysis:")
    for x, y, z in directions:
        direction = dt.Vector3(x, y, z)
        max_reach = fs.find_max_reach(init_result.chain, direction)
        distance = math.sqrt(max_reach.x**2 + max_reach.y**2 + max_reach.z**2)
        
        print(f"  Direction ({x},{y},{z}): Max reach = ({max_reach.x:.1f}, {max_reach.y:.1f}, {max_reach.z:.1f}), Distance = {distance:.1f}")

def test_fabrik_solver():
    # Parse command line arguments for target position
    if len(sys.argv) == 2:
        target_x, target_y, target_z = parse_coordinates(sys.argv[1])
        print(f"Using command line target: ({target_x}, {target_y}, {target_z})")
    else:
        # Default target: reachable position
        target_x, target_y, target_z = 100, 100, 300
        print("Using default target: (100, 100, 300)")
        print("Usage: python3 test_fabrik_solver.py x,y,z")
    
    try:
        import delta_robot.fabrik_solver as fs
        import delta_robot.fabrik_initialization as fi
        import delta_robot.delta_types as dt
    except ImportError as e:
        print(f"Error: Module not found: {e}")
        print("Build all FABRIK modules first (initialization, backward, forward, solver).")
        return False
    
    print("Testing Complete FABRIK Solver Module")
    print("=" * 80)
    
    # Test 1: Basic solving
    print(f"\n=== BASIC SOLVING TEST ===")
    
    # Initialize robot chain
    print("Initializing 3-segment robot chain...")
    init_result = fi.FabrikInitialization.initialize_straight_up(3)
    initial_chain = init_result.chain
    
    print(f"Initial chain: {len(initial_chain.joints)} joints, total reach: {init_result.total_reach:.1f}")
    
    # Create target position
    target_position = dt.Vector3(target_x, target_y, target_z)
    
    # Check reachability
    is_reachable = fs.is_target_reachable(initial_chain, target_position)
    distance_to_target = math.sqrt(target_x**2 + target_y**2 + target_z**2)
    
    print(f"Target: ({target_x}, {target_y}, {target_z})")
    print(f"Distance from base: {distance_to_target:.2f}")
    print(f"Maximum reach: {init_result.total_reach:.2f}")
    print(f"Reachable: {'✓ YES' if is_reachable else '✗ NO'}")
    
    if not is_reachable:
        print(f"⚠️  Target is {distance_to_target - init_result.total_reach:.2f} units beyond reach")
        print("Trying anyway (solver should handle unreachable targets)...")
    
    # Solve with default configuration
    print(f"\nSolving with default configuration...")
    result = fs.FabrikSolver.solve(initial_chain, target_position)
    
    print_solution_summary(result, "Default Configuration")
    
    # Test 2: Solve with detailed configuration
    print(f"\n=== DETAILED CONFIGURATION TEST ===")
    
    config = fs.FabrikSolverConfig()
    config.tolerance = 0.001
    config.max_backward_forward_cycles = 100
    config.track_convergence_history = True
    config.verbose_logging = False
    
    print(f"Using detailed configuration:")
    print(f"  Tolerance: {config.tolerance}")
    print(f"  Max cycles: {config.max_backward_forward_cycles}")
    print(f"  Track history: {config.track_convergence_history}")
    
    result_detailed = fs.FabrikSolver.solve_to_target(initial_chain, target_position, config)
    
    print_solution_summary(result_detailed, "Detailed Configuration")
    print_convergence_history(result_detailed)
    
    # Test 3: Convenience function
    print(f"\n=== CONVENIENCE FUNCTION TEST ===")
    
    result_convenience = fs.solve_delta_robot(3, target_position, 0.01)
    print_solution_summary(result_convenience, "Convenience Function")
    
    # Test 4: Solution validation
    print(f"\n=== SOLUTION VALIDATION ===")
    
    final_chain = result.final_chain
    
    is_valid = fs.FabrikSolver.is_solution_valid(final_chain)
    chain_error = fs.FabrikSolver.calculate_chain_error(final_chain)
    
    print(f"Solution valid: {'✓ YES' if is_valid else '✗ NO'}")
    print(f"Chain error: {chain_error:.6f}")
    
    # Check base position
    base_pos = final_chain.joints[0].position
    base_distance_from_origin = math.sqrt(base_pos.x**2 + base_pos.y**2 + base_pos.z**2)
    print(f"Base position: ({base_pos.x:.6f}, {base_pos.y:.6f}, {base_pos.z:.6f})")
    print(f"Base distance from origin: {base_distance_from_origin:.6f}")
    
    # Check segment lengths
    print(f"\nSegment length verification:")
    all_lengths_ok = True
    for i, segment in enumerate(final_chain.segments):
        start_pos = final_chain.joints[segment.start_joint_index].position
        end_pos = final_chain.joints[segment.end_joint_index].position
        
        actual_length = calculate_distance(start_pos, end_pos)
        expected_length = segment.length
        error = abs(actual_length - expected_length)
        
        status = "✓" if error < 0.01 else "✗"
        print(f"  Segment {i}: Expected={expected_length:.2f}, Actual={actual_length:.2f}, Error={error:.4f} {status}")
        
        if error >= 0.01:
            all_lengths_ok = False
    
    # Run additional tests
    test_multiple_targets()
    test_different_configurations()
    test_workspace_analysis()
    
    # Final summary
    print(f"\n=== FINAL SUMMARY ===")
    
    success_criteria = [
        (result.converged, "Basic solving converged"),
        (result.final_error <= 1.0, "Reasonable accuracy achieved"),
        (is_valid, "Solution is structurally valid"),
        (all_lengths_ok, "Segment lengths preserved"),
        (base_distance_from_origin <= 0.1, "Base properly fixed at origin"),
    ]
    
    all_success = all(criterion[0] for criterion in success_criteria)
    
    for success, description in success_criteria:
        status = "✓" if success else "✗"
        print(f"  {status} {description}")
    
    if all_success:
        print("\n🎉 FABRIK Solver working perfectly!")
        print("✓ Complete FABRIK algorithm implemented successfully")
        print("✓ Ready for production use in delta robot control")
    else:
        print("\n⚠️  Some issues detected - check implementation")
    
    print(f"\nPerformance summary:")
    print(f"  - Target: ({target_x}, {target_y}, {target_z})")
    print(f"  - Reachable: {is_reachable}")
    print(f"  - Converged: {result.converged}")
    print(f"  - Final error: {result.final_error:.6f}")
    print(f"  - Total iterations: {result.total_iterations}")
    print(f"  - Solve time: {result.solve_time_ms:.2f}ms")
    print(f"  - Valid solution: {is_valid}")
    
    return True

if __name__ == "__main__":
    test_fabrik_solver()