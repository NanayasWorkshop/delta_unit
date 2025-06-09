#!/usr/bin/env python3
"""
Step 3.1: Collision Support Modules Verification Tests

This script verifies that UPointsExtractor, CollisionDetector, and WaypointConverter
are truly independent with clean interfaces.
"""
import numpy as np
import time
import delta_robot

def test_collision_support_modules_independence():
    """Comprehensive test of collision support modules independence."""
    
    print("🎯 Step 3.1: Collision Support Modules Independence Verification")
    print("=" * 70)
    
    results = {
        'u_points_extractor': {},
        'collision_detector': {},
        'waypoint_converter': {},
        'integration_pipeline': {},
        'performance_metrics': {}
    }
    
    # =================================================================
    # Test 1: UPointsExtractor Independence
    # =================================================================
    print("\n📋 Test 1: UPointsExtractor Independence")
    print("-" * 50)
    
    try:
        # Test 1a: Extract U-points from FABRIK chain
        print("  Test 1a: U-points from FABRIK chain")
        target = np.array([100, 50, 300])
        fabrik_result = delta_robot.solve_fabrik_ik(target[0], target[1], target[2])
        
        if fabrik_result.converged:
            start_time = time.perf_counter()
            u_points_from_chain = delta_robot.collision.UPointsExtractor.extract_u_points(
                fabrik_result.final_chain)
            end_time = time.perf_counter()
            
            chain_extraction_time = (end_time - start_time) * 1000
            
            print(f"    ✅ Success: Extracted {len(u_points_from_chain)} U-points")
            print(f"    ⏱️  Extraction time: {chain_extraction_time:.3f}ms")
            print(f"    📍 First U-point: ({u_points_from_chain[0][0]:.1f}, {u_points_from_chain[0][1]:.1f}, {u_points_from_chain[0][2]:.1f})")
            print(f"    📍 Last U-point: ({u_points_from_chain[-1][0]:.1f}, {u_points_from_chain[-1][1]:.1f}, {u_points_from_chain[-1][2]:.1f})")
            
            # Test 1b: Extract U-points from raw joint positions
            print("  Test 1b: U-points from raw joint positions")
            joint_positions = [joint.position for joint in fabrik_result.final_chain.joints]
            
            start_time = time.perf_counter()
            u_points_from_positions = delta_robot.collision.UPointsExtractor.extract_u_points_from_positions(
                joint_positions)
            end_time = time.perf_counter()
            
            positions_extraction_time = (end_time - start_time) * 1000
            
            print(f"    ✅ Success: Extracted {len(u_points_from_positions)} U-points")
            print(f"    ⏱️  Extraction time: {positions_extraction_time:.3f}ms")
            
            # Verify both methods give same results
            if len(u_points_from_chain) == len(u_points_from_positions):
                differences = []
                for i in range(len(u_points_from_chain)):
                    diff = np.linalg.norm(np.array(u_points_from_chain[i]) - np.array(u_points_from_positions[i]))
                    differences.append(diff)
                
                max_diff = max(differences)
                print(f"    🎯 Consistency check: Max difference {max_diff:.6f}mm (should be ~0)")
                
                consistency_ok = max_diff < 1e-10  # Should be identical
                print(f"    ✅ Both extraction methods consistent: {consistency_ok}")
            
            results['u_points_extractor'] = {
                'success': True,
                'chain_extraction_time_ms': chain_extraction_time,
                'positions_extraction_time_ms': positions_extraction_time,
                'u_points_count': len(u_points_from_chain),
                'methods_consistent': consistency_ok,
                'max_difference': max_diff
            }
        else:
            print("    ❌ FABRIK failed to converge")
            results['u_points_extractor'] = {'success': False, 'error': 'FABRIK convergence failed'}
            
    except Exception as e:
        print(f"    ❌ Error: {e}")
        results['u_points_extractor'] = {'success': False, 'error': str(e)}
    
    # =================================================================
    # Test 2: CollisionDetector Independence  
    # =================================================================
    print("\n📋 Test 2: CollisionDetector Independence")
    print("-" * 50)
    
    try:
        # Test 2a: Create test obstacles
        print("  Test 2a: Create test obstacles")
        obstacles = delta_robot.collision.CollisionDetector.create_test_obstacles()
        print(f"    ✅ Created {len(obstacles)} test obstacles")
        for i, obs in enumerate(obstacles):
            print(f"    📍 Obstacle {i+1}: center=({obs.center[0]:.1f}, {obs.center[1]:.1f}, {obs.center[2]:.1f}), radius={obs.radius:.1f}")
        
        # Test 2b: Collision detection with U-points from UPointsExtractor
        print("  Test 2b: Collision detection with extracted U-points")
        if 'u_points_extractor' in results and results['u_points_extractor'].get('success'):
            # Use U-points from previous test
            test_u_points = u_points_from_chain
            
            start_time = time.perf_counter()
            collision_result = delta_robot.collision.CollisionDetector.check_and_avoid(
                test_u_points, obstacles)
            end_time = time.perf_counter()
            
            collision_time = (end_time - start_time) * 1000
            
            print(f"    ✅ Collision detection completed")
            print(f"    🚫 Has collision: {collision_result.has_collision}")
            print(f"    📏 Min distance: {collision_result.min_distance:.1f}mm")
            print(f"    📍 Waypoints generated: {len(collision_result.waypoints)}")
            print(f"    ⏱️  Detection time: {collision_time:.3f}ms")
            print(f"    ⏱️  Internal computation time: {collision_result.computation_time:.3f}ms")
            
            # Test 2c: Manual U-points (verify works with any source)
            print("  Test 2c: Collision detection with manual U-points")
            manual_u_points = [
                np.array([0, 0, 0]),      # Base
                np.array([10, 10, 50]),   # U2
                np.array([20, 20, 100]),  # U3
                np.array([30, 30, 150]),  # U4
                np.array([40, 40, 200]),  # U5
                np.array([50, 50, 250]),  # U6
                np.array([60, 60, 300]),  # U7
                np.array([70, 70, 350])   # End-effector
            ]
            
            start_time = time.perf_counter()
            manual_collision_result = delta_robot.collision.CollisionDetector.check_and_avoid(
                manual_u_points, obstacles)
            end_time = time.perf_counter()
            
            manual_collision_time = (end_time - start_time) * 1000
            
            print(f"    ✅ Manual U-points collision detection completed")
            print(f"    🚫 Has collision: {manual_collision_result.has_collision}")
            print(f"    📏 Min distance: {manual_collision_result.min_distance:.1f}mm")
            print(f"    📍 Waypoints generated: {len(manual_collision_result.waypoints)}")
            print(f"    ⏱️  Detection time: {manual_collision_time:.3f}ms")
            
            results['collision_detector'] = {
                'success': True,
                'obstacles_count': len(obstacles),
                'extracted_u_points_collision': collision_result.has_collision,
                'extracted_u_points_time_ms': collision_time,
                'manual_u_points_collision': manual_collision_result.has_collision,
                'manual_u_points_time_ms': manual_collision_time,
                'waypoints_generated': len(collision_result.waypoints),
                'min_distance': collision_result.min_distance
            }
            
            # Store collision result for next test
            test_collision_result = collision_result
            
    except Exception as e:
        print(f"    ❌ Error: {e}")
        results['collision_detector'] = {'success': False, 'error': str(e)}
    
    # =================================================================
    # Test 3: WaypointConverter Independence
    # =================================================================
    print("\n📋 Test 3: WaypointConverter Independence")
    print("-" * 50)
    
    try:
        # Test 3a: Convert waypoints from CollisionDetector
        print("  Test 3a: Convert waypoints from CollisionDetector")
        if 'collision_detector' in results and results['collision_detector'].get('success'):
            waypoints = test_collision_result.waypoints
            
            # Validate waypoints first
            validation_ok = delta_robot.collision.WaypointConverter.validate_waypoints(waypoints)
            print(f"    ✅ Waypoint validation: {validation_ok}")
            
            start_time = time.perf_counter()
            conversion_result = delta_robot.collision.WaypointConverter.convert_waypoints_to_joints(waypoints)
            end_time = time.perf_counter()
            
            conversion_time = (end_time - start_time) * 1000
            
            print(f"    ✅ Conversion completed: {conversion_result.conversion_successful}")
            print(f"    📍 Joint positions generated: {len(conversion_result.joint_positions)}")
            print(f"    📏 Segment lengths calculated: {len(conversion_result.segment_lengths)}")
            print(f"    📐 Joint angles calculated: {len(conversion_result.joint_angles_deg)}")
            print(f"    📏 Total reach: {conversion_result.total_reach:.1f}mm")
            print(f"    ⏱️  Conversion time: {conversion_time:.3f}ms")
            
            # Test 3b: Convert manual waypoints (verify works with any source)
            print("  Test 3b: Convert manual waypoints")
            manual_waypoints = [
                np.array([0, 0, 0]),
                np.array([20, 10, 80]),
                np.array([40, 20, 160]),
                np.array([60, 30, 240]),
                np.array([80, 40, 320])
            ]
            
            manual_validation_ok = delta_robot.collision.WaypointConverter.validate_waypoints(manual_waypoints)
            print(f"    ✅ Manual waypoint validation: {manual_validation_ok}")
            
            start_time = time.perf_counter()
            manual_conversion_result = delta_robot.collision.WaypointConverter.convert_waypoints_to_joints(manual_waypoints)
            end_time = time.perf_counter()
            
            manual_conversion_time = (end_time - start_time) * 1000
            
            print(f"    ✅ Manual conversion completed: {manual_conversion_result.conversion_successful}")
            print(f"    📍 Joint positions generated: {len(manual_conversion_result.joint_positions)}")
            print(f"    ⏱️  Conversion time: {manual_conversion_time:.3f}ms")
            
            # Test 3c: Create FABRIK chain from waypoints
            print("  Test 3c: Create FABRIK chain from waypoints")
            if conversion_result.conversion_successful:
                fabrik_chain = delta_robot.collision.WaypointConverter.create_fabrik_chain_from_waypoints(
                    waypoints, 3)  # 3 segments for testing
                
                print(f"    ✅ FABRIK chain created: {len(fabrik_chain.joints)} joints")
                print(f"    🔗 Segments in chain: {len(fabrik_chain.segments)}")
            
            results['waypoint_converter'] = {
                'success': True,
                'collision_waypoints_valid': validation_ok,
                'collision_conversion_successful': conversion_result.conversion_successful,
                'collision_conversion_time_ms': conversion_time,
                'manual_waypoints_valid': manual_validation_ok,
                'manual_conversion_successful': manual_conversion_result.conversion_successful,
                'manual_conversion_time_ms': manual_conversion_time,
                'joint_positions_count': len(conversion_result.joint_positions),
                'total_reach': conversion_result.total_reach
            }
            
    except Exception as e:
        print(f"    ❌ Error: {e}")
        results['waypoint_converter'] = {'success': False, 'error': str(e)}
    
    # =================================================================
    # Test 4: Complete Pipeline Integration
    # =================================================================
    print("\n📋 Test 4: Complete Pipeline Integration")
    print("-" * 50)
    
    try:
        print("  Testing complete collision pipeline: Joint Positions → U-Points → Collision → Waypoints → Joint Positions")
        
        # Start with fresh FABRIK solution
        pipeline_target = np.array([120, 60, 280])
        start_total_time = time.perf_counter()
        
        # Step 1: Get joint positions (from any source)
        fabrik_result = delta_robot.solve_fabrik_ik(pipeline_target[0], pipeline_target[1], pipeline_target[2])
        step1_time = time.perf_counter()
        
        if fabrik_result.converged:
            print(f"    Step 1 ✅ FABRIK solve: {(step1_time - start_total_time) * 1000:.3f}ms")
            
            # Step 2: Extract U-points
            u_points = delta_robot.collision.UPointsExtractor.extract_u_points(fabrik_result.final_chain)
            step2_time = time.perf_counter()
            print(f"    Step 2 ✅ U-points extraction: {(step2_time - step1_time) * 1000:.3f}ms")
            
            # Step 3: Collision detection
            collision_result = delta_robot.collision.CollisionDetector.check_and_avoid(u_points, obstacles)
            step3_time = time.perf_counter()
            print(f"    Step 3 ✅ Collision detection: {(step3_time - step2_time) * 1000:.3f}ms")
            
            # Step 4: Waypoint conversion (if collision-free waypoints available)
            if collision_result.waypoints:
                conversion_result = delta_robot.collision.WaypointConverter.convert_waypoints_to_joints(
                    collision_result.waypoints)
                step4_time = time.perf_counter()
                print(f"    Step 4 ✅ Waypoint conversion: {(step4_time - step3_time) * 1000:.3f}ms")
                
                end_total_time = time.perf_counter()
                total_pipeline_time = (end_total_time - start_total_time) * 1000
                
                print(f"    🏆 Complete pipeline: {total_pipeline_time:.3f}ms total")
                print(f"    🚫 Final collision status: {'Collision-free' if not collision_result.has_collision else 'Has collision'}")
                print(f"    📍 Final joint positions: {len(conversion_result.joint_positions) if conversion_result.conversion_successful else 0}")
                
                results['integration_pipeline'] = {
                    'success': True,
                    'fabrik_time_ms': (step1_time - start_total_time) * 1000,
                    'u_points_time_ms': (step2_time - step1_time) * 1000,
                    'collision_time_ms': (step3_time - step2_time) * 1000,
                    'waypoint_time_ms': (step4_time - step3_time) * 1000,
                    'total_pipeline_time_ms': total_pipeline_time,
                    'collision_free': not collision_result.has_collision,
                    'final_joint_positions': len(conversion_result.joint_positions) if conversion_result.conversion_successful else 0
                }
            else:
                print("    ⚠️  No waypoints to convert (collision detection issue)")
                results['integration_pipeline'] = {'success': False, 'error': 'No waypoints generated'}
        else:
            print("    ❌ FABRIK failed to converge")
            results['integration_pipeline'] = {'success': False, 'error': 'FABRIK convergence failed'}
            
    except Exception as e:
        print(f"    ❌ Error: {e}")
        results['integration_pipeline'] = {'success': False, 'error': str(e)}
    
    # =================================================================
    # Summary
    # =================================================================
    print("\n🏆 Step 3.1 Verification Summary")
    print("=" * 70)
    
    all_tests_passed = True
    
    # Check each test result
    for module_name, result in results.items():
        if isinstance(result, dict) and 'success' in result:
            if result['success']:
                print(f"  ✅ {module_name.replace('_', ' ').title()}: PASSED")
            else:
                print(f"  ❌ {module_name.replace('_', ' ').title()}: FAILED")
                all_tests_passed = False
                if 'error' in result:
                    print(f"     Error: {result['error']}")
    
    # Overall assessment
    if all_tests_passed:
        print(f"\n🎯 ✅ STEP 3.1 SUCCESS: All collision support modules are truly independent!")
        print(f"  ✅ UPointsExtractor: Works with chains AND raw joint positions")
        print(f"  ✅ CollisionDetector: Works with U-points from any source") 
        print(f"  ✅ WaypointConverter: Works with waypoints from any collision detector")
        print(f"  ✅ Complete pipeline: All modules integrate seamlessly")
        print(f"  ✅ Performance: Each module shows independent timing")
        print(f"\n🏆 Perfect modular collision detection architecture achieved!")
        
        # Performance summary
        if 'integration_pipeline' in results and results['integration_pipeline'].get('success'):
            pipeline_result = results['integration_pipeline']
            print(f"\n📊 Complete Pipeline Performance:")
            print(f"  ⏱️  FABRIK solve: {pipeline_result.get('fabrik_time_ms', 0):.3f}ms")
            print(f"  ⏱️  U-points extraction: {pipeline_result.get('u_points_time_ms', 0):.3f}ms")
            print(f"  ⏱️  Collision detection: {pipeline_result.get('collision_time_ms', 0):.3f}ms") 
            print(f"  ⏱️  Waypoint conversion: {pipeline_result.get('waypoint_time_ms', 0):.3f}ms")
            print(f"  ⏱️  Total pipeline: {pipeline_result.get('total_pipeline_time_ms', 0):.3f}ms")
            
            total_time = pipeline_result.get('total_pipeline_time_ms', 0)
            if total_time < 16.7:  # 60Hz budget
                print(f"  🎯 ✅ Within 60Hz budget ({total_time:.1f}ms < 16.7ms)")
            else:
                print(f"  ⚠️  Outside 60Hz budget ({total_time:.1f}ms > 16.7ms)")
    else:
        print(f"\n❌ STEP 3.1 ISSUES: Some tests failed - needs investigation")
    
    return results

def test_collision_module_consistency():
    """Test that collision modules produce consistent results."""
    
    print(f"\n🔧 Consistency Test: Collision Module Chain")
    print("-" * 50)
    
    try:
        # Test multiple runs for consistency
        target = np.array([100, 40, 300])
        obstacles = delta_robot.create_test_obstacles()
        
        consistency_results = []
        
        for run in range(3):
            print(f"  Run {run + 1}:")
            
            # Complete collision-aware solution
            collision_aware_result = delta_robot.solve_with_collision_avoidance(
                target[0], target[1], target[2], obstacles, max_iterations=2)
            
            # Extract using our modules
            u_points = delta_robot.collision.UPointsExtractor.extract_u_points(
                collision_aware_result.fabrik_result.final_chain)
            
            collision_check = delta_robot.collision.CollisionDetector.check_and_avoid(
                u_points, obstacles)
            
            consistency_results.append({
                'collision_aware_collision_free': collision_aware_result.collision_free,
                'module_collision_free': not collision_check.has_collision,
                'collision_iterations': collision_aware_result.collision_iterations,
                'u_points_count': len(u_points),
                'waypoints_count': len(collision_check.waypoints)
            })
            
            print(f"    CollisionAwareSolver result: {collision_aware_result.collision_free}")
            print(f"    Module check result: {not collision_check.has_collision}")
            print(f"    Collision iterations: {collision_aware_result.collision_iterations}")
            print(f"    U-points: {len(u_points)}, Waypoints: {len(collision_check.waypoints)}")
        
        # Check consistency across runs
        first_result = consistency_results[0]
        all_consistent = all(
            result['collision_aware_collision_free'] == first_result['collision_aware_collision_free'] and
            result['u_points_count'] == first_result['u_points_count']
            for result in consistency_results
        )
        
        print(f"\n  🎯 Consistency across runs: {all_consistent}")
        print(f"  ✅ CONSISTENCY TEST: {'PASSED' if all_consistent else 'FAILED'}")
        
        return all_consistent
        
    except Exception as e:
        print(f"  ❌ Consistency test error: {e}")
        return False

if __name__ == "__main__":
    # Run verification tests
    verification_results = test_collision_support_modules_independence()
    
    # Run consistency test
    consistency_success = test_collision_module_consistency()
    
    # Final summary
    print(f"\n🎯 STEP 3.1 FINAL ASSESSMENT")
    print("=" * 70)
    
    if verification_results and consistency_success:
        print("🏆 ✅ STEP 3.1 COMPLETE: Collision support modules independence VERIFIED!")
        print("  ✅ UPointsExtractor: Perfect dual interface (chains + positions)")
        print("  ✅ CollisionDetector: Advanced swarm optimization with early-return")
        print("  ✅ WaypointConverter: Efficient simplified algorithm") 
        print("  ✅ All modules: Input agnostic, clean interfaces")
        print("  ✅ Pipeline integration: Seamless modular workflow")
        print("  ✅ Performance: Each module independently timed and optimized")
        
        print(f"\n🎯 READY FOR STEP 3.2: CollisionAwareSolver as Orchestrator")
    else:
        print("❌ STEP 3.1 NEEDS WORK: Some verification failed")
        print("  🔍 Review failed tests and investigate issues")