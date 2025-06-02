#!/usr/bin/env python3
"""
Complete FABRIK Solver Visual Debugger
Shows detailed step-by-step visualization of the entire FABRIK algorithm
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
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

def extract_joint_positions(chain):
    """Extract joint positions as numpy array"""
    positions = []
    for joint in chain.joints:
        positions.append([joint.position.x, joint.position.y, joint.position.z])
    return np.array(positions)

def calculate_segment_errors(chain, expected_lengths):
    """Calculate segment length errors"""
    errors = []
    actual_lengths = []
    
    for i, segment in enumerate(chain.segments):
        start_pos = chain.joints[segment.start_joint_index].position
        end_pos = chain.joints[segment.end_joint_index].position
        
        actual_length = calculate_distance(start_pos, end_pos)
        expected_length = expected_lengths[i] if i < len(expected_lengths) else segment.length
        
        error = abs(actual_length - expected_length)
        errors.append(error)
        actual_lengths.append(actual_length)
    
    return errors, actual_lengths

def plot_chain_3d(ax, chain, color='blue', alpha=1.0, linewidth=2, label='Chain', show_joints=True):
    """Plot a robot chain in 3D"""
    positions = extract_joint_positions(chain)
    
    if show_joints:
        # Plot joints with different styles
        for i, pos in enumerate(positions):
            joint_type = str(chain.joints[i].type)
            if 'FIXED_BASE' in joint_type:
                ax.scatter(*pos, color='black', s=150, marker='s', alpha=alpha, 
                          edgecolor='white', linewidth=1, label=f'{label} Base' if i == 0 else "")
            elif 'SPHERICAL' in joint_type:
                ax.scatter(*pos, color=color, s=80, marker='o', alpha=alpha, 
                          edgecolor='white', linewidth=1)
            else:  # END_EFFECTOR
                ax.scatter(*pos, color='red', s=120, marker='*', alpha=alpha, 
                          edgecolor='white', linewidth=1, label=f'{label} End-Effector' if i == len(positions)-1 else "")
    
    # Plot segments
    for i in range(len(positions) - 1):
        ax.plot([positions[i][0], positions[i+1][0]], 
                [positions[i][1], positions[i+1][1]], 
                [positions[i][2], positions[i+1][2]], 
                color=color, linewidth=linewidth, alpha=alpha,
                label=f'{label}' if i == 0 else "")
    
    return positions

def create_convergence_animation_data(target_x, target_y, target_z, verbose=False):
    """Perform FABRIK solving and collect all intermediate states"""
    
    try:
        import delta_robot.fabrik_initialization as fi
        import delta_robot.fabrik_backward as fb
        import delta_robot.fabrik_forward as ff
        import delta_robot.fabrik_solver as fs
        import delta_robot.delta_types as dt
    except ImportError as e:
        print(f"Error: delta_robot package not found: {e}")
        return None
    
    # Initialize chain
    init_result = fi.FabrikInitialization.initialize_straight_up(3)
    initial_chain = init_result.chain
    target_position = dt.Vector3(target_x, target_y, target_z)
    
    # Store all states during solving
    all_states = []
    cycle_info = []
    
    # Manual FABRIK cycles to capture intermediate states
    current_chain = initial_chain
    max_cycles = 20  # Limit for visualization
    tolerance = 0.01
    
    all_states.append({
        'chain': current_chain,
        'phase': 'Initial',
        'cycle': 0,
        'end_effector': fs.FabrikSolver.get_end_effector_position(current_chain),
        'error': calculate_distance(fs.FabrikSolver.get_end_effector_position(current_chain), target_position),
        'base_error': current_chain.joints[0].position.norm(),
    })
    
    for cycle in range(max_cycles):
        # Backward iteration
        backward_result = fb.FabrikBackward.iterate_to_target(current_chain, target_position, tolerance, 50)
        backward_chain = backward_result.updated_chain
        
        backward_ee = fs.FabrikSolver.get_end_effector_position(backward_chain)
        backward_error = calculate_distance(backward_ee, target_position)
        
        all_states.append({
            'chain': backward_chain,
            'phase': 'Backward',
            'cycle': cycle + 1,
            'end_effector': backward_ee,
            'error': backward_error,
            'base_error': backward_chain.joints[0].position.norm(),
            'iterations': backward_result.iterations_used
        })
        
        # Forward iteration
        forward_result = ff.FabrikForward.iterate_from_base(backward_chain, tolerance, 50)
        forward_chain = forward_result.updated_chain
        
        forward_ee = fs.FabrikSolver.get_end_effector_position(forward_chain)
        forward_error = calculate_distance(forward_ee, target_position)
        
        all_states.append({
            'chain': forward_chain,
            'phase': 'Forward',
            'cycle': cycle + 1,
            'end_effector': forward_ee,
            'error': forward_error,
            'base_error': forward_chain.joints[0].position.norm(),
            'iterations': forward_result.iterations_used
        })
        
        cycle_info.append({
            'cycle': cycle + 1,
            'backward_iterations': backward_result.iterations_used,
            'forward_iterations': forward_result.iterations_used,
            'backward_error': backward_error,
            'forward_error': forward_error,
            'converged': forward_error <= tolerance
        })
        
        current_chain = forward_chain
        
        if verbose:
            print(f"Cycle {cycle + 1}: Backward error = {backward_error:.4f}, Forward error = {forward_error:.4f}")
        
        # Check convergence
        if forward_error <= tolerance:
            print(f"Converged after {cycle + 1} cycles!")
            break
    
    return {
        'states': all_states,
        'cycle_info': cycle_info,
        'target': target_position,
        'initial_chain': initial_chain,
        'max_reach': init_result.total_reach
    }

def visualize_fabrik_solver_complete(target_x, target_y, target_z):
    """Create comprehensive visualization of complete FABRIK solver"""
    
    print(f"Collecting FABRIK solving data for target ({target_x}, {target_y}, {target_z})...")
    animation_data = create_convergence_animation_data(target_x, target_y, target_z, verbose=True)
    
    if animation_data is None:
        return False
    
    states = animation_data['states']
    cycle_info = animation_data['cycle_info']
    target = animation_data['target']
    max_reach = animation_data['max_reach']
    
    # Create large figure with multiple subplots
    fig = plt.figure(figsize=(24, 12))
    
    # Main 3D visualization (largest plot)
    ax_3d = fig.add_subplot(231, projection='3d')
    
    # Plot target
    ax_3d.scatter([target.x], [target.y], [target.z], color='gold', s=300, 
                  marker='X', label='Target', edgecolor='black', linewidth=2)
    
    # Plot initial chain
    initial_positions = plot_chain_3d(ax_3d, states[0]['chain'], color='lightblue', 
                                     alpha=0.3, linewidth=2, label='Initial')
    
    # Plot several intermediate states
    colors = ['orange', 'red', 'purple', 'green', 'brown']
    for i, state in enumerate(states[1::4]):  # Every 4th state to avoid clutter
        if i >= len(colors):
            break
        phase_label = f"Cycle {state['cycle']} {state['phase']}"
        plot_chain_3d(ax_3d, state['chain'], color=colors[i], 
                     alpha=0.6 + i*0.1, linewidth=2, label=phase_label, show_joints=False)
    
    # Plot final chain
    if states:
        final_positions = plot_chain_3d(ax_3d, states[-1]['chain'], color='red', 
                                       alpha=1.0, linewidth=4, label='Final')
    
    # Set 3D plot properties
    max_range = max(abs(target.x), abs(target.y), abs(target.z), max_reach) * 1.1
    ax_3d.set_xlim([-max_range/4, max_range])
    ax_3d.set_ylim([-max_range/4, max_range])
    ax_3d.set_zlim([0, max_range])
    ax_3d.set_xlabel('X')
    ax_3d.set_ylabel('Y')
    ax_3d.set_zlabel('Z')
    ax_3d.set_title('FABRIK Solver Convergence', fontsize=12, weight='bold')
    ax_3d.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    # Error convergence plot
    ax_error = fig.add_subplot(232)
    
    cycles = [s['cycle'] for s in states]
    errors = [s['error'] for s in states]
    base_errors = [s['base_error'] for s in states]
    phases = [s['phase'] for s in states]
    
    # Color points by phase
    for i, (cycle, error, phase) in enumerate(zip(cycles, errors, phases)):
        color = 'blue' if phase == 'Initial' else ('orange' if phase == 'Backward' else 'red')
        marker = 'o' if phase == 'Initial' else ('s' if phase == 'Backward' else '^')
        ax_error.scatter(i, error, color=color, marker=marker, s=50, alpha=0.8)
    
    ax_error.plot(range(len(errors)), errors, 'k-', alpha=0.3, linewidth=1)
    ax_error.set_xlabel('Step')
    ax_error.set_ylabel('End-Effector Error')
    ax_error.set_title('Convergence Progress')
    ax_error.grid(True, alpha=0.3)
    ax_error.legend(['Initial', 'Backward', 'Forward'])
    
    # Add horizontal line for tolerance
    ax_error.axhline(y=0.01, color='green', linestyle='--', alpha=0.7, label='Tolerance')
    
    # Base error plot
    ax_base = fig.add_subplot(233)
    
    for i, (cycle, base_error, phase) in enumerate(zip(cycles, base_errors, phases)):
        color = 'blue' if phase == 'Initial' else ('orange' if phase == 'Backward' else 'red')
        marker = 'o' if phase == 'Initial' else ('s' if phase == 'Backward' else '^')
        ax_base.scatter(i, base_error, color=color, marker=marker, s=50, alpha=0.8)
    
    ax_base.plot(range(len(base_errors)), base_errors, 'k-', alpha=0.3, linewidth=1)
    ax_base.set_xlabel('Step')
    ax_base.set_ylabel('Base Position Error')
    ax_base.set_title('Base Constraint')
    ax_base.grid(True, alpha=0.3)
    ax_base.axhline(y=0.01, color='green', linestyle='--', alpha=0.7, label='Tolerance')
    
    # Segment length errors plot
    ax_segments = fig.add_subplot(234)
    
    if states:
        final_state = states[-1]
        original_lengths = [seg.length for seg in animation_data['initial_chain'].segments]
        
        # Calculate new lengths (this is what forward iteration tries to achieve)
        try:
            import delta_robot.fabrik_forward as ff
            new_lengths = ff.FabrikForward.calculate_new_segment_lengths(states[1]['chain'])  # After first backward
        except:
            new_lengths = original_lengths
        
        errors, actual_lengths = calculate_segment_errors(final_state['chain'], new_lengths)
        
        x_pos = np.arange(len(original_lengths))
        width = 0.25
        
        ax_segments.bar(x_pos - width, original_lengths, width, label='Original', alpha=0.7, color='blue')
        ax_segments.bar(x_pos, new_lengths, width, label='Recalculated', alpha=0.7, color='orange')
        ax_segments.bar(x_pos + width, actual_lengths, width, label='Final', alpha=0.7, color='red')
        
        ax_segments.set_xlabel('Segment Index')
        ax_segments.set_ylabel('Length')
        ax_segments.set_title('Segment Length Analysis')
        ax_segments.set_xticks(x_pos)
        ax_segments.set_xticklabels([f'Seg {i}' for i in range(len(original_lengths))])
        ax_segments.legend()
        ax_segments.grid(True, alpha=0.3)
        
        # Add error annotations
        for i, error in enumerate(errors):
            ax_segments.text(i, max(actual_lengths) * 1.05, f'±{error:.1f}', 
                           ha='center', va='bottom', fontsize=8,
                           color='red' if error > 1 else 'green')
    
    # Cycle performance analysis
    ax_cycles = fig.add_subplot(235)
    
    if cycle_info:
        cycles_num = [info['cycle'] for info in cycle_info]
        backward_iters = [info['backward_iterations'] for info in cycle_info]
        forward_iters = [info['forward_iterations'] for info in cycle_info]
        
        ax_cycles.bar(cycles_num, backward_iters, label='Backward Iterations', alpha=0.7, color='orange')
        ax_cycles.bar(cycles_num, forward_iters, bottom=backward_iters, 
                     label='Forward Iterations', alpha=0.7, color='red')
        
        ax_cycles.set_xlabel('FABRIK Cycle')
        ax_cycles.set_ylabel('Iterations')
        ax_cycles.set_title('Iterations per Cycle')
        ax_cycles.legend()
        ax_cycles.grid(True, alpha=0.3)
    
    # Detailed statistics text
    ax_text = fig.add_subplot(236)
    ax_text.axis('off')
    
    # Calculate final statistics
    final_state = states[-1] if states else None
    total_cycles = len(cycle_info)
    total_backward_iters = sum(info['backward_iterations'] for info in cycle_info)
    total_forward_iters = sum(info['forward_iterations'] for info in cycle_info)
    converged = any(info['converged'] for info in cycle_info)
    
    stats_text = f"""FABRIK SOLVER ANALYSIS

TARGET:
Position: ({target.x}, {target.y}, {target.z})
Distance: {math.sqrt(target.x**2 + target.y**2 + target.z**2):.1f}
Max Reach: {max_reach:.1f}
Reachable: {'YES' if math.sqrt(target.x**2 + target.y**2 + target.z**2) <= max_reach else 'NO'}

ALGORITHM PERFORMANCE:
Total Cycles: {total_cycles}
Total Iterations: {total_backward_iters + total_forward_iters}
  - Backward: {total_backward_iters}
  - Forward: {total_forward_iters}
Converged: {'YES' if converged else 'NO'}

FINAL RESULT:"""

    if final_state:
        final_ee = final_state['end_effector']
        final_error = final_state['error']
        final_base_error = final_state['base_error']
        
        stats_text += f"""
End-Effector: ({final_ee.x:.2f}, {final_ee.y:.2f}, {final_ee.z:.2f})
Target Error: {final_error:.4f}
Base Error: {final_base_error:.4f}
Accuracy: {((1 - final_error/math.sqrt(target.x**2 + target.y**2 + target.z**2)) * 100):.1f}%

CONVERGENCE ANALYSIS:"""
        
        if cycle_info:
            best_error = min(info['forward_error'] for info in cycle_info)
            worst_error = max(info['forward_error'] for info in cycle_info)
            avg_error = sum(info['forward_error'] for info in cycle_info) / len(cycle_info)
            
            stats_text += f"""
Best Error: {best_error:.4f}
Worst Error: {worst_error:.4f}
Average Error: {avg_error:.4f}
Error Reduction: {((errors[0] - final_error) / errors[0] * 100):.1f}%

SEGMENT ANALYSIS:"""
            
            # Calculate segment statistics
            original_lengths = [seg.length for seg in animation_data['initial_chain'].segments]
            if final_state:
                try:
                    import delta_robot.fabrik_forward as ff
                    new_lengths = ff.FabrikForward.calculate_new_segment_lengths(states[1]['chain'])
                    errors, actual_lengths = calculate_segment_errors(final_state['chain'], new_lengths)
                    
                    max_segment_error = max(errors)
                    avg_segment_error = sum(errors) / len(errors)
                    
                    stats_text += f"""
Max Segment Error: {max_segment_error:.3f}
Avg Segment Error: {avg_segment_error:.3f}
Segments OK: {sum(1 for e in errors if e < 0.1)}/{len(errors)}"""
                except:
                    stats_text += "\nSegment analysis failed"
    
    stats_text += f"""

ISSUES & RECOMMENDATIONS:"""
    
    if not converged:
        stats_text += "\nWARNING: Algorithm did not converge"
        stats_text += "\n   - Try smaller tolerance"
        stats_text += "\n   - Increase max cycles"
        stats_text += "\n   - Check for unreachable target"
    
    if final_state and final_state['base_error'] > 0.1:
        stats_text += "\nWARNING: Base constraint violated"
        stats_text += "\n   - Forward iteration issues"
        stats_text += "\n   - Check segment length calculation"
    
    if final_state:
        try:
            import delta_robot.fabrik_forward as ff
            new_lengths = ff.FabrikForward.calculate_new_segment_lengths(states[1]['chain'])
            errors, _ = calculate_segment_errors(final_state['chain'], new_lengths)
            if max(errors) > 1.0:
                stats_text += "\nWARNING: Large segment length errors"
                stats_text += "\n   - Algorithm stability issues"
                stats_text += "\n   - Check constraint implementation"
        except:
            pass
    
    ax_text.text(0.05, 0.95, stats_text, transform=ax_text.transAxes, fontsize=9,
                verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round,pad=0.5', facecolor='lightblue', alpha=0.8))
    
    plt.tight_layout()
    plt.show()
    
    # Print console summary
    print(f"\n{'='*80}")
    print(f"FABRIK SOLVER VISUAL ANALYSIS COMPLETE")
    print(f"{'='*80}")
    print(f"Target: ({target.x}, {target.y}, {target.z})")
    print(f"Cycles: {total_cycles}, Total iterations: {total_backward_iters + total_forward_iters}")
    print(f"Converged: {'YES' if converged else 'NO'}")
    if final_state:
        print(f"Final error: {final_state['error']:.4f}")
        print(f"Base error: {final_state['base_error']:.4f}")
    print(f"{'='*80}")
    
    return True

def main():
    if len(sys.argv) == 2:
        target_x, target_y, target_z = parse_coordinates(sys.argv[1])
    else:
        target_x, target_y, target_z = 100, 100, 300
        print("Usage: python3 test_fabrik_solver_visual.py x,y,z")
    
    print(f"Creating comprehensive FABRIK solver visualization for target: ({target_x}, {target_y}, {target_z})")
    
    success = visualize_fabrik_solver_complete(target_x, target_y, target_z)
    
    if success:
        print("\nVisualization shows:")
        print("1. 3D convergence progression (all intermediate states)")
        print("2. Error convergence graph (target distance over time)")
        print("3. Base constraint enforcement (base should stay at origin)")
        print("4. Segment length preservation analysis")
        print("5. Per-cycle iteration count")
        print("6. Detailed statistics and issue analysis")
        print("\nTry different targets to test convergence:")
        print("  python3 test_fabrik_solver_visual.py 0,0,400    # Straight up")
        print("  python3 test_fabrik_solver_visual.py 200,0,200  # Far sideways")
        print("  python3 test_fabrik_solver_visual.py 50,50,100  # Easy target")
        print("  python3 test_fabrik_solver_visual.py 150,150,350 # Challenging")
    else:
        print("Visualization failed - check module installation")

if __name__ == "__main__":
    main()