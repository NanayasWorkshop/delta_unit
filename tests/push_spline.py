import numpy as np
import plotly.graph_objects as go
from scipy.interpolate import splprep, splev
import random

class ConicalSwarmSplineAvoider:
    def __init__(self, obstacles, original_points, spline_diameter=10):
        self.obstacles = []
        for obs in obstacles:
            self.obstacles.append({
                'center': np.array(obs['center']),
                'radius': obs['radius'] + spline_diameter/2  # Add safety buffer
            })
        
        self.spline_diameter = spline_diameter
        self.original_obstacles = obstacles
        self.original_points = np.array(original_points)
        self.num_points = len(original_points)
        
        # Calculate original cumulative distances
        self.original_cumulative_distances = self.calculate_cumulative_distances(original_points)
        
        # Swarm parameters
        self.num_swarms = 8  # Multiple swarms exploring different directions
        self.num_generations = 3
        self.iterations_per_generation = 60
        self.dt = 0.12
        
        # Force weights
        self.obstacle_avoidance_strength = 150.0
        self.path_following_strength = 32.0
        self.neighbor_cohesion_strength = 8.0
        self.velocity_damping = 0.93
        self.smoothing_strength = 4.0
        
        # Calculate original segment lengths
        self.original_segments = []
        for i in range(len(original_points) - 1):
            dist = np.linalg.norm(np.array(original_points[i+1]) - np.array(original_points[i]))
            self.original_segments.append(dist)
        
        # Calculate path length for cone scaling
        self.total_path_length = sum(self.original_segments)
        
        # Pre-calculate exploration directions (8 main directions + combinations)
        self.exploration_directions = [
            np.array([1, 0, 0]),    # +X
            np.array([-1, 0, 0]),   # -X
            np.array([0, 1, 0]),    # +Y
            np.array([0, -1, 0]),   # -Y
            np.array([0, 0, 1]),    # +Z
            np.array([0, 0, -1]),   # -Z
            np.array([1, 1, 0]) / np.sqrt(2),    # +X+Y diagonal
            np.array([1, -1, 0]) / np.sqrt(2),   # +X-Y diagonal
            np.array([-1, 1, 0]) / np.sqrt(2),   # -X+Y diagonal
            np.array([-1, -1, 0]) / np.sqrt(2),  # -X-Y diagonal
            np.array([1, 0, 1]) / np.sqrt(2),    # +X+Z diagonal
            np.array([1, 0, -1]) / np.sqrt(2),   # +X-Z diagonal
            np.array([-1, 0, 1]) / np.sqrt(2),   # -X+Z diagonal
            np.array([-1, 0, -1]) / np.sqrt(2),  # -X-Z diagonal
            np.array([0, 1, 1]) / np.sqrt(2),    # +Y+Z diagonal
            np.array([0, 1, -1]) / np.sqrt(2),   # +Y-Z diagonal
            np.array([0, -1, 1]) / np.sqrt(2),   # -Y+Z diagonal
            np.array([0, -1, -1]) / np.sqrt(2),  # -Y-Z diagonal
        ]

    def calculate_cumulative_distances(self, points):
        """Calculate cumulative distances along a path"""
        points = np.array(points)
        cumulative_distances = [0.0]
        
        for i in range(len(points) - 1):
            distance = np.linalg.norm(points[i+1] - points[i])
            cumulative_distances.append(cumulative_distances[-1] + distance)
        
        return cumulative_distances

    def sample_spline_at_distances(self, spline_points, target_distances):
        """Sample points on spline at specific cumulative distances"""
        if spline_points is None or len(spline_points) < 2:
            return None
            
        # Calculate cumulative distances along the spline
        spline_cumulative = [0.0]
        for i in range(len(spline_points) - 1):
            distance = np.linalg.norm(spline_points[i+1] - spline_points[i])
            spline_cumulative.append(spline_cumulative[-1] + distance)
        
        total_spline_length = spline_cumulative[-1]
        waypoints = []
        
        for target_distance in target_distances:
            if target_distance <= 0:
                waypoints.append(spline_points[0])
            elif target_distance >= total_spline_length:
                waypoints.append(spline_points[-1])
            else:
                # Find the segment containing this distance
                for i in range(len(spline_cumulative) - 1):
                    if spline_cumulative[i] <= target_distance <= spline_cumulative[i+1]:
                        # Interpolate between points i and i+1
                        segment_length = spline_cumulative[i+1] - spline_cumulative[i]
                        if segment_length > 0:
                            t = (target_distance - spline_cumulative[i]) / segment_length
                            interpolated_point = (1 - t) * spline_points[i] + t * spline_points[i+1]
                            waypoints.append(interpolated_point)
                        else:
                            waypoints.append(spline_points[i])
                        break
        
        return np.array(waypoints)
        
    def create_spline(self, points, num_samples=100):
        """Create smooth spline from control points"""
        points = np.array(points)
        try:
            tck, u = splprep([points[:, 0], points[:, 1], points[:, 2]], s=0, k=min(3, len(points)-1))
            u_new = np.linspace(0, 1, num_samples)
            spline_points = np.array(splev(u_new, tck)).T
            return spline_points, tck, u
        except:
            return None, None, None
    
    def check_collision(self, spline_points):
        """Check if spline collides with any obstacle"""
        if spline_points is None:
            return True, np.array([]), float('-inf')
            
        min_distance = float('inf')
        has_collision = False
        collision_points = []
        
        for obstacle in self.obstacles:
            distances = np.linalg.norm(spline_points - obstacle['center'], axis=1)
            collision_mask = distances < obstacle['radius']
            
            if np.any(collision_mask):
                has_collision = True
                collision_points.extend(spline_points[collision_mask])
            
            min_dist_to_obstacle = np.min(distances) - obstacle['radius']
            min_distance = min(min_distance, min_dist_to_obstacle)
        
        return has_collision, np.array(collision_points), min_distance
    
    def calculate_cone_radius(self, point_index):
        """Calculate how wide the exploration cone should be at this point"""
        if point_index == 0 or point_index == len(self.original_points) - 1:
            return 0  # No exploration at start/end
        
        # Distance along path (0 to 1)
        distance_so_far = sum(self.original_segments[:point_index])
        progress = distance_so_far / self.total_path_length
        
        # Cone shape: wide in middle, narrow at ends
        # Like a spindle: 0 -> wide -> 0
        cone_factor = 4 * progress * (1 - progress)  # Peaks at 0.5, goes to 0 at ends
        
        # Scale by proximity to obstacles
        obstacle_proximity_factor = 1.0
        point_pos = self.original_points[point_index]
        for obstacle in self.obstacles:
            distance_to_obstacle = np.linalg.norm(point_pos - obstacle['center'])
            if distance_to_obstacle < obstacle['radius'] * 3:
                # Closer to obstacle = wider exploration
                proximity = max(0, 1 - (distance_to_obstacle / (obstacle['radius'] * 3)))
                obstacle_proximity_factor = max(obstacle_proximity_factor, 1 + proximity * 2)
        
        # Base exploration radius (adjustable)
        base_radius = 80.0
        
        return base_radius * cone_factor * obstacle_proximity_factor
    
    def initialize_swarm(self, swarm_id, generation_num):
        """Initialize one swarm with specific exploration direction"""
        particles = self.original_points.copy()
        
        # Each swarm gets a different exploration bias
        primary_direction = self.exploration_directions[swarm_id % len(self.exploration_directions)]
        secondary_direction = self.exploration_directions[(swarm_id + 4) % len(self.exploration_directions)]
        
        print(f"  Swarm {swarm_id}: Exploring primarily in direction {primary_direction}")
        
        for i in range(1, len(particles) - 1):  # Skip start/end
            cone_radius = self.calculate_cone_radius(i)
            
            if cone_radius > 0:
                # Mix of directional bias and random exploration
                directional_offset = primary_direction * cone_radius * 0.6
                secondary_offset = secondary_direction * cone_radius * 0.3
                random_offset = np.array([
                    random.uniform(-1, 1),
                    random.uniform(-1, 1),
                    random.uniform(-1, 1)
                ]) * cone_radius * 0.3
                
                total_offset = directional_offset + secondary_offset + random_offset
                
                # Scale by generation (more aggressive exploration in later generations)
                generation_scale = 1.0 + generation_num * 0.4
                total_offset *= generation_scale
                
                # Apply obstacle avoidance during initialization
                for obstacle in self.obstacles:
                    to_particle = (particles[i] + total_offset) - obstacle['center']
                    distance = np.linalg.norm(to_particle)
                    
                    if distance < obstacle['radius'] * 1.5:
                        # Push away from obstacle
                        if distance > 1e-6:
                            push_direction = to_particle / distance
                            push_amount = obstacle['radius'] * 1.5 - distance + 20
                            total_offset += push_direction * push_amount
                
                particles[i] = particles[i] + total_offset
        
        return particles
    
    def calculate_obstacle_force(self, position):
        """Calculate repulsive force from obstacles"""
        total_force = np.zeros(3)
        
        for obstacle in self.obstacles:
            to_particle = position - obstacle['center']
            distance = np.linalg.norm(to_particle)
            
            influence_radius = obstacle['radius'] * 3.5
            
            if distance < influence_radius:
                if distance < 1e-6:
                    direction = np.array([1, 0, 0])
                else:
                    direction = to_particle / distance
                
                if distance < obstacle['radius']:
                    force_magnitude = self.obstacle_avoidance_strength * 20
                elif distance < obstacle['radius'] * 1.3:
                    force_magnitude = self.obstacle_avoidance_strength * 8
                elif distance < obstacle['radius'] * 2.0:
                    force_magnitude = self.obstacle_avoidance_strength * 3
                else:
                    force_magnitude = self.obstacle_avoidance_strength / (distance - obstacle['radius'] + 0.1)
                
                total_force += direction * force_magnitude
        
        return total_force
    
    def calculate_path_following_force(self, particle_idx, current_position, generation_num):
        """Gentle force toward original path - allows wide exploration"""
        if particle_idx == 0 or particle_idx == len(self.original_points) - 1:
            return np.zeros(3)
        
        original_pos = self.original_points[particle_idx]
        to_original = original_pos - current_position
        distance_from_original = np.linalg.norm(to_original)
        
        # Very weak force - only pulls back if very far from original
        if distance_from_original > 100:  # Only activate if more than 100 units away
            if distance_from_original > 0:
                direction = to_original / distance_from_original
                force_magnitude = self.path_following_strength * 0.3  # Very weak
                return direction * force_magnitude
        
        return np.zeros(3)
    
    def calculate_neighbor_cohesion_force(self, particle_idx, particles):
        """Maintain spacing between neighbors"""
        if particle_idx == 0 or particle_idx == len(particles) - 1:
            return np.zeros(3)
        
        total_force = np.zeros(3)
        
        # Previous neighbor
        if particle_idx > 0:
            to_prev = particles[particle_idx-1] - particles[particle_idx]
            current_dist = np.linalg.norm(to_prev)
            target_dist = self.original_segments[particle_idx-1]
            
            if current_dist > 0:
                dist_error = current_dist - target_dist
                # Allow ±15% tolerance for more flexibility
                if abs(dist_error) > target_dist * 0.15:
                    direction = to_prev / current_dist
                    force_magnitude = self.neighbor_cohesion_strength * (dist_error / target_dist) * 0.5
                    total_force += direction * force_magnitude
        
        # Next neighbor
        if particle_idx < len(particles) - 1:
            to_next = particles[particle_idx+1] - particles[particle_idx]
            current_dist = np.linalg.norm(to_next)
            target_dist = self.original_segments[particle_idx]
            
            if current_dist > 0:
                dist_error = current_dist - target_dist
                if abs(dist_error) > target_dist * 0.15:
                    direction = to_next / current_dist
                    force_magnitude = self.neighbor_cohesion_strength * (dist_error / target_dist) * 0.5
                    total_force += direction * force_magnitude
        
        return total_force
    
    def calculate_smoothing_force(self, particle_idx, particles):
        """Encourage smooth curves"""
        if particle_idx <= 0 or particle_idx >= len(particles) - 1:
            return np.zeros(3)
        
        prev_point = particles[particle_idx - 1]
        curr_point = particles[particle_idx]
        next_point = particles[particle_idx + 1]
        
        expected_position = (prev_point + next_point) / 2.0
        to_smooth = expected_position - curr_point
        
        return to_smooth * self.smoothing_strength * 0.3  # Gentle smoothing
    
    def evaluate_solution(self, particles):
        """Evaluate solution quality"""
        spline_points, _, _ = self.create_spline(particles)
        if spline_points is None:
            return float('inf')
        
        has_collision, _, min_distance = self.check_collision(spline_points)
        
        if has_collision:
            return 10000 + abs(min_distance) * 200
        
        # For collision-free solutions
        score = 0
        
        # Smoothness evaluation
        smoothness_penalty = 0
        for i in range(1, len(particles) - 1):
            prev_vec = particles[i] - particles[i-1]
            next_vec = particles[i+1] - particles[i]
            
            if np.linalg.norm(prev_vec) > 0 and np.linalg.norm(next_vec) > 0:
                prev_vec_norm = prev_vec / np.linalg.norm(prev_vec)
                next_vec_norm = next_vec / np.linalg.norm(next_vec)
                
                dot_product = np.clip(np.dot(prev_vec_norm, next_vec_norm), -1, 1)
                angle = np.arccos(dot_product)
                smoothness_penalty += angle
        
        # Path length penalty (prefer shorter paths)
        if spline_points is not None:
            path_length = 0
            for i in range(len(spline_points) - 1):
                path_length += np.linalg.norm(spline_points[i+1] - spline_points[i])
            
            original_spline, _, _ = self.create_spline(self.original_points)
            if original_spline is not None:
                original_length = 0
                for i in range(len(original_spline) - 1):
                    original_length += np.linalg.norm(original_spline[i+1] - original_spline[i])
                
                length_ratio = path_length / (original_length + 1e-6)
                length_penalty = abs(length_ratio - 1.0) * 10
            else:
                length_penalty = 0
        else:
            length_penalty = 0
        
        # Efficiency bonus (stay reasonably close to obstacles)
        efficiency_bonus = 0
        if spline_points is not None:
            min_distances = []
            for point in spline_points:
                min_dist = float('inf')
                for obstacle in self.obstacles:
                    dist = np.linalg.norm(point - obstacle['center']) - obstacle['radius']
                    min_dist = min(min_dist, dist)
                min_distances.append(max(0, min_dist))
            
            avg_distance = np.mean(min_distances)
            target_distance = 25
            if avg_distance > target_distance:
                efficiency_bonus = (avg_distance - target_distance) / 10.0
        
        score = smoothness_penalty * 2.0 + length_penalty + efficiency_bonus
        
        return score
    
    def run_swarm(self, swarm_id, generation_num, initial_particles):
        """Run one swarm optimization"""
        particles = initial_particles.copy()
        velocities = np.zeros_like(particles)
        
        best_particles = particles.copy()
        best_score = self.evaluate_solution(particles)
        
        for iteration in range(self.iterations_per_generation):
            forces = np.zeros_like(particles)
            
            for i in range(len(particles)):
                if i == 0 or i == len(particles) - 1:
                    continue
                
                obstacle_force = self.calculate_obstacle_force(particles[i])
                path_force = self.calculate_path_following_force(i, particles[i], generation_num)
                cohesion_force = self.calculate_neighbor_cohesion_force(i, particles)
                smoothing_force = self.calculate_smoothing_force(i, particles)
                
                # Small exploration noise
                exploration_noise = np.array([
                    random.uniform(-1, 1),
                    random.uniform(-1, 1),
                    random.uniform(-1, 1)
                ]) * 1.5 * (1.0 - iteration / self.iterations_per_generation)
                
                total_force = (obstacle_force + 
                             path_force * 0.2 + 
                             cohesion_force * 0.6 + 
                             smoothing_force * 0.4 + 
                             exploration_noise * 0.3)
                
                forces[i] = total_force
            
            # Update particles
            for i in range(1, len(particles) - 1):
                velocities[i] = velocities[i] * self.velocity_damping + forces[i] * self.dt
                
                max_velocity = 4.0
                if np.linalg.norm(velocities[i]) > max_velocity:
                    velocities[i] = velocities[i] / np.linalg.norm(velocities[i]) * max_velocity
                
                particles[i] = particles[i] + velocities[i] * self.dt
            
            # Evaluate
            current_score = self.evaluate_solution(particles)
            if current_score < best_score:
                best_score = current_score
                best_particles = particles.copy()
        
        return best_particles, best_score

    def refine_solution(self, best_particles):
        """Refine the best solution by pulling control points closer to original while maintaining collision avoidance"""
        print("\n=== REFINEMENT PHASE ===")
        print("Pulling control points closer to original positions...")
        
        refined_particles = best_particles.copy()
        refinement_iterations = 50
        step_size = 0.5  # Small steps to avoid jumping into obstacles
        
        for iteration in range(refinement_iterations):
            improved = False
            
            for i in range(1, len(refined_particles) - 1):  # Skip start/end points
                original_pos = self.original_points[i]
                current_pos = refined_particles[i]
                
                # Direction toward original position
                to_original = original_pos - current_pos
                distance_to_original = np.linalg.norm(to_original)
                
                if distance_to_original > 1.0:  # Only refine if reasonably far from original
                    # Try moving a small step toward original
                    direction = to_original / distance_to_original
                    test_position = current_pos + direction * step_size
                    
                    # Test this new position
                    test_particles = refined_particles.copy()
                    test_particles[i] = test_position
                    
                    # Check if spline with this test position is still collision-free
                    test_spline, _, _ = self.create_spline(test_particles)
                    if test_spline is not None:
                        has_collision, _, min_distance = self.check_collision(test_spline)
                        
                        if not has_collision and min_distance > 5.0:  # Safe margin
                            # Also check that we're not making the solution worse
                            current_score = self.evaluate_solution(test_particles)
                            original_score = self.evaluate_solution(refined_particles)
                            
                            if current_score <= original_score * 1.1:  # Allow slight score increase for better shape
                                refined_particles[i] = test_position
                                improved = True
            
            # Reduce step size over iterations for fine-tuning
            step_size *= 0.98
            
            if iteration % 10 == 0:
                total_displacement = 0
                for i in range(len(refined_particles)):
                    displacement = np.linalg.norm(refined_particles[i] - self.original_points[i])
                    total_displacement += displacement
                print(f"  Iteration {iteration}: Average displacement = {total_displacement/len(refined_particles):.2f}")
            
            # Early stopping if no improvements
            if not improved and iteration > 20:
                print(f"  Refinement converged at iteration {iteration}")
                break
        
        # Final evaluation
        refined_spline, _, _ = self.create_spline(refined_particles)
        if refined_spline is not None:
            has_collision, _, min_distance = self.check_collision(refined_spline)
            refined_score = self.evaluate_solution(refined_particles)
            original_score = self.evaluate_solution(best_particles)
            
            print(f"Refinement results:")
            print(f"  Still collision-free: {not has_collision}")
            print(f"  Min distance: {min_distance:.2f}")
            print(f"  Score change: {original_score:.2f} -> {refined_score:.2f}")
            
            # Calculate shape similarity improvement
            original_displacement = 0
            refined_displacement = 0
            for i in range(len(refined_particles)):
                original_displacement += np.linalg.norm(best_particles[i] - self.original_points[i])
                refined_displacement += np.linalg.norm(refined_particles[i] - self.original_points[i])
            
            print(f"  Average displacement: {original_displacement/len(refined_particles):.2f} -> {refined_displacement/len(refined_particles):.2f}")
            improvement = original_displacement - refined_displacement
            print(f"  Shape improvement: {improvement:.2f} units closer to original")
            
            if not has_collision and min_distance > 0:
                return refined_particles
            else:
                print("  Refinement caused collision - keeping original solution")
                return best_particles
        else:
            print("  Refinement failed - keeping original solution")
            return best_particles

    def solve(self):
        """Main conical swarm optimization with refinement"""
        print("Starting conical swarm optimization...")
        print(f"Running {self.num_swarms} swarms exploring in different directions")
        
        all_results = []
        overall_best_score = float('inf')
        overall_best_result = None
        
        for generation in range(self.num_generations):
            print(f"\n=== GENERATION {generation} ===")
            generation_results = []
            
            # Run multiple swarms in parallel exploration
            for swarm_id in range(self.num_swarms):
                print(f"\nSwarm {swarm_id} (Generation {generation}):")
                
                # Initialize this swarm
                initial_particles = self.initialize_swarm(swarm_id, generation)
                
                # Run swarm optimization
                best_particles, best_score = self.run_swarm(swarm_id, generation, initial_particles)
                
                # Evaluate final result
                spline_points, _, _ = self.create_spline(best_particles)
                has_collision, collision_points, min_distance = self.check_collision(spline_points)
                
                # Only generate waypoints for the best result (will be added later)
                waypoints = None
                
                result = {
                    'generation': generation,
                    'swarm_id': swarm_id,
                    'control_points': best_particles,
                    'spline_points': spline_points,
                    'waypoints': waypoints,
                    'score': best_score,
                    'has_collision': has_collision,
                    'min_distance': min_distance,
                    'collision_points': collision_points
                }
                
                generation_results.append(result)
                all_results.append(result)
                
                status = "CLEAR" if not has_collision else "COLLISION"
                print(f"  Result: {status}, Score={best_score:.2f}, Min_dist={min_distance:.2f}")
                
                # Update overall best
                if best_score < overall_best_score:
                    overall_best_score = best_score
                    overall_best_result = result
                    print(f"  *** NEW OVERALL BEST ***")
            
            # Check if we have good collision-free solutions
            collision_free = [r for r in generation_results if not r['has_collision']]
            print(f"Generation {generation} found {len(collision_free)} collision-free solutions!")
            
            # Continue to next generation to potentially find better solutions
            # (removed early termination to explore all generations)
        
        # Refine the best solution to be closer to original shape
        if overall_best_result is not None and not overall_best_result['has_collision']:
            refined_control_points = self.refine_solution(overall_best_result['control_points'])
            
            # Update the best result with refined control points
            refined_spline, _, _ = self.create_spline(refined_control_points)
            if refined_spline is not None:
                has_collision, collision_points, min_distance = self.check_collision(refined_spline)
                refined_score = self.evaluate_solution(refined_control_points)
                
                overall_best_result.update({
                    'control_points': refined_control_points,
                    'spline_points': refined_spline,
                    'score': refined_score,
                    'has_collision': has_collision,
                    'min_distance': min_distance,
                    'collision_points': collision_points
                })
        
        # Generate waypoints for the best result only
        if overall_best_result is not None and overall_best_result['spline_points'] is not None:
            overall_best_result['waypoints'] = self.sample_spline_at_distances(
                overall_best_result['spline_points'], 
                self.original_cumulative_distances
            )
        
        return overall_best_result, all_results

def visualize_results(avoider, best_result, all_results):
    """Visualize only the best result with waypoints"""
    fig = go.Figure()
    
    # Add obstacles
    colors = ['Reds', 'Blues', 'Greens', 'Oranges', 'Purples']
    for i, obstacle in enumerate(avoider.original_obstacles):
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        
        x_sphere = obstacle['radius'] * np.outer(np.cos(u), np.sin(v)) + obstacle['center'][0]
        y_sphere = obstacle['radius'] * np.outer(np.sin(u), np.sin(v)) + obstacle['center'][1]
        z_sphere = obstacle['radius'] * np.outer(np.ones(np.size(u)), np.cos(v)) + obstacle['center'][2]
        
        safety_radius = obstacle['radius'] + avoider.spline_diameter/2
        x_safety = safety_radius * np.outer(np.cos(u), np.sin(v)) + obstacle['center'][0]
        y_safety = safety_radius * np.outer(np.sin(u), np.sin(v)) + obstacle['center'][1]
        z_safety = safety_radius * np.outer(np.ones(np.size(u)), np.cos(v)) + obstacle['center'][2]
        
        color = colors[i % len(colors)]
        
        fig.add_trace(go.Surface(x=x_sphere, y=y_sphere, z=z_sphere, 
                                opacity=0.6, colorscale=color, showscale=False,
                                name=f'Obstacle {i+1}'))
        fig.add_trace(go.Surface(x=x_safety, y=y_safety, z=z_safety, 
                                opacity=0.2, colorscale=color, showscale=False,
                                name=f'Safety Zone {i+1}'))
    
    # Original path (straight lines)
    fig.add_trace(go.Scatter3d(x=avoider.original_points[:, 0], 
                              y=avoider.original_points[:, 1], 
                              z=avoider.original_points[:, 2],
                              mode='lines+markers', 
                              line=dict(color='blue', width=4, dash='dash'),
                              marker=dict(color='blue', size=6),
                              name='Original Path (Linear)'))
    
    # Best solution spline
    if best_result['spline_points'] is not None:
        fig.add_trace(go.Scatter3d(x=best_result['spline_points'][:, 0], 
                                  y=best_result['spline_points'][:, 1], 
                                  z=best_result['spline_points'][:, 2],
                                  mode='lines', line=dict(color='lime', width=6),
                                  name=f"Best Solution Spline (Score: {best_result['score']:.1f})"))
        
        # Best solution waypoints
        if best_result['waypoints'] is not None:
            fig.add_trace(go.Scatter3d(x=best_result['waypoints'][:, 0], 
                                      y=best_result['waypoints'][:, 1], 
                                      z=best_result['waypoints'][:, 2],
                                      mode='markers+lines', 
                                      marker=dict(color='red', size=10, symbol='diamond'),
                                      line=dict(color='red', width=3, dash='solid'),
                                      name="Distance-Preserved Waypoints"))
    
    collision_free_count = len([r for r in all_results if not r['has_collision']])
    
    fig.update_layout(
        title=f"Best Solution: {collision_free_count} Total Solutions Found",
        scene=dict(
            xaxis_title="X",
            yaxis_title="Y", 
            zaxis_title="Z",
            camera=dict(eye=dict(x=1.5, y=1.5, z=1.5))
        ),
        height=800,
        width=1200
    )
    
    fig.show()

# Main execution
if __name__ == "__main__":
    obstacles = [
        {'center': [123, 28, 500], 'radius': 60},
        {'center': [-30, -30, 370], 'radius': 45}
    ]
    
    original_points = [
        [0.00, 0.00, 0.00],
        [-28.27, -14.13, 138.91],
        [-82.79, -41.39, 271.58],
        [-116.18, -58.09, 410.51],
        [-72.06, -36.03, 526.33],
        [38.68, 19.34, 542.27],
        [106.96, 53.48, 442.65],
        [100.00, 50.00, 300.00]
    ]
    
    # Create and run conical swarm optimizer
    avoider = ConicalSwarmSplineAvoider(obstacles, original_points, spline_diameter=20)
    
    # Print original cumulative distances for reference
    print("Original cumulative distances:")
    for i, dist in enumerate(avoider.original_cumulative_distances):
        print(f"  Point {i}: {dist:.2f} units from start")
    
    # Check original
    original_spline, _, _ = avoider.create_spline(original_points)
    has_collision, _, min_distance = avoider.check_collision(original_spline)
    print(f"\nOriginal spline collision: {has_collision}, min distance: {min_distance:.2f}")
    
    # Solve
    best_result, all_results = avoider.solve()
    
    # Results
    print("\n" + "="*70)
    print("FINAL RESULTS:")
    print("="*70)
    collision_free_count = len([r for r in all_results if not r['has_collision']])
    print(f"Found {collision_free_count} collision-free solutions out of {len(all_results)} attempts")
    print(f"Best from Swarm {best_result['swarm_id']}, Generation {best_result['generation']}")
    print(f"Collision avoided: {not best_result['has_collision']}")
    print(f"Minimum distance to obstacle: {best_result['min_distance']:.2f}")
    print(f"Score: {best_result['score']:.2f}")
    
    print("\nFinal control points:")
    for i, point in enumerate(best_result['control_points']):
        original = original_points[i]
        movement = np.linalg.norm(point - np.array(original))
        print(f"  {i}: ({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f}) - moved {movement:.2f} units")
    
    print("\nWaypoints at original distances:")
    if best_result['waypoints'] is not None:
        for i, (waypoint, dist) in enumerate(zip(best_result['waypoints'], avoider.original_cumulative_distances)):
            print(f"  Waypoint {i}: ({waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f}) at distance {dist:.2f}")
            
            # Compare with original point
            original = np.array(original_points[i])
            displacement = np.linalg.norm(waypoint - original)
            print(f"    -> Displaced {displacement:.2f} units from original point")
    else:
        print("  No waypoints generated (spline creation failed)")
    
    # Calculate distance preservation accuracy
    if best_result['waypoints'] is not None and len(best_result['waypoints']) > 1:
        print("\nDistance preservation check:")
        waypoint_distances = avoider.calculate_cumulative_distances(best_result['waypoints'])
        for i, (original_dist, waypoint_dist) in enumerate(zip(avoider.original_cumulative_distances, waypoint_distances)):
            error = abs(waypoint_dist - original_dist)
            error_percent = (error / (original_dist + 1e-6)) * 100
            print(f"  Segment {i}: Original={original_dist:.2f}, Waypoint={waypoint_dist:.2f}, Error={error:.2f} ({error_percent:.1f}%)")
    
    # Visualize
    visualize_results(avoider, best_result, all_results)