import numpy as np
from scipy.interpolate import splprep, splev
import random
import time

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
        self.num_swarms = 8
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
        
        # Pre-calculate exploration directions
        self.exploration_directions = [
            np.array([1, 0, 0]), np.array([-1, 0, 0]), np.array([0, 1, 0]), np.array([0, -1, 0]),
            np.array([0, 0, 1]), np.array([0, 0, -1]), np.array([1, 1, 0]) / np.sqrt(2),
            np.array([1, -1, 0]) / np.sqrt(2), np.array([-1, 1, 0]) / np.sqrt(2),
            np.array([-1, -1, 0]) / np.sqrt(2), np.array([1, 0, 1]) / np.sqrt(2),
            np.array([1, 0, -1]) / np.sqrt(2), np.array([-1, 0, 1]) / np.sqrt(2),
            np.array([-1, 0, -1]) / np.sqrt(2), np.array([0, 1, 1]) / np.sqrt(2),
            np.array([0, 1, -1]) / np.sqrt(2), np.array([0, -1, 1]) / np.sqrt(2),
            np.array([0, -1, -1]) / np.sqrt(2)
        ]

    def calculate_cumulative_distances(self, points):
        points = np.array(points)
        cumulative_distances = [0.0]
        
        for i in range(len(points) - 1):
            distance = np.linalg.norm(points[i+1] - points[i])
            cumulative_distances.append(cumulative_distances[-1] + distance)
        
        return cumulative_distances

    def sample_spline_at_distances(self, spline_points, target_distances):
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
                for i in range(len(spline_cumulative) - 1):
                    if spline_cumulative[i] <= target_distance <= spline_cumulative[i+1]:
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
        points = np.array(points)
        try:
            tck, u = splprep([points[:, 0], points[:, 1], points[:, 2]], s=0, k=min(3, len(points)-1))
            u_new = np.linspace(0, 1, num_samples)
            spline_points = np.array(splev(u_new, tck)).T
            return spline_points, tck, u
        except:
            return None, None, None
    
    def check_collision(self, spline_points):
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
        if point_index == 0 or point_index == len(self.original_points) - 1:
            return 0
        
        distance_so_far = sum(self.original_segments[:point_index])
        progress = distance_so_far / self.total_path_length
        cone_factor = 4 * progress * (1 - progress)
        
        obstacle_proximity_factor = 1.0
        point_pos = self.original_points[point_index]
        for obstacle in self.obstacles:
            distance_to_obstacle = np.linalg.norm(point_pos - obstacle['center'])
            if distance_to_obstacle < obstacle['radius'] * 3:
                proximity = max(0, 1 - (distance_to_obstacle / (obstacle['radius'] * 3)))
                obstacle_proximity_factor = max(obstacle_proximity_factor, 1 + proximity * 2)
        
        base_radius = 80.0
        return base_radius * cone_factor * obstacle_proximity_factor
    
    def initialize_swarm(self, swarm_id, generation_num):
        particles = self.original_points.copy()
        primary_direction = self.exploration_directions[swarm_id % len(self.exploration_directions)]
        secondary_direction = self.exploration_directions[(swarm_id + 4) % len(self.exploration_directions)]
        
        for i in range(1, len(particles) - 1):
            cone_radius = self.calculate_cone_radius(i)
            
            if cone_radius > 0:
                directional_offset = primary_direction * cone_radius * 0.6
                secondary_offset = secondary_direction * cone_radius * 0.3
                random_offset = np.array([
                    random.uniform(-1, 1),
                    random.uniform(-1, 1),
                    random.uniform(-1, 1)
                ]) * cone_radius * 0.3
                
                total_offset = directional_offset + secondary_offset + random_offset
                generation_scale = 1.0 + generation_num * 0.4
                total_offset *= generation_scale
                
                for obstacle in self.obstacles:
                    to_particle = (particles[i] + total_offset) - obstacle['center']
                    distance = np.linalg.norm(to_particle)
                    
                    if distance < obstacle['radius'] * 1.5:
                        if distance > 1e-6:
                            push_direction = to_particle / distance
                            push_amount = obstacle['radius'] * 1.5 - distance + 20
                            total_offset += push_direction * push_amount
                
                particles[i] = particles[i] + total_offset
        
        return particles
    
    def calculate_obstacle_force(self, position):
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
        if particle_idx == 0 or particle_idx == len(self.original_points) - 1:
            return np.zeros(3)
        
        original_pos = self.original_points[particle_idx]
        to_original = original_pos - current_position
        distance_from_original = np.linalg.norm(to_original)
        
        if distance_from_original > 100:
            if distance_from_original > 0:
                direction = to_original / distance_from_original
                force_magnitude = self.path_following_strength * 0.3
                return direction * force_magnitude
        
        return np.zeros(3)
    
    def calculate_neighbor_cohesion_force(self, particle_idx, particles):
        if particle_idx == 0 or particle_idx == len(particles) - 1:
            return np.zeros(3)
        
        total_force = np.zeros(3)
        
        if particle_idx > 0:
            to_prev = particles[particle_idx-1] - particles[particle_idx]
            current_dist = np.linalg.norm(to_prev)
            target_dist = self.original_segments[particle_idx-1]
            
            if current_dist > 0:
                dist_error = current_dist - target_dist
                if abs(dist_error) > target_dist * 0.15:
                    direction = to_prev / current_dist
                    force_magnitude = self.neighbor_cohesion_strength * (dist_error / target_dist) * 0.5
                    total_force += direction * force_magnitude
        
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
        if particle_idx <= 0 or particle_idx >= len(particles) - 1:
            return np.zeros(3)
        
        prev_point = particles[particle_idx - 1]
        curr_point = particles[particle_idx]
        next_point = particles[particle_idx + 1]
        
        expected_position = (prev_point + next_point) / 2.0
        to_smooth = expected_position - curr_point
        
        return to_smooth * self.smoothing_strength * 0.3
    
    def evaluate_solution(self, particles):
        spline_points, _, _ = self.create_spline(particles)
        if spline_points is None:
            return float('inf')
        
        has_collision, _, min_distance = self.check_collision(spline_points)
        
        if has_collision:
            return 10000 + abs(min_distance) * 200
        
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
        
        # Path length penalty
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
        
        # Efficiency bonus
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
            
            for i in range(1, len(particles) - 1):
                velocities[i] = velocities[i] * self.velocity_damping + forces[i] * self.dt
                
                max_velocity = 4.0
                if np.linalg.norm(velocities[i]) > max_velocity:
                    velocities[i] = velocities[i] / np.linalg.norm(velocities[i]) * max_velocity
                
                particles[i] = particles[i] + velocities[i] * self.dt
            
            current_score = self.evaluate_solution(particles)
            if current_score < best_score:
                best_score = current_score
                best_particles = particles.copy()
        
        return best_particles, best_score

    def refine_solution(self, best_particles):
        refined_particles = best_particles.copy()
        refinement_iterations = 50
        step_size = 0.5
        
        for iteration in range(refinement_iterations):
            improved = False
            
            for i in range(1, len(refined_particles) - 1):
                original_pos = self.original_points[i]
                current_pos = refined_particles[i]
                
                to_original = original_pos - current_pos
                distance_to_original = np.linalg.norm(to_original)
                
                if distance_to_original > 1.0:
                    direction = to_original / distance_to_original
                    test_position = current_pos + direction * step_size
                    
                    test_particles = refined_particles.copy()
                    test_particles[i] = test_position
                    
                    test_spline, _, _ = self.create_spline(test_particles)
                    if test_spline is not None:
                        has_collision, _, min_distance = self.check_collision(test_spline)
                        
                        if not has_collision and min_distance > 5.0:
                            current_score = self.evaluate_solution(test_particles)
                            original_score = self.evaluate_solution(refined_particles)
                            
                            if current_score <= original_score * 1.1:
                                refined_particles[i] = test_position
                                improved = True
            
            step_size *= 0.98
            
            if not improved and iteration > 20:
                break
        
        refined_spline, _, _ = self.create_spline(refined_particles)
        if refined_spline is not None:
            has_collision, _, min_distance = self.check_collision(refined_spline)
            
            if not has_collision and min_distance > 0:
                return refined_particles
            else:
                return best_particles
        else:
            return best_particles

    def solve(self, verbose=False):
        start_time = time.time()
        
        if verbose:
            print("Starting conical swarm optimization...")
        
        all_results = []
        overall_best_score = float('inf')
        overall_best_result = None
        
        for generation in range(self.num_generations):
            if verbose:
                print(f"Generation {generation}")
            
            for swarm_id in range(self.num_swarms):
                initial_particles = self.initialize_swarm(swarm_id, generation)
                best_particles, best_score = self.run_swarm(swarm_id, generation, initial_particles)
                
                spline_points, _, _ = self.create_spline(best_particles)
                has_collision, collision_points, min_distance = self.check_collision(spline_points)
                
                result = {
                    'generation': generation,
                    'swarm_id': swarm_id,
                    'control_points': best_particles,
                    'spline_points': spline_points,
                    'waypoints': None,
                    'score': best_score,
                    'has_collision': has_collision,
                    'min_distance': min_distance,
                    'collision_points': collision_points
                }
                
                all_results.append(result)
                
                if best_score < overall_best_score:
                    overall_best_score = best_score
                    overall_best_result = result
        
        # Refine best solution
        if overall_best_result is not None and not overall_best_result['has_collision']:
            if verbose:
                print("Refining best solution...")
            refined_control_points = self.refine_solution(overall_best_result['control_points'])
            
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
        
        # Generate waypoints for best result
        if overall_best_result is not None and overall_best_result['spline_points'] is not None:
            overall_best_result['waypoints'] = self.sample_spline_at_distances(
                overall_best_result['spline_points'], 
                self.original_cumulative_distances
            )
        
        total_time = time.time() - start_time
        overall_best_result['computation_time'] = total_time
        
        if verbose:
            print(f"Total computation time: {total_time:.2f} seconds")
        
        return overall_best_result, all_results

# Example usage
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
    
    # Create and solve
    avoider = ConicalSwarmSplineAvoider(obstacles, original_points, spline_diameter=20)
    best_result, all_results = avoider.solve(verbose=True)
    
    # Print results
    collision_free_count = len([r for r in all_results if not r['has_collision']])
    print(f"\nFound {collision_free_count} collision-free solutions")
    print(f"Best score: {best_result['score']:.2f}")
    print(f"Min distance: {best_result['min_distance']:.2f}")
    print(f"Collision avoided: {not best_result['has_collision']}")
    print(f"Total computation time: {best_result['computation_time']:.2f} seconds")