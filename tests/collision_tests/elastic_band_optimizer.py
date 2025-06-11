import numpy as np
import plotly.graph_objects as go
from scipy.interpolate import splprep, splev
import time

class ElasticBandOptimizer:
    def __init__(self, obstacles, original_points, spline_diameter=20):
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
        
        # Calculate original cumulative distances and segment lengths
        self.original_cumulative_distances = self.calculate_cumulative_distances(original_points)
        self.original_segments = []
        for i in range(len(original_points) - 1):
            dist = np.linalg.norm(np.array(original_points[i+1]) - np.array(original_points[i]))
            self.original_segments.append(dist)
        
        # Elastic band parameters (tunable)
        self.max_iterations = 200
        self.dt = 0.1
        self.convergence_threshold = 0.01
        
        # Force weights - these are the main tuning parameters
        self.obstacle_repulsion_strength = 200.0    # How hard to push away from obstacles
        self.spring_stiffness = 50.0                # How rigid the segments
        self.bending_resistance = 15.0              # How much it resists bending
        self.target_attraction_strength = 30.0      # Pull end-effector to target
        self.damping = 0.85                         # Prevents oscillation (energy loss)
        
        # 3D bending constraints - MINIMUM BENDING RADIUS 80mm
        self.min_bending_radius = 80.0  # 80mm minimum bending radius
        self.max_curvature = 1.0 / self.min_bending_radius  # Maximum curvature = 1/80 = 0.0125
        
        # Segment length constraints (±3% as mentioned)
        self.length_tolerance = 0.03  # 3% tolerance
        
    def calculate_cumulative_distances(self, points):
        """Calculate cumulative distances along a path"""
        points = np.array(points)
        cumulative_distances = [0.0]
        
        for i in range(len(points) - 1):
            distance = np.linalg.norm(points[i+1] - points[i])
            cumulative_distances.append(cumulative_distances[-1] + distance)
        
        return cumulative_distances

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

    def calculate_obstacle_force(self, position):
        """Calculate repulsive force from obstacles (3D)"""
        total_force = np.zeros(3)
        
        for obstacle in self.obstacles:
            to_point = position - obstacle['center']
            distance = np.linalg.norm(to_point)
            
            # Influence radius - force drops with distance
            influence_radius = obstacle['radius'] * 3.0
            
            if distance < influence_radius:
                if distance < 1e-6:
                    direction = np.array([1, 0, 0])  # Default direction
                else:
                    direction = to_point / distance
                
                # Force magnitude increases rapidly as we get closer
                if distance < obstacle['radius']:
                    # Inside obstacle - very strong repulsion
                    force_magnitude = self.obstacle_repulsion_strength * 10.0
                elif distance < obstacle['radius'] * 1.2:
                    # Very close - strong repulsion
                    force_magnitude = self.obstacle_repulsion_strength * 5.0
                else:
                    # Moderate distance - inverse square law
                    safe_distance = distance - obstacle['radius']
                    force_magnitude = self.obstacle_repulsion_strength / (safe_distance + 0.1)
                
                total_force += direction * force_magnitude
        
        return total_force

    def calculate_spring_force(self, i, points):
        """Calculate spring forces to maintain segment lengths"""
        total_force = np.zeros(3)
        
        # Force from previous point (if exists)
        if i > 0:
            to_prev = points[i-1] - points[i]
            current_dist = np.linalg.norm(to_prev)
            target_dist = self.original_segments[i-1]
            
            if current_dist > 1e-6:
                direction = to_prev / current_dist
                # Spring force proportional to length error
                length_error = current_dist - target_dist
                force_magnitude = self.spring_stiffness * length_error
                total_force += direction * force_magnitude
        
        # Force from next point (if exists)
        if i < len(points) - 1:
            to_next = points[i+1] - points[i]
            current_dist = np.linalg.norm(to_next)
            target_dist = self.original_segments[i]
            
            if current_dist > 1e-6:
                direction = to_next / current_dist
                length_error = current_dist - target_dist
                force_magnitude = self.spring_stiffness * length_error
                total_force += direction * force_magnitude
        
        return total_force

    def calculate_bending_force_3d(self, i, points):
        """Calculate 3D bending resistance force"""
        if i <= 0 or i >= len(points) - 1:
            return np.zeros(3)
        
        p1 = points[i-1]
        p2 = points[i]
        p3 = points[i+1]
        
        # Calculate curvature vector
        v1 = p2 - p1
        v2 = p3 - p2
        
        v1_norm = np.linalg.norm(v1)
        v2_norm = np.linalg.norm(v2)
        
        if v1_norm < 1e-6 or v2_norm < 1e-6:
            return np.zeros(3)
        
        v1_unit = v1 / v1_norm
        v2_unit = v2 / v2_norm
        
        # Curvature vector points toward center of curvature
        curvature_vector = v2_unit - v1_unit
        curvature_magnitude = np.linalg.norm(curvature_vector)
        
        # If curvature exceeds limit, apply resistance force
        if curvature_magnitude > self.max_curvature:
            if curvature_magnitude > 1e-6:
                # Force opposes excessive curvature
                resistance_direction = -curvature_vector / curvature_magnitude
                excess_curvature = curvature_magnitude - self.max_curvature
                force_magnitude = self.bending_resistance * excess_curvature * 100
                return resistance_direction * force_magnitude
        
        return np.zeros(3)

    def calculate_target_attraction(self, position, target):
        """Calculate attraction force toward target (for end-effector only)"""
        to_target = target - position
        distance = np.linalg.norm(to_target)
        
        if distance < 1e-6:
            return np.zeros(3)
        
        direction = to_target / distance
        # Linear attraction (could be quadratic for stronger pull)
        force_magnitude = self.target_attraction_strength * min(distance / 100.0, 1.0)
        
        return direction * force_magnitude

    def enforce_length_constraints(self, points):
        """Enforce segment length constraints (±3%)"""
        constrained_points = points.copy()
        
        # Iterative constraint enforcement
        for iteration in range(5):  # Multiple passes for better convergence
            for i in range(len(constrained_points) - 1):
                current_length = np.linalg.norm(constrained_points[i+1] - constrained_points[i])
                target_length = self.original_segments[i]
                
                min_length = target_length * (1 - self.length_tolerance)
                max_length = target_length * (1 + self.length_tolerance)
                
                if current_length < min_length or current_length > max_length:
                    # Clamp length to valid range
                    if current_length > 1e-6:
                        direction = (constrained_points[i+1] - constrained_points[i]) / current_length
                        if current_length < min_length:
                            new_length = min_length
                        else:
                            new_length = max_length
                        
                        # Move the second point to satisfy constraint
                        constrained_points[i+1] = constrained_points[i] + direction * new_length
        
        return constrained_points

    def solve(self, target_position=None):
        """Main elastic band optimization"""
        start_time = time.time()
        
        print("Starting Elastic Band optimization...")
        print(f"Max iterations: {self.max_iterations}")
        print(f"Force parameters: Obstacle={self.obstacle_repulsion_strength}, Spring={self.spring_stiffness}, Bending={self.bending_resistance}")
        
        # Initialize with original points
        points = self.original_points.copy()
        velocities = np.zeros_like(points)
        
        # If no target specified, use the last point
        if target_position is None:
            target_position = points[-1].copy()
        
        iteration_scores = []
        
        for iteration in range(self.max_iterations):
            forces = np.zeros_like(points)
            
            # Calculate forces for each point
            for i in range(len(points)):
                # Skip fixed endpoints (first and last points)
                if i == 0:
                    continue  # Base is fixed
                if i == len(points) - 1:
                    # End-effector: only target attraction + obstacle avoidance
                    forces[i] += self.calculate_obstacle_force(points[i])
                    forces[i] += self.calculate_target_attraction(points[i], target_position)
                    continue
                
                # Middle points: all forces
                obstacle_force = self.calculate_obstacle_force(points[i])
                spring_force = self.calculate_spring_force(i, points)
                bending_force = self.calculate_bending_force_3d(i, points)
                
                forces[i] = obstacle_force + spring_force + bending_force
            
            # Update velocities and positions (skip fixed points)
            max_force = 0
            for i in range(1, len(points)):  # Skip base point (i=0 is fixed)
                velocities[i] = velocities[i] * self.damping + forces[i] * self.dt
                
                # Velocity limiting for stability
                max_velocity = 5.0
                if np.linalg.norm(velocities[i]) > max_velocity:
                    velocities[i] = velocities[i] / np.linalg.norm(velocities[i]) * max_velocity
                
                points[i] += velocities[i] * self.dt
                max_force = max(max_force, np.linalg.norm(forces[i]))
            
            # Enforce segment length constraints
            points = self.enforce_length_constraints(points)
            
            # Evaluate current solution
            spline_points, _, _ = self.create_spline(points)
            has_collision, _, min_distance = self.check_collision(spline_points)
            
            # Calculate score (similar to swarm)
            if has_collision:
                score = 10000 + abs(min_distance) * 200
            else:
                # Smoothness score
                smoothness_penalty = 0
                for j in range(1, len(points) - 1):
                    prev_vec = points[j] - points[j-1]
                    next_vec = points[j+1] - points[j]
                    
                    if np.linalg.norm(prev_vec) > 0 and np.linalg.norm(next_vec) > 0:
                        prev_vec_norm = prev_vec / np.linalg.norm(prev_vec)
                        next_vec_norm = next_vec / np.linalg.norm(next_vec)
                        
                        dot_product = np.clip(np.dot(prev_vec_norm, next_vec_norm), -1, 1)
                        angle = np.arccos(dot_product)
                        smoothness_penalty += angle
                
                score = smoothness_penalty * 2.0
            
            iteration_scores.append(score)
            
            # Progress reporting
            if iteration % 20 == 0:
                status = "COLLISION" if has_collision else "CLEAR"
                print(f"  Iteration {iteration}: {status}, Score={score:.2f}, Min_dist={min_distance:.2f}, Max_force={max_force:.2f}")
            
            # Convergence check
            if iteration > 10:
                recent_scores = iteration_scores[-10:]
                score_change = max(recent_scores) - min(recent_scores)
                if score_change < self.convergence_threshold and max_force < 1.0:
                    print(f"  Converged at iteration {iteration}")
                    break
        
        # Final evaluation
        final_spline, _, _ = self.create_spline(points)
        has_collision, collision_points, min_distance = self.check_collision(final_spline)
        final_score = iteration_scores[-1] if iteration_scores else float('inf')
        
        # Generate waypoints
        waypoints = None
        if final_spline is not None:
            waypoints = self.sample_spline_at_distances(final_spline, self.original_cumulative_distances)
        
        computation_time = time.time() - start_time
        
        result = {
            'method': 'Elastic Band',
            'control_points': points,
            'spline_points': final_spline,
            'waypoints': waypoints,
            'score': final_score,
            'has_collision': has_collision,
            'min_distance': min_distance,
            'collision_points': collision_points,
            'iterations_used': iteration + 1,
            'computation_time': computation_time,
            'converged': iteration < self.max_iterations - 1
        }
        
        return result

def visualize_results(avoider, result):
    """Visualize the elastic band result"""
    fig = go.Figure()
    
    # Add obstacles
    colors = ['Reds', 'Blues', 'Greens']
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
    
    # Original path
    fig.add_trace(go.Scatter3d(x=avoider.original_points[:, 0], 
                              y=avoider.original_points[:, 1], 
                              z=avoider.original_points[:, 2],
                              mode='lines+markers', 
                              line=dict(color='blue', width=4, dash='dash'),
                              marker=dict(color='blue', size=6),
                              name='Original Path'))
    
    # Elastic band solution
    if result['spline_points'] is not None:
        fig.add_trace(go.Scatter3d(x=result['spline_points'][:, 0], 
                                  y=result['spline_points'][:, 1], 
                                  z=result['spline_points'][:, 2],
                                  mode='lines', line=dict(color='lime', width=6),
                                  name=f"Elastic Band Solution (Score: {result['score']:.1f})"))
        
        if result['waypoints'] is not None:
            fig.add_trace(go.Scatter3d(x=result['waypoints'][:, 0], 
                                      y=result['waypoints'][:, 1], 
                                      z=result['waypoints'][:, 2],
                                      mode='markers+lines', 
                                      marker=dict(color='red', size=10, symbol='diamond'),
                                      line=dict(color='red', width=3),
                                      name="Waypoints"))
    
    # Control points (show the actual elastic band)
    fig.add_trace(go.Scatter3d(x=result['control_points'][:, 0], 
                              y=result['control_points'][:, 1], 
                              z=result['control_points'][:, 2],
                              mode='markers', 
                              marker=dict(color='orange', size=8, symbol='circle'),
                              name="Control Points"))
    
    fig.update_layout(
        title=f"Elastic Band Optimization - {result['method']}",
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
    
    # Create and run elastic band optimizer
    optimizer = ElasticBandOptimizer(obstacles, original_points, spline_diameter=20)
    
    # Print original status
    print("Original cumulative distances:")
    for i, dist in enumerate(optimizer.original_cumulative_distances):
        print(f"  Point {i}: {dist:.2f} units from start")
    
    original_spline, _, _ = optimizer.create_spline(original_points)
    has_collision, _, min_distance = optimizer.check_collision(original_spline)
    print(f"\nOriginal spline collision: {has_collision}, min distance: {min_distance:.2f}")
    
    # Solve
    result = optimizer.solve()
    
    # Results
    print("\n" + "="*70)
    print("ELASTIC BAND RESULTS:")
    print("="*70)
    print(f"Method: {result['method']}")
    print(f"Converged: {result['converged']} (in {result['iterations_used']} iterations)")
    print(f"Computation time: {result['computation_time']:.3f} seconds")
    print(f"Collision avoided: {not result['has_collision']}")
    print(f"Minimum distance to obstacle: {result['min_distance']:.2f}")
    print(f"Final score: {result['score']:.2f}")
    
    print("\nFinal control points:")
    for i, point in enumerate(result['control_points']):
        original = original_points[i]
        movement = np.linalg.norm(point - np.array(original))
        print(f"  {i}: ({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f}) - moved {movement:.2f} units")
    
    # Distance preservation check
    if result['waypoints'] is not None and len(result['waypoints']) > 1:
        print("\nDistance preservation check:")
        waypoint_distances = optimizer.calculate_cumulative_distances(result['waypoints'])
        for i, (original_dist, waypoint_dist) in enumerate(zip(optimizer.original_cumulative_distances, waypoint_distances)):
            error = abs(waypoint_dist - original_dist)
            error_percent = (error / (original_dist + 1e-6)) * 100
            print(f"  Segment {i}: Original={original_dist:.2f}, Waypoint={waypoint_dist:.2f}, Error={error:.2f} ({error_percent:.1f}%)")
    
    # Visualize
    visualize_results(optimizer, result)