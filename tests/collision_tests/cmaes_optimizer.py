import numpy as np
import plotly.graph_objects as go
from scipy.interpolate import splprep, splev
import time
import warnings
warnings.filterwarnings('ignore')

class CMAESOptimizer:
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
        
        # CMA-ES parameters
        self.max_evaluations = 10000  # Maximum function evaluations
        self.initial_sigma = 30.0      # Initial step size (search radius)
        self.population_size = None    # Auto-determine based on dimension
        
        # Constraint parameters - MINIMUM BENDING RADIUS 80mm
        self.length_tolerance = 0.03  # ±3% segment length tolerance
        self.min_bending_radius = 80.0  # 80mm minimum bending radius
        self.max_curvature = 1.0 / self.min_bending_radius  # Maximum curvature = 1/80 = 0.0125
        
        # Penalty weights for constraint violations
        self.collision_penalty = 10000
        self.length_penalty_weight = 1000
        self.curvature_penalty_weight = 500
        
        # Track function evaluations
        self.evaluation_count = 0
        self.best_score_history = []
        self.generation_count = 0

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

    def points_to_vector(self, points):
        """Convert 3D points to flat vector for optimization (skip first and last)"""
        # Only optimize middle points (indices 1 to n-2)
        middle_points = points[1:-1]
        return middle_points.flatten()
    
    def vector_to_points(self, vector):
        """Convert flat vector back to 3D points"""
        # Reshape middle points
        middle_points = vector.reshape(-1, 3)
        
        # Combine with fixed first and last points
        points = np.zeros((self.num_points, 3))
        points[0] = self.original_points[0]  # Fixed start
        points[1:-1] = middle_points        # Optimized middle
        points[-1] = self.original_points[-1]  # Fixed end
        
        return points

    def calculate_segment_length_penalty(self, points):
        """Calculate penalty for violating segment length constraints"""
        penalty = 0.0
        
        for i in range(len(points) - 1):
            current_length = np.linalg.norm(points[i+1] - points[i])
            target_length = self.original_segments[i]
            
            min_length = target_length * (1 - self.length_tolerance)
            max_length = target_length * (1 + self.length_tolerance)
            
            if current_length < min_length:
                violation = min_length - current_length
                penalty += violation * self.length_penalty_weight
            elif current_length > max_length:
                violation = current_length - max_length
                penalty += violation * self.length_penalty_weight
        
        return penalty

    def calculate_curvature_penalty(self, points):
        """Calculate penalty for violating bending radius constraints"""
        penalty = 0.0
        
        for i in range(1, len(points) - 1):
            p1 = points[i-1]
            p2 = points[i]
            p3 = points[i+1]
            
            # Calculate curvature
            v1 = p2 - p1
            v2 = p3 - p2
            
            v1_norm = np.linalg.norm(v1)
            v2_norm = np.linalg.norm(v2)
            
            if v1_norm > 1e-6 and v2_norm > 1e-6:
                v1_unit = v1 / v1_norm
                v2_unit = v2 / v2_norm
                
                # Curvature magnitude
                curvature_vector = v2_unit - v1_unit
                curvature_magnitude = np.linalg.norm(curvature_vector)
                
                if curvature_magnitude > self.max_curvature:
                    violation = curvature_magnitude - self.max_curvature
                    penalty += violation * self.curvature_penalty_weight
        
        return penalty

    def objective_function(self, vector):
        """Objective function for CMA-ES"""
        self.evaluation_count += 1
        
        # Convert vector to points
        points = self.vector_to_points(vector)
        
        # Create spline and check collision
        spline_points, _, _ = self.create_spline(points)
        if spline_points is None:
            return 1e6  # Invalid spline
        
        has_collision, _, min_distance = self.check_collision(spline_points)
        
        # Base score calculation
        if has_collision:
            score = self.collision_penalty + abs(min_distance) * 200
        else:
            # Smoothness evaluation
            smoothness_penalty = 0
            for i in range(1, len(points) - 1):
                prev_vec = points[i] - points[i-1]
                next_vec = points[i+1] - points[i]
                
                if np.linalg.norm(prev_vec) > 0 and np.linalg.norm(next_vec) > 0:
                    prev_vec_norm = prev_vec / np.linalg.norm(prev_vec)
                    next_vec_norm = next_vec / np.linalg.norm(next_vec)
                    
                    dot_product = np.clip(np.dot(prev_vec_norm, next_vec_norm), -1, 1)
                    angle = np.arccos(dot_product)
                    smoothness_penalty += angle
            
            score = smoothness_penalty * 2.0
        
        # Add constraint penalties
        score += self.calculate_segment_length_penalty(points)
        score += self.calculate_curvature_penalty(points)
        
        # Track best score
        if len(self.best_score_history) == 0 or score < min(self.best_score_history):
            self.best_score_history.append(score)
            if self.evaluation_count % 100 == 0 or len(self.best_score_history) < 10:
                status = "COLLISION" if has_collision else "CLEAR"
                print(f"  Evaluation {self.evaluation_count}: {status}, Score={score:.2f}, Min_dist={min_distance:.2f}")
        
        return score

    def simple_cmaes(self, initial_x, sigma, max_evals):
        """Simple CMA-ES implementation (since cma library might not be available)"""
        print("Using simplified CMA-ES implementation")
        
        # Problem dimension
        N = len(initial_x)
        
        # Population size (lambda)
        lam = 4 + int(3 * np.log(N))  # Default CMA-ES population size
        if self.population_size:
            lam = self.population_size
        
        # Selection size (mu)
        mu = lam // 2
        
        # Weights for recombination
        weights = np.log(mu + 0.5) - np.log(np.arange(1, mu + 1))
        weights = weights / np.sum(weights)
        mueff = np.sum(weights)**2 / np.sum(weights**2)
        
        # Strategy parameters
        cc = (4 + mueff/N) / (N + 4 + 2*mueff/N)
        cs = (mueff + 2) / (N + mueff + 5)
        c1 = 2 / ((N + 1.3)**2 + mueff)
        cmu = min(1 - c1, 2 * (mueff - 2 + 1/mueff) / ((N + 2)**2 + mueff))
        damps = 1 + 2*max(0, np.sqrt((mueff-1)/(N+1)) - 1) + cs
        
        # Dynamic parameters
        xmean = initial_x.copy()
        pc = np.zeros(N)
        ps = np.zeros(N)
        B = np.eye(N)
        D = np.ones(N)
        C = np.eye(N)
        eigeneval = 0
        
        generation = 0
        best_fitness = float('inf')
        best_x = xmean.copy()
        
        for evaluation in range(0, max_evals, lam):
            generation += 1
            
            # Generate population
            population = []
            fitness = []
            
            for i in range(lam):
                # Sample from multivariate normal
                z = np.random.randn(N)
                y = B @ (D * z)
                x = xmean + sigma * y
                
                population.append(x)
                f = self.objective_function(x)
                fitness.append(f)
                
                if f < best_fitness:
                    best_fitness = f
                    best_x = x.copy()
            
            # Sort by fitness
            indices = np.argsort(fitness)
            fitness = [fitness[i] for i in indices]
            population = [population[i] for i in indices]
            
            # Update mean
            xold = xmean.copy()
            xmean = np.sum([weights[i] * population[i] for i in range(mu)], axis=0)
            
            # Update evolution paths
            ps = (1 - cs) * ps + np.sqrt(cs * (2 - cs) * mueff) * (xmean - xold) / sigma
            
            # Update covariance matrix
            if generation % (N / (10 * c1 + cmu)) < 1:  # Update eigendecomposition
                C = np.triu(C) + np.triu(C, 1).T  # Enforce symmetry
                D, B = np.linalg.eigh(C)
                D = np.sqrt(np.maximum(D, 0))  # Ensure positive
                eigeneval = generation
            
            # Adapt step size
            ps_norm = np.linalg.norm(ps)
            sigma = sigma * np.exp((cs / damps) * (ps_norm / np.sqrt(N) - 1))
            
            # Progress reporting
            if generation % 10 == 0:
                print(f"  Generation {generation}: Best fitness = {best_fitness:.2f}, Sigma = {sigma:.3f}")
            
            # Early termination checks
            if best_fitness < 20.0:  # Good collision-free solution found
                print(f"  Early termination: Good solution found at generation {generation}")
                break
            
            if sigma < 1e-10:  # Step size too small
                print(f"  Early termination: Step size too small at generation {generation}")
                break
            
            if self.evaluation_count >= max_evals:
                break
        
        self.generation_count = generation
        return best_x, best_fitness

    def solve(self):
        """Main CMA-ES optimization"""
        start_time = time.time()
        
        print("Starting CMA-ES optimization...")
        print(f"Max evaluations: {self.max_evaluations}")
        print(f"Initial sigma: {self.initial_sigma}")
        
        # Reset tracking variables
        self.evaluation_count = 0
        self.best_score_history = []
        self.generation_count = 0
        
        # Initial solution (flatten middle points of original)
        initial_vector = self.points_to_vector(self.original_points)
        dimension = len(initial_vector)
        
        print(f"Optimizing {dimension} parameters ({dimension//3} points)")
        
        # Try to import cma library, fall back to simple implementation
        try:
            import cma
            print("Using professional CMA library")
            
            # Set population size if not specified
            if self.population_size is None:
                self.population_size = 4 + int(3 * np.log(dimension))
            
            # CMA-ES options
            options = {
                'maxfevals': self.max_evaluations,
                'popsize': self.population_size,
                'verb_disp': 100,  # Display every 100 evaluations
                'verb_log': 0,     # No log file
                'tolfun': 1e-6,    # Function tolerance
                'tolx': 1e-8,      # Solution tolerance
            }
            
            # Run CMA-ES
            es = cma.CMAEvolutionStrategy(initial_vector, self.initial_sigma, options)
            
            while not es.stop():
                solutions = es.ask()
                fitness_list = [self.objective_function(x) for x in solutions]
                es.tell(solutions, fitness_list)
                
                # Progress reporting
                if es.counteval % 100 == 0:
                    best_fitness = min(fitness_list)
                    print(f"  Evaluation {es.counteval}: Best fitness = {best_fitness:.2f}")
                
                # Early termination for good solutions
                if min(fitness_list) < 20.0:
                    print(f"  Early termination: Good solution found at evaluation {es.counteval}")
                    break
            
            # Get best solution
            final_vector = es.result.xbest
            final_score = es.result.fbest
            self.generation_count = es.counteval // self.population_size
            
        except ImportError:
            print("CMA library not available, using simplified implementation")
            # Use our simple CMA-ES implementation
            final_vector, final_score = self.simple_cmaes(
                initial_vector, self.initial_sigma, self.max_evaluations
            )
        
        # Convert solution back to points
        final_points = self.vector_to_points(final_vector)
        
        # Final evaluation
        final_spline, _, _ = self.create_spline(final_points)
        has_collision, collision_points, min_distance = self.check_collision(final_spline)
        
        # Generate waypoints
        waypoints = None
        if final_spline is not None:
            waypoints = self.sample_spline_at_distances(final_spline, self.original_cumulative_distances)
        
        computation_time = time.time() - start_time
        
        result = {
            'method': 'CMA-ES',
            'control_points': final_points,
            'spline_points': final_spline,
            'waypoints': waypoints,
            'score': final_score,
            'has_collision': has_collision,
            'min_distance': min_distance,
            'collision_points': collision_points,
            'generations_used': self.generation_count,
            'evaluations_used': self.evaluation_count,
            'computation_time': computation_time,
            'converged': final_score < 100.0  # Reasonable convergence criterion
        }
        
        return result

def visualize_results(optimizer, result):
    """Visualize the CMA-ES result"""
    fig = go.Figure()
    
    # Add obstacles
    colors = ['Reds', 'Blues', 'Greens']
    for i, obstacle in enumerate(optimizer.original_obstacles):
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        
        x_sphere = obstacle['radius'] * np.outer(np.cos(u), np.sin(v)) + obstacle['center'][0]
        y_sphere = obstacle['radius'] * np.outer(np.sin(u), np.sin(v)) + obstacle['center'][1]
        z_sphere = obstacle['radius'] * np.outer(np.ones(np.size(u)), np.cos(v)) + obstacle['center'][2]
        
        safety_radius = obstacle['radius'] + optimizer.spline_diameter/2
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
    fig.add_trace(go.Scatter3d(x=optimizer.original_points[:, 0], 
                              y=optimizer.original_points[:, 1], 
                              z=optimizer.original_points[:, 2],
                              mode='lines+markers', 
                              line=dict(color='blue', width=4, dash='dash'),
                              marker=dict(color='blue', size=6),
                              name='Original Path'))
    
    # CMA-ES solution
    if result['spline_points'] is not None:
        fig.add_trace(go.Scatter3d(x=result['spline_points'][:, 0], 
                                  y=result['spline_points'][:, 1], 
                                  z=result['spline_points'][:, 2],
                                  mode='lines', line=dict(color='gold', width=6),
                                  name=f"CMA-ES Solution (Score: {result['score']:.1f})"))
        
        if result['waypoints'] is not None:
            fig.add_trace(go.Scatter3d(x=result['waypoints'][:, 0], 
                                      y=result['waypoints'][:, 1], 
                                      z=result['waypoints'][:, 2],
                                      mode='markers+lines', 
                                      marker=dict(color='red', size=10, symbol='diamond'),
                                      line=dict(color='red', width=3),
                                      name="Waypoints"))
    
    # Control points
    fig.add_trace(go.Scatter3d(x=result['control_points'][:, 0], 
                              y=result['control_points'][:, 1], 
                              z=result['control_points'][:, 2],
                              mode='markers', 
                              marker=dict(color='orange', size=8, symbol='circle'),
                              name="Control Points"))
    
    fig.update_layout(
        title=f"CMA-ES Optimization - {result['method']}",
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
    
    # Create and run CMA-ES optimizer
    optimizer = CMAESOptimizer(obstacles, original_points, spline_diameter=20)
    
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
    print("CMA-ES RESULTS:")
    print("="*70)
    print(f"Method: {result['method']}")
    print(f"Converged: {result['converged']} (in {result['generations_used']} generations)")
    print(f"Function evaluations: {result['evaluations_used']}")
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