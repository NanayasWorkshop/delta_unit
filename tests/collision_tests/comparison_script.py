import numpy as np
import plotly.graph_objects as go
import plotly.subplots as sp
import time
import pandas as pd

# Import all optimization methods
# (You would import from the separate files in practice)
# from elastic_band_optimizer import ElasticBandOptimizer
# from differential_evolution_optimizer import DifferentialEvolutionOptimizer  
# from cmaes_optimizer import CMAESOptimizer
# from push_spline import ConicalSwarmSplineAvoider

class OptimizationComparison:
    def __init__(self, obstacles, original_points, spline_diameter=20):
        self.obstacles = obstacles
        self.original_points = original_points
        self.spline_diameter = spline_diameter
        self.results = {}
        
    def run_all_methods(self):
        """Run all optimization methods and collect results"""
        print("="*80)
        print("OPTIMIZATION METHODS COMPARISON")
        print("="*80)
        print(f"Problem: {len(self.original_points)} control points, {len(self.obstacles)} obstacles")
        print(f"Spline diameter: {self.spline_diameter}mm")
        print()
        
        methods = [
            ('Elastic Band', self._run_elastic_band),
            ('Differential Evolution', self._run_differential_evolution),
            ('CMA-ES', self._run_cmaes),
            ('Conical Swarm', self._run_swarm)
        ]
        
        for method_name, method_func in methods:
            print(f"\n{'='*20} {method_name.upper()} {'='*20}")
            try:
                start_time = time.time()
                result = method_func()
                total_time = time.time() - start_time
                
                result['total_time'] = total_time
                result['method'] = method_name
                self.results[method_name] = result
                
                print(f"✓ {method_name} completed in {total_time:.2f}s")
                
            except Exception as e:
                print(f"✗ {method_name} failed: {str(e)}")
                self.results[method_name] = {
                    'method': method_name,
                    'failed': True,
                    'error': str(e)
                }
    
    def _run_elastic_band(self):
        """Run Elastic Band optimizer"""
        from elastic_band_optimizer import ElasticBandOptimizer
        optimizer = ElasticBandOptimizer(self.obstacles, self.original_points, self.spline_diameter)
        return optimizer.solve()
    
    def _run_differential_evolution(self):
        """Run Differential Evolution optimizer"""
        from differential_evolution_optimizer import DifferentialEvolutionOptimizer
        optimizer = DifferentialEvolutionOptimizer(self.obstacles, self.original_points, self.spline_diameter)
        return optimizer.solve()
    
    def _run_cmaes(self):
        """Run CMA-ES optimizer"""
        from cmaes_optimizer import CMAESOptimizer
        optimizer = CMAESOptimizer(self.obstacles, self.original_points, self.spline_diameter)
        return optimizer.solve()
    
    def _run_swarm(self):
        """Run Conical Swarm optimizer"""
        from conical_swarm_optimizer import ConicalSwarmOptimizer
        optimizer = ConicalSwarmOptimizer(self.obstacles, self.original_points, self.spline_diameter)
        return optimizer.solve()
    
    def print_comparison_table(self):
        """Print a comparison table of all methods"""
        print("\n" + "="*100)
        print("RESULTS COMPARISON TABLE")
        print("="*100)
        
        # Create comparison data
        comparison_data = []
        
        for method_name, result in self.results.items():
            if result.get('failed', False):
                comparison_data.append({
                    'Method': method_name,
                    'Success': '❌ FAILED',
                    'Collision Free': 'N/A',
                    'Score': 'N/A',
                    'Min Distance': 'N/A',
                    'Time (s)': 'N/A',
                    'Iterations/Evals': 'N/A'
                })
            else:
                # Calculate iterations/evaluations
                iters_evals = 'N/A'
                if 'iterations_used' in result:
                    iters_evals = f"{result['iterations_used']} iter"
                elif 'evaluations_used' in result:
                    iters_evals = f"{result['evaluations_used']} eval"
                elif 'generations_used' in result:
                    iters_evals = f"{result['generations_used']} gen"
                
                comparison_data.append({
                    'Method': method_name,
                    'Success': '✓ SUCCESS',
                    'Collision Free': '✓ YES' if not result.get('has_collision', True) else '❌ NO',
                    'Score': f"{result.get('score', float('inf')):.1f}",
                    'Min Distance': f"{result.get('min_distance', 0):.1f}mm",
                    'Time (s)': f"{result.get('total_time', 0):.2f}",
                    'Iterations/Evals': iters_evals
                })
        
        # Create DataFrame for nice formatting
        df = pd.DataFrame(comparison_data)
        print(df.to_string(index=False))
        
        # Performance rankings
        print(f"\n{'='*50}")
        print("PERFORMANCE RANKINGS")
        print("="*50)
        
        # Speed ranking
        successful_results = {k: v for k, v in self.results.items() if not v.get('failed', False)}
        if successful_results:
            speed_ranking = sorted(successful_results.items(), key=lambda x: x[1].get('total_time', float('inf')))
            print("Speed (fastest to slowest):")
            for i, (method, result) in enumerate(speed_ranking, 1):
                print(f"  {i}. {method}: {result.get('total_time', 0):.2f}s")
            
            # Quality ranking (collision-free solutions only)
            collision_free = {k: v for k, v in successful_results.items() if not v.get('has_collision', True)}
            if collision_free:
                quality_ranking = sorted(collision_free.items(), key=lambda x: x[1].get('score', float('inf')))
                print("\nQuality (best to worst score, collision-free only):")
                for i, (method, result) in enumerate(quality_ranking, 1):
                    print(f"  {i}. {method}: Score={result.get('score', 0):.1f}, Min_dist={result.get('min_distance', 0):.1f}mm")
            else:
                print("\nQuality: No collision-free solutions found!")
                
            # Distance from original (shape preservation)
            print("\nShape Preservation (closest to original):")
            shape_scores = []
            for method, result in successful_results.items():
                if 'control_points' in result and result['control_points'] is not None:
                    total_displacement = 0
                    for i, point in enumerate(result['control_points']):
                        original = np.array(self.original_points[i])
                        displacement = np.linalg.norm(point - original)
                        total_displacement += displacement
                    avg_displacement = total_displacement / len(result['control_points'])
                    shape_scores.append((method, avg_displacement))
            
            shape_ranking = sorted(shape_scores, key=lambda x: x[1])
            for i, (method, displacement) in enumerate(shape_ranking, 1):
                print(f"  {i}. {method}: {displacement:.1f}mm average displacement")
    
    def visualize_all_results(self):
        """Create a comprehensive visualization of all results"""
        # Filter successful results
        successful_results = {k: v for k, v in self.results.items() if not v.get('failed', False)}
        
        if not successful_results:
            print("No successful results to visualize!")
            return
        
        # Create subplot figure
        num_methods = len(successful_results)
        cols = min(2, num_methods)
        rows = (num_methods + 1) // 2
        
        fig = sp.make_subplots(
            rows=rows, cols=cols,
            specs=[[{'type': 'scatter3d'} for _ in range(cols)] for _ in range(rows)],
            subplot_titles=list(successful_results.keys()),
            vertical_spacing=0.1,
            horizontal_spacing=0.1
        )
        
        # Color scheme for different methods
        method_colors = {
            'Elastic Band': 'lime',
            'Differential Evolution': 'purple', 
            'CMA-ES': 'gold',
            'Conical Swarm': 'cyan'
        }
        
        for idx, (method_name, result) in enumerate(successful_results.items()):
            row = (idx // cols) + 1
            col = (idx % cols) + 1
            
            # Add obstacles (same for all)
            for i, obstacle in enumerate(self.obstacles):
                u = np.linspace(0, 2 * np.pi, 10)  # Fewer points for subplots
                v = np.linspace(0, np.pi, 10)
                
                x_sphere = obstacle['radius'] * np.outer(np.cos(u), np.sin(v)) + obstacle['center'][0]
                y_sphere = obstacle['radius'] * np.outer(np.sin(u), np.sin(v)) + obstacle['center'][1]
                z_sphere = obstacle['radius'] * np.outer(np.ones(np.size(u)), np.cos(v)) + obstacle['center'][2]
                
                fig.add_trace(
                    go.Surface(x=x_sphere, y=y_sphere, z=z_sphere, 
                              opacity=0.3, colorscale='Reds', showscale=False,
                              name=f'Obstacle {i+1}'),
                    row=row, col=col
                )
            
            # Original path
            fig.add_trace(
                go.Scatter3d(x=[p[0] for p in self.original_points], 
                            y=[p[1] for p in self.original_points], 
                            z=[p[2] for p in self.original_points],
                            mode='lines+markers', 
                            line=dict(color='blue', width=3, dash='dash'),
                            marker=dict(color='blue', size=4),
                            name='Original'),
                row=row, col=col
            )
            
            # Method solution
            if result.get('spline_points') is not None:
                color = method_colors.get(method_name, 'red')
                
                fig.add_trace(
                    go.Scatter3d(x=result['spline_points'][:, 0], 
                                y=result['spline_points'][:, 1], 
                                z=result['spline_points'][:, 2],
                                mode='lines', 
                                line=dict(color=color, width=4),
                                name=f"{method_name}"),
                    row=row, col=col
                )
                
                # Waypoints
                if result.get('waypoints') is not None:
                    fig.add_trace(
                        go.Scatter3d(x=result['waypoints'][:, 0], 
                                    y=result['waypoints'][:, 1], 
                                    z=result['waypoints'][:, 2],
                                    mode='markers', 
                                    marker=dict(color='red', size=6, symbol='diamond'),
                                    name="Waypoints"),
                        row=row, col=col
                    )
        
        fig.update_layout(
            title_text="Optimization Methods Comparison",
            height=400 * rows,
            width=600 * cols,
            showlegend=False  # Too cluttered with multiple subplots
        )
        
        fig.show()
    
    def analyze_constraint_satisfaction(self):
        """Analyze how well each method satisfies constraints"""
        print(f"\n{'='*60}")
        print("CONSTRAINT SATISFACTION ANALYSIS")
        print("="*60)
        
        successful_results = {k: v for k, v in self.results.items() if not v.get('failed', False)}
        
        for method_name, result in successful_results.items():
            print(f"\n{method_name}:")
            
            if 'waypoints' in result and result['waypoints'] is not None:
                # Check distance preservation
                original_dists = self.calculate_cumulative_distances(self.original_points)
                waypoint_dists = self.calculate_cumulative_distances(result['waypoints'])
                
                print("  Distance preservation:")
                max_error_percent = 0
                for i, (orig, wayp) in enumerate(zip(original_dists, waypoint_dists)):
                    error = abs(wayp - orig)
                    error_percent = (error / (orig + 1e-6)) * 100
                    max_error_percent = max(max_error_percent, error_percent)
                    if error_percent > 5.0:  # Flag large errors
                        print(f"    ❌ Segment {i}: {error_percent:.1f}% error")
                
                if max_error_percent <= 3.0:
                    print(f"    ✓ All segments within 3% tolerance (max: {max_error_percent:.1f}%)")
                elif max_error_percent <= 5.0:
                    print(f"    ⚠️  Some segments exceed 3% tolerance (max: {max_error_percent:.1f}%)")
                else:
                    print(f"    ❌ Large constraint violations (max: {max_error_percent:.1f}%)")
            else:
                print("  ❌ No waypoints available for constraint analysis")
    
    def calculate_cumulative_distances(self, points):
        """Calculate cumulative distances along a path"""
        points = np.array(points)
        cumulative_distances = [0.0]
        
        for i in range(len(points) - 1):
            distance = np.linalg.norm(points[i+1] - points[i])
            cumulative_distances.append(cumulative_distances[-1] + distance)
        
        return cumulative_distances

def main():
    """Main comparison execution"""
    # Test setup (same as in individual scripts)
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
    
    # Run comparison
    comparison = OptimizationComparison(obstacles, original_points, spline_diameter=20)
    comparison.run_all_methods()
    comparison.print_comparison_table()
    comparison.analyze_constraint_satisfaction()
    comparison.visualize_all_results()
    
    # Summary conclusions
    print(f"\n{'='*80}")
    print("SUMMARY CONCLUSIONS")
    print("="*80)
    
    successful = [k for k, v in comparison.results.items() if not v.get('failed', False)]
    collision_free = [k for k, v in comparison.results.items() 
                     if not v.get('failed', False) and not v.get('has_collision', True)]
    
    print(f"Methods tested: {len(comparison.results)}")
    print(f"Successful runs: {len(successful)}")
    print(f"Collision-free solutions: {len(collision_free)}")
    
    if collision_free:
        print(f"\nRecommendation: {collision_free[0]} performed best overall")
    else:
        print("\nNo collision-free solutions found - consider:")
        print("  - Reducing obstacle safety margins")
        print("  - Allowing larger segment length tolerance") 
        print("  - Using different initial conditions")
        print("  - Increasing search space or iterations")

if __name__ == "__main__":
    main()