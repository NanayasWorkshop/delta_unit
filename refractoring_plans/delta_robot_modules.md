# Delta Robot LEGO Block Modules

## Level 0: Core Math Modules (No Dependencies)

### FermatSolver
- **Input**: `Vector3 direction`
- **Output**: `FermatResult { z_A, z_B, z_C, fermat_point, computation_time }`
- **Purpose**: Calculate Fermat point and motor Z positions from direction vector
- **Dependencies**: None (pure math)

### JointStateSolver  
- **Input**: `Vector3 direction, Vector3 fermat_point`
- **Output**: `JointStateResult { prismatic_length, roll_angle, pitch_angle, computation_time }`
- **Purpose**: Calculate joint angles from direction and fermat point
- **Dependencies**: None (pure math)

## Level 1: Composite Kinematics Modules

### KinematicsSolver
- **Input**: `Vector3 direction`
- **Output**: `KinematicsResult { end_effector_position, prismatic_length, fermat_data, joint_data, computation_time }`
- **Purpose**: Complete forward kinematics chain from direction to end-effector
- **Dependencies**: FermatSolver + JointStateSolver (internal)

### OrientationSolver
- **Input**: `Vector3 direction`
- **Output**: `OrientationResult { transformation_matrix, coordinate_frames, computation_time }`
- **Purpose**: Calculate orientation transformation matrix and coordinate frames
- **Dependencies**: KinematicsSolver (internal)

## Level 2: Inverse Kinematics Modules

### FABRIKSolver
- **Input**: `Vector3 target, Vector3[] initial_joint_positions, FABRIKConfig config`
- **Output**: `FABRIKResult { joint_positions[], converged, final_error, iterations, computation_time }`
- **Purpose**: Solve inverse kinematics to reach target position
- **Dependencies**: KinematicsSolver (for segment length calculations)

### CollisionAwareSolver (Orchestrator)
- **Input**: `Vector3 target, Obstacle[] obstacles, Vector3[] initial_joint_positions, CollisionConfig config`
- **Output**: `CollisionResult { joint_positions[], collision_free, collision_iterations, fabrik_time, collision_time }`
- **Purpose**: Orchestrate collision-aware solving by coordinating calls to other modules in iteration loop
- **Dependencies**: FABRIKSolver + UPointsExtractor + CollisionDetector + WaypointConverter
- **Note**: Provides high-level convenience API (4 lines of user code vs 32 lines manual orchestration)

## Level 3: Physical Interface Modules

### SegmentCalculator
- **Input**: `Vector3[] joint_positions, int num_segments`
- **Output**: `SegmentResult { segment_end_effectors[], segment_numbers[], calculation_time }`
- **Purpose**: Calculate physical segment end-effector positions from joint positions
- **Dependencies**: KinematicsSolver (for direction transformations)
- **Agnostic**: Doesn't care how joint_positions were obtained

### MotorController
- **Input**: `Vector3[] segment_positions, int num_levels`
- **Output**: `MotorResult { levels[], motor_coordinates[], transformation_data[], calculation_time }`
- **Purpose**: Convert segment positions to motor coordinates and level transformations
- **Dependencies**: KinematicsSolver + OrientationSolver (for transformations)
- **Agnostic**: Doesn't care how segment_positions were obtained

## Support Modules (Used by other modules)

### UPointsExtractor
- **Input**: `Vector3[] joint_positions`
- **Output**: `Vector3[] u_points`
- **Purpose**: Extract collision detection points from joint positions
- **Dependencies**: None

### CollisionDetector
- **Input**: `Vector3[] u_points, Obstacle[] obstacles, double spline_diameter`
- **Output**: `CollisionResult { has_collision, waypoints[], min_distance, collision_points[], computation_time }`
- **Purpose**: Detect collisions and provide avoidance waypoints
- **Dependencies**: None (contains swarm optimization internally)

### WaypointConverter
- **Input**: `Vector3[] waypoints`
- **Output**: `WaypointResult { joint_positions[], conversion_successful, segment_lengths[], computation_time }`
- **Purpose**: Convert collision avoidance waypoints back to joint positions
- **Dependencies**: None

## Data Handoff Points (Where Modules Don't Care About Upstream)

1. **Joint Positions Handoff**: `Vector3[] joint_positions`
   - Produced by: FABRIKSolver OR CollisionAwareSolver
   - Consumed by: SegmentCalculator, UPointsExtractor
   - **Agnostic**: Consumers don't care about IK method used

2. **Segment Positions Handoff**: `Vector3[] segment_positions` 
   - Produced by: SegmentCalculator
   - Consumed by: MotorController
   - **Agnostic**: Consumer doesn't care about segment calculation method

3. **Direction Vector Handoff**: `Vector3 direction`
   - Produced by: User input or other calculations
   - Consumed by: All kinematics modules
   - **Agnostic**: Consumers don't care about direction source

## Usage Patterns

### Simple IK Workflow
```
Target → FABRIKSolver → SegmentCalculator → MotorController
```

### Collision-Aware Workflow  
```
Target + Obstacles → CollisionAwareSolver → coordinates: FABRIKSolver + UPointsExtractor + CollisionDetector + WaypointConverter (loop) → SegmentCalculator → MotorController
```

### Flexible Access Patterns
```cpp
// High-level convenience (most users) - 4 lines
auto result = CollisionAwareSolver::solve(target, obstacles, joints, config);
auto segments = SegmentCalculator::calculate(result.joint_positions);
auto motors = MotorController::calculate(segments.segment_positions);

// Low-level control (advanced users, testing, research) - 32+ lines
auto fabrik_result = FABRIKSolver::solve(target, joints, config);
auto u_points = UPointsExtractor::extract(fabrik_result.joint_positions);
auto collision_result = CollisionDetector::check(u_points, obstacles);
// ... manual iteration loop
```

### Research/Analysis Workflows
```
Target → FABRIKSolver → [custom analysis]
Segments → [custom processor] → MotorController
Direction → KinematicsSolver → [analysis]
```