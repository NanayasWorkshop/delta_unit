"""
Delta Robot Python Package (Consolidated Module with Mesh Collision)
"""
import numpy as np

# Import the single consolidated module
try:
    from . import delta_robot_complete as delta_complete
    
    # Create convenient aliases that maintain backward compatibility
    class fermat:
        FermatModule = delta_complete.FermatModule
        FermatResult = delta_complete.FermatResult
    
    class joint_state:
        JointStateModule = delta_complete.JointStateModule
        JointStateResult = delta_complete.JointStateResult
    
    class kinematics:
        KinematicsModule = delta_complete.KinematicsModule
        KinematicsResult = delta_complete.KinematicsResult
    
    class orientation:
        OrientationModule = delta_complete.OrientationModule
        OrientationResult = delta_complete.OrientationResult
    
    class fabrik:
        FabrikSolver = delta_complete.FabrikSolver
        FabrikInitialization = delta_complete.FabrikInitialization
        FabrikChain = delta_complete.FabrikChain
        FabrikJoint = delta_complete.FabrikJoint
        solve_delta_robot = delta_complete.solve_delta_robot
        
    class motor:
        MotorModule = delta_complete.MotorModule
        MotorResult = delta_complete.MotorResult
        LevelData = delta_complete.LevelData
    
    class collision:
        CollisionPill = delta_complete.CollisionPill
        CollisionMesh = delta_complete.CollisionMesh
        Triangle = delta_complete.Triangle
        
        # Mesh management functions
        update_mesh = delta_complete.update_collision_mesh
        remove_mesh = delta_complete.remove_collision_mesh
        clear_all_meshes = delta_complete.clear_all_collision_meshes
        get_pills = delta_complete.get_collision_pills
        get_meshes = delta_complete.get_collision_meshes
        
    # Constants from the consolidated module
    FABRIK_TOLERANCE = delta_complete.FABRIK_TOLERANCE
    DEFAULT_ROBOT_SEGMENTS = delta_complete.DEFAULT_ROBOT_SEGMENTS
    COLLISION_PILL_RADIUS = delta_complete.COLLISION_PILL_RADIUS
    MAX_COLLISION_RESOLVE_ATTEMPTS = delta_complete.MAX_COLLISION_RESOLVE_ATTEMPTS
    
except ImportError as e:
    raise ImportError(f"Failed to import delta_robot_complete module: {e}")

def calculate_complete_pipeline(x, y, z):
    """Complete delta robot calculation pipeline."""
    direction_vector = np.array([x, y, z])
    
    fermat_result = fermat.FermatModule.calculate(x, y, z)
    joint_result = joint_state.JointStateModule.calculate_from_fermat(direction_vector, fermat_result)
    kinematics_result = kinematics.KinematicsModule.calculate(x, y, z)
    orientation_result = orientation.OrientationModule.calculate(x, y, z)
    
    return {
        'input_vector': (x, y, z),
        'fermat': fermat_result,
        'joint_state': joint_result,
        'kinematics': kinematics_result,
        'orientation': orientation_result
    }

def solve_fabrik_ik(target_x, target_y, target_z, num_segments=None, tolerance=None):
    """Complete FABRIK inverse kinematics solution."""
    if num_segments is None:
        num_segments = DEFAULT_ROBOT_SEGMENTS
    if tolerance is None:
        tolerance = FABRIK_TOLERANCE
    
    target = np.array([target_x, target_y, target_z])
    return fabrik.solve_delta_robot(num_segments, target, tolerance)

def calculate_motors(target_x, target_y, target_z):
    """Calculate motor positions using the Motor module."""
    return motor.MotorModule.calculate_motors(target_x, target_y, target_z)

def update_collision_mesh(mesh_id, vertices, faces):
    """
    Update collision mesh for dynamic obstacles.
    
    Args:
        mesh_id (int): Unique identifier for the mesh
        vertices (numpy.ndarray): Nx3 array of vertex positions
        faces (numpy.ndarray): Mx3 array of triangle face indices
    """
    # Ensure correct data types
    vertices = np.asarray(vertices, dtype=np.float32)
    faces = np.asarray(faces, dtype=np.int32)
    
    # Validate shapes
    if vertices.ndim != 2 or vertices.shape[1] != 3:
        raise ValueError("Vertices must be Nx3 array")
    if faces.ndim != 2 or faces.shape[1] != 3:
        raise ValueError("Faces must be Mx3 array")
    
    return collision.update_mesh(mesh_id, vertices, faces)

def remove_collision_mesh(mesh_id):
    """Remove collision mesh by ID."""
    return collision.remove_mesh(mesh_id)

def clear_all_collision_meshes():
    """Clear all collision meshes."""
    return collision.clear_all_meshes()

def get_collision_pills():
    """Get current active collision pills."""
    return collision.get_pills()

def get_collision_meshes():
    """Get current active collision meshes."""
    return collision.get_meshes()

def verify_installation():
    """Verify that the consolidated module imported correctly."""
    try:
        # Test basic functionality
        result = motor.MotorModule.calculate_motors(100, 50, 300)
        print("✓ Consolidated delta robot module imported successfully!")
        print("✓ All functionality available through single module")
        print("✓ Collision detection system integrated")
        print("✓ Mesh collision support enabled")
        return True
    except Exception as e:
        print(f"✗ Module test failed: {e}")
        return False

# Convenience functions for creating vectors
def create_vector3(x=0, y=0, z=0):
    """Create Vector3 (numpy array) from coordinates."""
    return np.array([x, y, z])

def create_sphere_mesh(center, radius, resolution=20):
    """
    Create a sphere mesh for collision testing.
    
    Args:
        center (tuple): (x, y, z) center position
        radius (float): Sphere radius
        resolution (int): Number of subdivisions
    
    Returns:
        tuple: (vertices, faces) numpy arrays
    """
    phi = np.linspace(0, np.pi, resolution)
    theta = np.linspace(0, 2*np.pi, resolution)
    
    vertices = []
    for p in phi:
        for t in theta:
            x = center[0] + radius * np.sin(p) * np.cos(t)
            y = center[1] + radius * np.sin(p) * np.sin(t)
            z = center[2] + radius * np.cos(p)
            vertices.append([x, y, z])
    
    vertices = np.array(vertices, dtype=np.float32)
    
    # Generate faces (simplified - this creates a basic triangulation)
    faces = []
    for i in range(resolution - 1):
        for j in range(resolution - 1):
            v1 = i * resolution + j
            v2 = i * resolution + (j + 1)
            v3 = (i + 1) * resolution + j
            v4 = (i + 1) * resolution + (j + 1)
            
            faces.append([v1, v2, v3])
            faces.append([v2, v4, v3])
    
    faces = np.array(faces, dtype=np.int32)
    return vertices, faces

# Export main interfaces
__all__ = [
    'fermat', 'joint_state', 'kinematics', 'orientation', 'fabrik', 'motor', 'collision',
    'calculate_complete_pipeline', 'solve_fabrik_ik', 'calculate_motors',
    'update_collision_mesh', 'remove_collision_mesh', 'clear_all_collision_meshes',
    'get_collision_pills', 'get_collision_meshes',
    'verify_installation', 'create_vector3', 'create_sphere_mesh',
    'FABRIK_TOLERANCE', 'DEFAULT_ROBOT_SEGMENTS', 'COLLISION_PILL_RADIUS', 'MAX_COLLISION_RESOLVE_ATTEMPTS'
]