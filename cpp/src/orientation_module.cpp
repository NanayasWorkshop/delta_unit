#include "orientation_module.hpp"
#include <cmath>

namespace delta {

OrientationResult OrientationModule::calculate(double x, double y, double z) {
    return calculate(Vector3(x, y, z));
}

OrientationResult OrientationModule::calculate(const Vector3& input_vector) {
    // Get kinematics result first
    KinematicsResult kinematics = KinematicsModule::calculate(input_vector);
    return calculate_from_kinematics(kinematics);
}

OrientationResult OrientationModule::calculate_from_kinematics(const KinematicsResult& kinematics_result) {
    // Get required points from kinematics result
    Vector3 fermat_point = kinematics_result.fermat_data.fermat_point;
    Vector3 end_effector = kinematics_result.end_effector_position;
    
    // Get A, B, C points from fermat data
    Vector3 base_A = get_base_position_A();
    Vector3 base_B = get_base_position_B();
    Vector3 base_C = get_base_position_C();
    
    Vector3 A_point(base_A.x, base_A.y, kinematics_result.fermat_data.z_A);
    Vector3 B_point(base_B.x, base_B.y, kinematics_result.fermat_data.z_B);
    Vector3 C_point(base_C.x, base_C.y, kinematics_result.fermat_data.z_C);
    
    // Step 1: Create UVW coordinate system at Fermat point
    CoordinateFrame UVW_frame = create_UVW_frame(fermat_point, A_point, B_point, C_point);
    
    // Step 2: Mirror UVW across XY plane to get IJK
    CoordinateFrame IJK_frame = mirror_across_XY(UVW_frame);
    
    // Step 3: Translate to align with XYZ origin, get U'V'W'
    CoordinateFrame aligned_frame = align_with_origin(UVW_frame, IJK_frame);
    
    // Step 4: Translate to end-effector position to get U''V''W''
    CoordinateFrame final_frame = translate_to_endeffector(aligned_frame, end_effector);
    
    // Create transformation matrix
    Matrix4x4 transformation = create_transformation_matrix(final_frame);
    
    // NEW! Return all coordinate frames
    return OrientationResult(transformation, fermat_point, end_effector, 
                           UVW_frame, IJK_frame, aligned_frame, final_frame);
}

CoordinateFrame OrientationModule::create_UVW_frame(const Vector3& fermat_point,
                                                   const Vector3& A_point,
                                                   const Vector3& B_point,
                                                   const Vector3& C_point) {
    // U-axis: Fermat point → A point
    Vector3 u_axis = (A_point - fermat_point).normalized();
    
    // W-axis: Normal to ABC plane (pointing +Z direction)
    Vector3 w_axis = calculate_plane_normal(A_point, B_point, C_point);
    // Ensure W points in +Z direction
    if (w_axis.z < 0) {
        w_axis = Vector3(-w_axis.x, -w_axis.y, -w_axis.z);
    }
    
    // V-axis: U × W (right-hand rule)
    Vector3 v_axis(
        u_axis.y * w_axis.z - u_axis.z * w_axis.y,
        u_axis.z * w_axis.x - u_axis.x * w_axis.z,
        u_axis.x * w_axis.y - u_axis.y * w_axis.x
    );
    v_axis = v_axis.normalized();
    
    return CoordinateFrame(fermat_point, u_axis, v_axis, w_axis);
}

CoordinateFrame OrientationModule::mirror_across_XY(const CoordinateFrame& uvw_frame) {
    // Mirror across XY plane: (x,y,z) → (x,y,-z)
    Vector3 mirrored_origin(uvw_frame.origin.x, uvw_frame.origin.y, -uvw_frame.origin.z);
    Vector3 mirrored_u(uvw_frame.u_axis.x, uvw_frame.u_axis.y, -uvw_frame.u_axis.z);
    Vector3 mirrored_v(uvw_frame.v_axis.x, uvw_frame.v_axis.y, -uvw_frame.v_axis.z);
    Vector3 mirrored_w(uvw_frame.w_axis.x, uvw_frame.w_axis.y, -uvw_frame.w_axis.z);
    
    // THEN invert W to make K point upward (create IJK from mirrored UVW)
    Vector3 inverted_k(-mirrored_w.x, -mirrored_w.y, -mirrored_w.z);
    
    return CoordinateFrame(mirrored_origin, mirrored_u, mirrored_v, inverted_k);
}

CoordinateFrame OrientationModule::align_with_origin(const CoordinateFrame& uvw_frame,
                                                    const CoordinateFrame& ijk_frame) {
    // Calculate transformation matrix to align IJK with XYZ
    
    // 1. Translation: move IJK origin to XYZ origin (0,0,0)
    Vector3 translation = Vector3(0, 0, 0) - ijk_frame.origin;
    
    // 2. Rotation: align IJK axes with XYZ axes
    // We need R such that: R * [I J K] = [X Y Z] = Identity
    // Therefore: R = [I J K]^(-1)  <<<--- KEY FIX: INVERSE, NOT TRANSPOSE!
    
    // Current IJK axes
    Vector3 ijk_i = ijk_frame.u_axis;
    Vector3 ijk_j = ijk_frame.v_axis;
    Vector3 ijk_k = ijk_frame.w_axis;
    
    // Build IJK matrix [I J K]
    // Matrix in column-major form: [I_x J_x K_x]
    //                              [I_y J_y K_y]
    //                              [I_z J_z K_z]
    double ijk_matrix[3][3] = {
        {ijk_i.x, ijk_j.x, ijk_k.x},
        {ijk_i.y, ijk_j.y, ijk_k.y},
        {ijk_i.z, ijk_j.z, ijk_k.z}
    };
    
    // Calculate determinant
    double det = ijk_matrix[0][0] * (ijk_matrix[1][1] * ijk_matrix[2][2] - ijk_matrix[1][2] * ijk_matrix[2][1]) -
                 ijk_matrix[0][1] * (ijk_matrix[1][0] * ijk_matrix[2][2] - ijk_matrix[1][2] * ijk_matrix[2][0]) +
                 ijk_matrix[0][2] * (ijk_matrix[1][0] * ijk_matrix[2][1] - ijk_matrix[1][1] * ijk_matrix[2][0]);
    
    // Calculate inverse matrix elements
    double inv_det = 1.0 / det;
    double R[3][3];
    
    R[0][0] = (ijk_matrix[1][1] * ijk_matrix[2][2] - ijk_matrix[1][2] * ijk_matrix[2][1]) * inv_det;
    R[0][1] = -(ijk_matrix[0][1] * ijk_matrix[2][2] - ijk_matrix[0][2] * ijk_matrix[2][1]) * inv_det;
    R[0][2] = (ijk_matrix[0][1] * ijk_matrix[1][2] - ijk_matrix[0][2] * ijk_matrix[1][1]) * inv_det;
    
    R[1][0] = -(ijk_matrix[1][0] * ijk_matrix[2][2] - ijk_matrix[1][2] * ijk_matrix[2][0]) * inv_det;
    R[1][1] = (ijk_matrix[0][0] * ijk_matrix[2][2] - ijk_matrix[0][2] * ijk_matrix[2][0]) * inv_det;
    R[1][2] = -(ijk_matrix[0][0] * ijk_matrix[1][2] - ijk_matrix[0][2] * ijk_matrix[1][0]) * inv_det;
    
    R[2][0] = (ijk_matrix[1][0] * ijk_matrix[2][1] - ijk_matrix[1][1] * ijk_matrix[2][0]) * inv_det;
    R[2][1] = -(ijk_matrix[0][0] * ijk_matrix[2][1] - ijk_matrix[0][1] * ijk_matrix[2][0]) * inv_det;
    R[2][2] = (ijk_matrix[0][0] * ijk_matrix[1][1] - ijk_matrix[0][1] * ijk_matrix[1][0]) * inv_det;
    
    // Apply translation to UVW origin
    Vector3 new_uvw_origin = uvw_frame.origin + translation;
    
    // Apply rotation R to UVW axes
    Vector3 new_u_axis(
        R[0][0] * uvw_frame.u_axis.x + R[0][1] * uvw_frame.u_axis.y + R[0][2] * uvw_frame.u_axis.z,
        R[1][0] * uvw_frame.u_axis.x + R[1][1] * uvw_frame.u_axis.y + R[1][2] * uvw_frame.u_axis.z,
        R[2][0] * uvw_frame.u_axis.x + R[2][1] * uvw_frame.u_axis.y + R[2][2] * uvw_frame.u_axis.z
    );
    
    Vector3 new_v_axis(
        R[0][0] * uvw_frame.v_axis.x + R[0][1] * uvw_frame.v_axis.y + R[0][2] * uvw_frame.v_axis.z,
        R[1][0] * uvw_frame.v_axis.x + R[1][1] * uvw_frame.v_axis.y + R[1][2] * uvw_frame.v_axis.z,
        R[2][0] * uvw_frame.v_axis.x + R[2][1] * uvw_frame.v_axis.y + R[2][2] * uvw_frame.v_axis.z
    );
    
    Vector3 new_w_axis(
        R[0][0] * uvw_frame.w_axis.x + R[0][1] * uvw_frame.w_axis.y + R[0][2] * uvw_frame.w_axis.z,
        R[1][0] * uvw_frame.w_axis.x + R[1][1] * uvw_frame.w_axis.y + R[1][2] * uvw_frame.w_axis.z,
        R[2][0] * uvw_frame.w_axis.x + R[2][1] * uvw_frame.w_axis.y + R[2][2] * uvw_frame.w_axis.z
    );
    
    return CoordinateFrame(new_uvw_origin, new_u_axis, new_v_axis, new_w_axis);
}

CoordinateFrame OrientationModule::translate_to_endeffector(const CoordinateFrame& aligned_frame,
                                                          const Vector3& end_effector_position) {
    // Translate coordinate frame to end-effector position
    return CoordinateFrame(end_effector_position, 
                          aligned_frame.u_axis, 
                          aligned_frame.v_axis, 
                          aligned_frame.w_axis);
}

Matrix4x4 OrientationModule::create_transformation_matrix(const CoordinateFrame& frame) {
    Matrix4x4 matrix;
    
    // Set rotation part (upper 3x3) - each axis as a column
    matrix.set_rotation(frame.u_axis, frame.v_axis, frame.w_axis);
    
    // Set translation part (last column)
    matrix.set_translation(frame.origin);
    
    // Bottom row is [0, 0, 0, 1] (already set by identity initialization)
    
    return matrix;
}

Vector3 OrientationModule::calculate_plane_normal(const Vector3& A_point,
                                                const Vector3& B_point,
                                                const Vector3& C_point) {
    // Calculate two vectors in the plane
    Vector3 AB = B_point - A_point;
    Vector3 AC = C_point - A_point;
    
    // Cross product gives normal vector - REVERSED ORDER: AC × AB instead of AB × AC
    Vector3 normal(
        AC.y * AB.z - AC.z * AB.y,
        AC.z * AB.x - AC.x * AB.z,
        AC.x * AB.y - AC.y * AB.x
    );
    
    return normal.normalized();
}

} // namespace delta