#include "orientation_module.hpp"
#include <chrono>
#include <cmath>

namespace delta {

OrientationResult OrientationModule::calculate(double x, double y, double z) {
    return calculate(Vector3(x, y, z));
}

OrientationResult OrientationModule::calculate(const Vector3& input_vector) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // NEW: Validate input (Step 1.2)
    if (!is_input_valid(input_vector)) {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        return OrientationResult::failed(input_vector, duration.count() / 1000.0);
    }
    
    // Perform calculation
    OrientationResult result = calculate_internal(input_vector);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    result.computation_time_ms = duration.count() / 1000.0;
    
    return result;
}

OrientationResult OrientationModule::calculate_from_kinematics(const KinematicsResult& kinematics_result) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Check if kinematics calculation was successful
    if (!kinematics_result.calculation_successful) {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        return OrientationResult::failed(kinematics_result.original_input, duration.count() / 1000.0);
    }
    
    // Perform calculation using existing kinematics result
    OrientationResult result = calculate_from_kinematics_internal(kinematics_result);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    result.computation_time_ms = duration.count() / 1000.0;
    
    return result;
}

bool OrientationModule::is_input_valid(const Vector3& input_vector) {
    // Use same validation as KinematicsModule
    return KinematicsModule::is_input_valid(input_vector);
}

OrientationResult OrientationModule::calculate_internal(const Vector3& input_vector) {
    // Get kinematics result first using improved KinematicsModule (Step 1.2)
    KinematicsResult kinematics = KinematicsModule::calculate(input_vector);
    
    // Check if kinematics calculation was successful
    if (!kinematics.calculation_successful) {
        return OrientationResult::failed(input_vector, kinematics.computation_time_ms);
    }
    
    return calculate_from_kinematics_internal(kinematics);
}

OrientationResult OrientationModule::calculate_from_kinematics_internal(const KinematicsResult& kinematics_result) {
    // Get required points from kinematics result
    Vector3 fermat_point = kinematics_result.fermat_data.fermat_point;
    Vector3 end_effector = kinematics_result.end_effector_position;
    
    // Get A, B, C points from fermat data
    Vector3 base_A = get_base_position_A();
    Vector3 base_B = get_base_position_B();
    Vector3 base_C = get_base_position_C();
    
    Vector3 A_point(base_A.x(), base_A.y(), kinematics_result.fermat_data.z_A);
    Vector3 B_point(base_B.x(), base_B.y(), kinematics_result.fermat_data.z_B);
    Vector3 C_point(base_C.x(), base_C.y(), kinematics_result.fermat_data.z_C);
    
    // Step 1: Create UVW coordinate system at Fermat point
    CoordinateFrame UVW_frame = create_UVW_frame(fermat_point, A_point, B_point, C_point);
    
    // Step 2: Mirror UVW across XY plane to get IJK
    CoordinateFrame IJK_frame = mirror_across_XY(UVW_frame);
    
    // Step 3: Translate to align with XYZ origin, get U'V'W'
    CoordinateFrame aligned_frame = align_with_origin(UVW_frame, IJK_frame);
    
    // Step 4: Translate to end-effector position to get U''V''W''
    CoordinateFrame final_frame = translate_to_endeffector(aligned_frame, end_effector);
    
    // Create transformation matrix
    Matrix4 transformation = create_transformation_matrix(final_frame);
    
    // Return all coordinate frames
    return OrientationResult(transformation, fermat_point, end_effector, 
                           UVW_frame, IJK_frame, aligned_frame, final_frame);
}

CoordinateFrame OrientationModule::create_UVW_frame(const Vector3& fermat_point,
                                                   const Vector3& A_point,
                                                   const Vector3& B_point,
                                                   const Vector3& C_point) {
    
    // V-axis: Fermat point → A point (aligns with Y-axis since A is on Y-axis)
    Vector3 v_axis = (A_point - fermat_point).normalized();
    
    // W-axis: Normal to ABC plane (pointing +Z direction)
    Vector3 w_axis = calculate_plane_normal(A_point, B_point, C_point);
    // Ensure W points in +Z direction
    if (w_axis.z() < 0) {
        w_axis = -w_axis;
    }
    
    // U-axis: V × W (cross product) - guarantees perfect orthogonality AND right-handed system
    Vector3 u_axis = v_axis.cross(w_axis).normalized();
    
    return CoordinateFrame(fermat_point, u_axis, v_axis, w_axis);
}

CoordinateFrame OrientationModule::mirror_across_XY(const CoordinateFrame& uvw_frame) {
    // Mirror across XY plane: (x,y,z) → (x,y,-z)
    Vector3 mirrored_origin(uvw_frame.origin.x(), uvw_frame.origin.y(), -uvw_frame.origin.z());
    Vector3 mirrored_u(uvw_frame.u_axis.x(), uvw_frame.u_axis.y(), -uvw_frame.u_axis.z());
    Vector3 mirrored_v(uvw_frame.v_axis.x(), uvw_frame.v_axis.y(), -uvw_frame.v_axis.z());
    Vector3 mirrored_w(uvw_frame.w_axis.x(), uvw_frame.w_axis.y(), -uvw_frame.w_axis.z());
    
    // THEN invert W to make K point upward (create IJK from mirrored UVW)
    Vector3 inverted_k = -mirrored_w;
    
    return CoordinateFrame(mirrored_origin, mirrored_u, mirrored_v, inverted_k);
}

CoordinateFrame OrientationModule::align_with_origin(const CoordinateFrame& uvw_frame,
                                                    const CoordinateFrame& ijk_frame) {
    // Calculate transformation matrix to align IJK with XYZ
    
    // 1. Translation: move IJK origin to XYZ origin (0,0,0)
    Vector3 translation = Vector3(0, 0, 0) - ijk_frame.origin;
    
    // 2. Rotation: align IJK axes with XYZ axes
    // Build IJK matrix [I J K] as column vectors
    Matrix3 ijk_matrix;
    ijk_matrix.col(0) = ijk_frame.u_axis;  // I column
    ijk_matrix.col(1) = ijk_frame.v_axis;  // J column  
    ijk_matrix.col(2) = ijk_frame.w_axis;  // K column
    
    // Calculate rotation matrix R = [I J K]^(-1)
    Matrix3 R = ijk_matrix.inverse();
    
    // Apply translation to UVW origin
    Vector3 new_uvw_origin = uvw_frame.origin + translation;
    
    // Apply rotation R to UVW axes
    Vector3 new_u_axis = R * uvw_frame.u_axis;
    Vector3 new_v_axis = R * uvw_frame.v_axis;
    Vector3 new_w_axis = R * uvw_frame.w_axis;
    
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

Matrix4 OrientationModule::create_transformation_matrix(const CoordinateFrame& frame) {
    Matrix4 matrix = Matrix4::Identity();
    
    // Set rotation part (upper 3x3) - each axis as a column
    matrix.block<3,3>(0,0).col(0) = frame.u_axis;
    matrix.block<3,3>(0,0).col(1) = frame.v_axis;
    matrix.block<3,3>(0,0).col(2) = frame.w_axis;
    
    // Set translation part (last column, first 3 rows)
    matrix.block<3,1>(0,3) = frame.origin;
    
    // Bottom row is [0, 0, 0, 1] (already set by Identity)
    
    return matrix;
}

Vector3 OrientationModule::calculate_plane_normal(const Vector3& A_point,
                                                const Vector3& B_point,
                                                const Vector3& C_point) {
    // Calculate two vectors in the plane
    Vector3 AB = B_point - A_point;
    Vector3 AC = C_point - A_point;
    
    // Cross product gives normal vector - REVERSED ORDER: AC × AB instead of AB × AC
    Vector3 normal = AC.cross(AB);
    
    return normal.normalized();
}

} // namespace delta