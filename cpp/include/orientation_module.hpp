#ifndef DELTA_ORIENTATION_MODULE_HPP
#define DELTA_ORIENTATION_MODULE_HPP

#include "math_utils.hpp"
#include "kinematics_module.hpp"

namespace delta {

// 4x4 Transformation Matrix structure
struct Matrix4x4 {
    double data[4][4];
    
    Matrix4x4() {
        // Initialize as identity matrix
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                data[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
    }
    
    // Access operators
    double* operator[](int row) { return data[row]; }
    const double* operator[](int row) const { return data[row]; }
    
    // Set translation (last column)
    void set_translation(const Vector3& translation) {
        data[0][3] = translation.x;
        data[1][3] = translation.y;
        data[2][3] = translation.z;
    }
    
    // Set rotation (upper 3x3)
    void set_rotation(const Vector3& u_axis, const Vector3& v_axis, const Vector3& w_axis) {
        // Column-major: each axis is a column
        data[0][0] = u_axis.x; data[0][1] = v_axis.x; data[0][2] = w_axis.x;
        data[1][0] = u_axis.y; data[1][1] = v_axis.y; data[1][2] = w_axis.y;
        data[2][0] = u_axis.z; data[2][1] = v_axis.z; data[2][2] = w_axis.z;
    }
};

// Coordinate Frame structure
struct CoordinateFrame {
    Vector3 origin;
    Vector3 u_axis;
    Vector3 v_axis;
    Vector3 w_axis;
    
    CoordinateFrame(const Vector3& orig = Vector3(0,0,0),
                    const Vector3& u = Vector3(1,0,0),
                    const Vector3& v = Vector3(0,1,0),
                    const Vector3& w = Vector3(0,0,1))
        : origin(orig), u_axis(u), v_axis(v), w_axis(w) {}
};

struct OrientationResult {
    Matrix4x4 transformation_matrix;        // Position + orientation combined
    
    // Reference data (for debugging/validation)
    Vector3 fermat_point;
    Vector3 end_effector_position;
    
    // NEW! All coordinate frames in transformation sequence
    CoordinateFrame UVW_at_fermat;          // Step 1: UVW frame at Fermat point
    CoordinateFrame IJK_mirrored;           // Step 2: IJK frame (mirrored across XY)
    CoordinateFrame UVW_prime_aligned;      // Step 3: U'V'W' frame (aligned with origin)
    CoordinateFrame final_frame;            // Step 4: U''V''W'' frame (at end-effector)
    
    OrientationResult(const Matrix4x4& matrix, const Vector3& fermat,
                     const Vector3& end_effector, const CoordinateFrame& uvw,
                     const CoordinateFrame& ijk, const CoordinateFrame& uvw_prime,
                     const CoordinateFrame& final)
        : transformation_matrix(matrix), fermat_point(fermat)
        , end_effector_position(end_effector), UVW_at_fermat(uvw)
        , IJK_mirrored(ijk), UVW_prime_aligned(uvw_prime), final_frame(final) {}
};

class OrientationModule {
public:
    // Main interface: input direction vector, get transformation matrix
    static OrientationResult calculate(double x, double y, double z);
    static OrientationResult calculate(const Vector3& input_vector);
    
    // Alternative: use existing kinematics result
    static OrientationResult calculate_from_kinematics(const KinematicsResult& kinematics_result);
    
private:
    // Step 1: Create UVW coordinate system at Fermat point
    static CoordinateFrame create_UVW_frame(const Vector3& fermat_point,
                                           const Vector3& A_point,
                                           const Vector3& B_point,
                                           const Vector3& C_point);
    
    // Step 2: Mirror UVW across XY plane to get IJK
    static CoordinateFrame mirror_across_XY(const CoordinateFrame& uvw_frame);
    
    // Step 3: Translate IJK to align with XYZ origin, get U'V'W'
    static CoordinateFrame align_with_origin(const CoordinateFrame& uvw_frame,
                                           const CoordinateFrame& ijk_frame);
    
    // Step 4: Translate to end-effector position to get U''V''W''
    static CoordinateFrame translate_to_endeffector(const CoordinateFrame& aligned_frame,
                                                   const Vector3& end_effector_position);
    
    // Helper: Create transformation matrix from coordinate frame
    static Matrix4x4 create_transformation_matrix(const CoordinateFrame& frame);
    
    // Helper: Calculate normal to ABC plane
    static Vector3 calculate_plane_normal(const Vector3& A_point,
                                        const Vector3& B_point,
                                        const Vector3& C_point);
};

} // namespace delta

#endif // DELTA_ORIENTATION_MODULE_HPP