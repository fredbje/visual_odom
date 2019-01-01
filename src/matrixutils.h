#ifndef VISUALODOMETRY_MATRIXUTILS_H
#define VISUALODOMETRY_MATRIXUTILS_H

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <gtsam/geometry/Pose3.h>
#include <Eigen/Dense>

template <typename T>
inline bool isRotationMatrix(const cv::Matx<T, 3, 3>& R)
{
//    cv::Matx33f Rt;
//    cv::transpose(R, Rt);
//    cv::Matx33f shouldBeIdentity = Rt * R;
//    cv::Matx33f I = cv::Matx33f::eye(3);

    return norm(cv::Matx<T, 3, 3>::eye(), R.t() * R) < 1e-6;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
template <typename T>
inline cv::Vec<T, 3> rotationMatrixToEulerAngles(const cv::Matx<T, 3, 3>& R)
{
    assert(isRotationMatrix(R));

    T sy = sqrt(R(0,0) * R(0,0) +  R(1,0) * R(1,0) );

    bool singular = sy < 1e-6; // If

    T x, y, z;
    if (!singular)
    {
        x = atan2(R(2,1) , R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2(R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    return cv::Vec<T, 3>(x, y, z);
}
template <typename T>
inline cv::Matx<T, 3, 3> euler2rot(const cv::Vec<T, 3>& euler)
{
    T x = euler(0);
    T y = euler(1);
    T z = euler(2);

    // Assuming the angles are in radians.
    T ch = cos(z);
    T sh = sin(z);
    T ca = cos(y);
    T sa = sin(y);
    T cb = cos(x);
    T sb = sin(x);

    T m00, m01, m02, m10, m11, m12, m20, m21, m22;

    m00 = ch * ca;
    m01 = sh*sb - ch*sa*cb;
    m02 = ch*sa*sb + sh*cb;
    m10 = sa;
    m11 = ca*cb;
    m12 = -ca*sb;
    m20 = -sh*ca;
    m21 = sh*sa*cb + ch*sb;
    m22 = -sh*sa*sb + ch*cb;

    cv::Matx<T, 3, 3> R;
    R(0,0) = m00;
    R(0,1) = m01;
    R(0,2) = m02;
    R(1,0) = m10;
    R(1,1) = m11;
    R(1,2) = m12;
    R(2,0) = m20;
    R(2,1) = m21;
    R(2,2) = m22;
    return R;
}

template <typename T>
inline cv::Matx<T, 3, 3> rotMatX(const T& angle)
{
    T s = sin(angle);
    T c = cos(angle);
    cv::Matx<T, 3, 3> R = cv::Matx<T, 3, 3>::eye();
    R(1, 1) = c;
    R(1, 2) = -s;
    R(2, 1) = s;
    R(2, 2) = c;
    return R;
}

template <typename T>
inline cv::Matx<T, 3, 3> rotMatY(const T& angle)
{
    T s = sin(angle);
    T c = cos(angle);
    cv::Matx<T, 3, 3> R = cv::Matx<T, 3, 3>::eye();
    R(0, 0) = c;
    R(0, 2) = s;
    R(2, 0) = -s;
    R(2, 2) = c;
    return R;
}

template <typename T>
inline cv::Matx<T, 3, 3> rotMatZ(const T& angle)
{
    T s = sin(angle);
    T c = cos(angle);
    cv::Matx<T, 3, 3> R = cv::Matx<T, 3, 3>::eye();
    R(0, 0) = c;
    R(0, 1) = -s;
    R(1, 0) = s;
    R(1, 1) = c;
    return R;
}

template <typename T>
inline cv::Matx<T, 4, 1> rotMat2Quat(const cv::Matx<T, 3, 3>& R)
{
    // Returned quaternion is 4x1 (qw, qx, qy, qz)^T.
    cv::Matx<T, 4, 1> q;
    q(0) = sqrt(1.0 + R(0, 0) + R(1, 1) + R(2, 2)) / 2.0;
    q(1) = (R(2, 1) - R(1, 2)) / (4.0 * q(0));
    q(2) = (R(0, 2) - R(2, 0)) / (4.0 * q(0));
    q(3) = (R(1, 0) - R(0, 1)) / (4.0 * q(0));
    return q;
}

inline cv::Matx33d quat2RotMat(const cv::Vec4d& q)
{
    // Input quaternion must be 4x1 (qw, qx, qy, qz)^T.
    cv::Matx33d R;
    double sqw = q(0) * q(0);
    double sqx = q(1) * q(1);
    double sqy = q(2) * q(2);
    double sqz = q(3) * q(3);

    // invs (inverse square length) is only required if quaternion is not already normalised
    double invs = 1.0 / (sqx + sqy + sqz + sqw);
    R(0, 0) = ( sqx - sqy - sqz + sqw)*invs; // since sqw + sqx + sqy + sqz =1/invs*invs
    R(1, 1) = (-sqx + sqy - sqz + sqw)*invs;
    R(2, 2) = (-sqx - sqy + sqz + sqw)*invs;

    R(1, 0) = 2.0 * (q(1)*q(2) + q(3)*q(0)) * invs;
    R(0, 1) = 2.0 * (q(1)*q(2) - q(3)*q(0)) * invs;

    R(2, 0) = 2.0 * (q(1)*q(3) - q(2)*q(0)) * invs;
    R(0, 2) = 2.0 * (q(1)*q(3) + q(2)*q(0)) * invs;

    R(2, 1) = 2.0 * (q(2)*q(3) + q(1)*q(0))*invs;
    R(1, 2) = 2.0 * (q(2)*q(3) - q(1)*q(0))*invs;

    return R;
}

template <typename T>
inline gtsam::Rot3 rotation2Rot3(const cv::Matx<T, 3, 3>& rotation)
{
    gtsam::Rot3 R(
            rotation(0, 0),
            rotation(0, 1),
            rotation(0, 2),

            rotation(1, 0),
            rotation(1, 1),
            rotation(1, 2),

            rotation(2, 0),
            rotation(2, 1),
            rotation(2, 2)
    );
    return R;
}

template <typename T>
inline void rotation2Rot3(const cv::Matx<T, 3, 3>& rotation, gtsam::Rot3& R)
{
    R = gtsam::Rot3(
            rotation(0, 0),
            rotation(0, 1),
            rotation(0, 2),

            rotation(1, 0),
            rotation(1, 1),
            rotation(1, 2),

            rotation(2, 0),
            rotation(2, 1),
            rotation(2, 2)
    );
}

template <typename T>
inline gtsam::Point3 translation2Point3(const cv::Matx<T, 3, 1>& translation)
{
    gtsam::Point3 t(
            translation(0),
            translation(1),
            translation(2)
    );
    return t;
}

template <typename T>
inline void translation2Point3(const cv::Matx<T, 3, 1>& translation, gtsam::Point3& t)
{
    t = gtsam::Point3(
            translation(0),
            translation(1),
            translation(2)
    );
}


template <typename T>
inline gtsam::Pose3 pose2Pose3(const cv::Matx<T, 4, 4>& poseCv)
{
    Eigen::Matrix4d pose;
    cv::cv2eigen(poseCv, pose);
    gtsam::Pose3 gtsamPose(
            pose
    );
    return gtsamPose;
}

template <typename T>
inline void pose2Pose3(const cv::Matx<T, 4, 4>& cvPose, gtsam::Pose3& gtsamPose)
{
    Eigen::Matrix4d pose;
    cv::cv2eigen(cvPose, pose);
    gtsamPose = gtsam::Pose3(pose);
}

inline cv::Matx<double, 4, 4> pose32pose(const gtsam::Pose3& poseGtsam)
{
    Eigen::Matrix4d pose = poseGtsam.matrix();
    cv::Matx<double, 4, 4> T(
            pose(0, 0),
            pose(0, 1),
            pose(0, 2),
            pose(0, 3),

            pose(1, 0),
            pose(1, 1),
            pose(1, 2),
            pose(1, 3),

            pose(2, 0),
            pose(2, 1),
            pose(2, 2),
            pose(2, 3),

            pose(3, 0),
            pose(3, 1),
            pose(3, 2),
            pose(3, 3)
    );
    return T;
}

#endif //VISUALODOMETRY_MATRIXUTILS_H
