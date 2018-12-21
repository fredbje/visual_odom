#ifndef VISUALODOMETRY_MATRIXUTILS_H
#define VISUALODOMETRY_MATRIXUTILS_H

#include <opencv2/core/core.hpp>

inline bool isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

    return  norm(I, shouldBeIdentity) < 1e-6;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
inline cv::Vec3d rotationMatrixToEulerAngles(cv::Mat &R)
{
    assert(isRotationMatrix(R));

    double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    double x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3d(x, y, z);
}

inline cv::Mat euler2rot(cv::Mat& rotationMatrix, const cv::Mat & euler)
{
    double x = euler.at<double>(0);
    double y = euler.at<double>(1);
    double z = euler.at<double>(2);

    // Assuming the angles are in radians.
    double ch = cos(z);
    double sh = sin(z);
    double ca = cos(y);
    double sa = sin(y);
    double cb = cos(x);
    double sb = sin(x);

    double m00, m01, m02, m10, m11, m12, m20, m21, m22;

    m00 = ch * ca;
    m01 = sh*sb - ch*sa*cb;
    m02 = ch*sa*sb + sh*cb;
    m10 = sa;
    m11 = ca*cb;
    m12 = -ca*sb;
    m20 = -sh*ca;
    m21 = sh*sa*cb + ch*sb;
    m22 = -sh*sa*sb + ch*cb;

    rotationMatrix.at<double>(0,0) = m00;
    rotationMatrix.at<double>(0,1) = m01;
    rotationMatrix.at<double>(0,2) = m02;
    rotationMatrix.at<double>(1,0) = m10;
    rotationMatrix.at<double>(1,1) = m11;
    rotationMatrix.at<double>(1,2) = m12;
    rotationMatrix.at<double>(2,0) = m20;
    rotationMatrix.at<double>(2,1) = m21;
    rotationMatrix.at<double>(2,2) = m22;

    return rotationMatrix;
}

inline cv::Mat rotMatX(const float& angle)
{
    float s = sinf(angle);
    float c = cosf(angle);
    cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
    R.at<float>(1, 1) = c;
    R.at<float>(1, 2) = -s;
    R.at<float>(2, 1) = s;
    R.at<float>(2, 2) = c;
    return R;
}

inline cv::Mat rotMatY(const float& angle)
{
    float s = sinf(angle);
    float c = cosf(angle);
    cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
    R.at<float>(0, 0) = c;
    R.at<float>(0, 2) = s;
    R.at<float>(2, 0) = -s;
    R.at<float>(2, 2) = c;
    return R;
}

inline cv::Mat rotMatZ(const float& angle)
{
    float s = sinf(angle);
    float c = cosf(angle);
    cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
    R.at<float>(0, 0) = c;
    R.at<float>(0, 1) = -s;
    R.at<float>(1, 0) = s;
    R.at<float>(1, 1) = c;
    return R;
}

#endif //VISUALODOMETRY_MATRIXUTILS_H
