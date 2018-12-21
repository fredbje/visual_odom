#ifndef VISUALODOMETRY_STEREOCAMERA_H
#define VISUALODOMETRY_STEREOCAMERA_H

#include <opencv2/core/mat.hpp>

class StereoCamera
{
public:
    StereoCamera(float f, float cx, float cy, float bf, int width, int height)
    : f_(f), cx_(cx), cy_(cy), bf_(bf), width_(width), height_(height)
    {
        projMatL_ = (cv::Mat_<float>(3, 4) << f, 0., cx, 0., 0., f, cy, 0., 0,  0., 1., 0.);
        projMatR_ = (cv::Mat_<float>(3, 4) << f, 0., cx, bf, 0., f, cy, 0., 0,  0., 1., 0.);
        K_ = (cv::Mat_<float>(3, 3) << f, 0., cx, 0., f, cy, 0., 0., 1.);
        principalPoint_ = cv::Point2f(cx, cy);
        distCoeffs_ = cv::Mat::zeros(4, 1, CV_64FC1);
    }

    const float& f() { return f_; }
    const float& cx() { return cx_; }
    const float& cy() { return cy_; }
    const float& bf() { return bf_; }
    const int& width() { return width_; }
    const int& height() { return height_; }

    const cv::Mat& projMatL() { return projMatL_; }
    const cv::Mat& projMatR() { return projMatR_; }
    const cv::Mat& K() { return K_; }
    const cv::Point2f& principalPoint() { return principalPoint_; }
    const cv::Mat& distCoeffs() { return distCoeffs_; }

private:
    float f_;
    float cx_;
    float cy_;
    float bf_;
    int width_;
    int height_;

    cv::Mat projMatL_;
    cv::Mat projMatR_;
    cv::Mat K_;
    cv::Point2f principalPoint_;
    cv::Mat distCoeffs_;
};

#endif //VISUALODOMETRY_STEREOCAMERA_H
