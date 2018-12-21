#ifndef VISUALODOMETRY_STEREOCAMERA_H
#define VISUALODOMETRY_STEREOCAMERA_H

#include <opencv2/core/mat.hpp>
#include "easylogging++.h"

class StereoCamera
{
public:
    StereoCamera(float fx, float fy, float cx, float cy, float bf, int width, int height,
            float k1 = 0, float k2 = 0, float p1 = 0, float p2 = 0)
    : fx_(fx), fy_(fy), cx_(cx), cy_(cy), bf_(bf), width_(width), height_(height), k1_(k1), k2_(k2), p1_(p1), p2_(p2)
    {
        projMatL_ = (cv::Mat_<float>(3, 4) << fx, 0., cx, 0., 0., fy, cy, 0., 0,  0., 1., 0.);
        projMatR_ = (cv::Mat_<float>(3, 4) << fx, 0., cx, bf, 0., fy, cy, 0., 0,  0., 1., 0.);
        K_ = (cv::Mat_<float>(3, 3) << fx, 0., cx, 0., fy, cy, 0., 0., 1.);
        principalPoint_ = cv::Point2f(cx, cy);
        distCoeffs_ = (cv::Mat_<float>(4, 1) << k1_, k2_, p1_, p2_);

        if( fx_ <= 0 || fy_ <= 0 || cx_ <= 0 || cy_ <= 0)
        {
            LOG(ERROR) << "Camera intrinsic parameters fx, fy, cx and cy must be positive";
            return;
        }

        if( (height_) <= 0 || width_ <= 0)
        {
            LOG(ERROR) << "Image height and width must be positive";
            return;
        }

        if(bf_ >= 0)
        {
            LOG(WARNING) << "Parameter bf from config should be negative";
        }

        LOG(INFO) << "P_left: " << std::endl << projMatL_;
        LOG(INFO) << "P_right: " << std::endl << projMatR_;
    }

    explicit StereoCamera(const cv::FileStorage& fSettings)
    : StereoCamera(fSettings["Camera.fx"],
            fSettings["Camera.fy"],
            fSettings["Camera.cx"],
            fSettings["Camera.cy"],
            fSettings["Camera.bf"],
            fSettings["Camera.width"],
            fSettings["Camera.height"],
            fSettings["Camera.k1"],
            fSettings["Camera.k2"],
            fSettings["Camera.p1"],
            fSettings["Camera.p2"])
            {

            }

    const float& fx() { return fx_; }
    const float& fy() { return fy_; }
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
    float fx_;
    float fy_;
    float cx_;
    float cy_;
    float bf_;
    int width_;
    int height_;

    float k1_;
    float k2_;
    float p1_;
    float p2_;

    cv::Mat projMatL_;
    cv::Mat projMatR_;
    cv::Mat K_;
    cv::Point2f principalPoint_;
    cv::Mat distCoeffs_;
};

#endif //VISUALODOMETRY_STEREOCAMERA_H
