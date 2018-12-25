#ifndef VISUALODOMETRY_STEREOCAMERA_H
#define VISUALODOMETRY_STEREOCAMERA_H

#include <opencv2/core/mat.hpp>
#include "easylogging++.h"


template <typename T>
class StereoCamera
{
public:
    StereoCamera(T fx, T fy, T cx, T cy, T bf, int width, int height,
            T k1 = 0, T k2 = 0, T p1 = 0, T p2 = 0)
    : fx_(fx), fy_(fy), cx_(cx), cy_(cy), bf_(bf), width_(width), height_(height), k1_(k1), k2_(k2), p1_(p1), p2_(p2)
    {
        projMatL_ = cv::Matx<T, 3, 4>(fx, 0., cx, 0., 0., fy, cy, 0., 0,  0., 1., 0.);
        projMatR_ = cv::Matx<T, 3, 4>(fx, 0., cx, bf, 0., fy, cy, 0., 0,  0., 1., 0.);
        K_ = cv::Matx<T, 3, 3>(fx, 0., cx, 0., fy, cy, 0., 0., 1.);
        principalPoint_ = cv::Point_<T>(cx, cy);
        distCoeffs_ = cv::Matx<T, 4, 1>(k1_, k2_, p1_, p2_);

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

    const T& fx() { return fx_; }
    const T& fy() { return fy_; }
    const T& cx() { return cx_; }
    const T& cy() { return cy_; }
    const T& bf() { return bf_; }
    const int& width() { return width_; }
    const int& height() { return height_; }

    const cv::Matx<T, 3, 4>& projMatL() { return projMatL_; }
    const cv::Matx<T, 3, 4>& projMatR() { return projMatR_; }
    const cv::Matx<T, 3, 3>& K() { return K_; }
    const cv::Point_<T>& principalPoint() { return principalPoint_; }
    const cv::Matx<T, 4, 1>& distCoeffs() { return distCoeffs_; }

private:
    T fx_;
    T fy_;
    T cx_;
    T cy_;
    T bf_;
    int width_;
    int height_;

    T k1_;
    T k2_;
    T p1_;
    T p2_;

    cv::Matx<T, 3, 4> projMatL_;
    cv::Matx<T, 3, 4> projMatR_;
    cv::Matx<T, 3, 3> K_;
    cv::Point_<T> principalPoint_;
    cv::Matx<T, 4, 1> distCoeffs_;
};

#endif //VISUALODOMETRY_STEREOCAMERA_H
