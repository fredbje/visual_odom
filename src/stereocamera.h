#ifndef VISUALODOMETRY_STEREOCAMERA_H
#define VISUALODOMETRY_STEREOCAMERA_H

#include <opencv2/core/mat.hpp>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/StereoCamera.h>
#include "easylogging++.h"


class StereoCamera {
public:
    StereoCamera(float fx, float fy, float cx, float cy, float bf, int width, int height,
                 float k1 = 0, float k2 = 0, float p1 = 0, float p2 = 0)
            : fx_(fx), fy_(fy), cx_(cx), cy_(cy), bf_(bf), base_(-bf / fx), width_(width), height_(height), k1_(k1),
              k2_(k2), p1_(p1), p2_(p2) {
        projMatL_ = cv::Matx<float, 3, 4>(fx, 0.f, cx, 0.f, 0.f, fy, cy, 0.f, 0, 0.f, 1.f, 0.f);
        projMatR_ = cv::Matx<float, 3, 4>(fx, 0.f, cx, bf, 0.f, fy, cy, 0.f, 0, 0.f, 1.f, 0.f);
        K_ = cv::Matx<float, 3, 3>(fx, 0.f, cx, 0.f, fy, cy, 0.f, 0.f, 1.f);
        principalPoint_ = cv::Point2f(cx, cy);
        distCoeffs_ = cv::Matx<float, 4, 1>(k1_, k2_, p1_, p2_);

        // construct the stereo calibration shared pointer, no need to delete it
        cal3Stereo_ = gtsam::Cal3_S2Stereo::shared_ptr(new gtsam::Cal3_S2Stereo(fx_, fy_, 0, cx_, cy_, base_));
        cal3L_ = gtsam::Cal3_S2::shared_ptr(new gtsam::Cal3_S2(fx_, fy_, 0, cx_, cy_));
        cal3R_ = gtsam::Cal3_S2::shared_ptr(new gtsam::Cal3_S2(fx_, fy_, 0, cx_, cy_));

        if (fx_ <= 0 || fy_ <= 0 || cx_ <= 0 || cy_ <= 0) {
            LOG(ERROR) << "Camera intrinsic parameters fx, fy, cx and cy must be positive";
            return;
        }

        if ((height_) <= 0 || width_ <= 0) {
            LOG(ERROR) << "Image height and width must be positive";
            return;
        }

        if (bf_ >= 0) {
            LOG(WARNING) << "Parameter bf from config should be negative";
        }

        LOG(INFO) << "P_left: " << std::endl << projMatL_;
        LOG(INFO) << "P_right: " << std::endl << projMatR_;
    }

    explicit StereoCamera(const cv::FileStorage &fSettings)
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
                           fSettings["Camera.p2"]) {

    }


    const float &fx() { return fx_; }

    const float &fy() { return fy_; }

    const float &cx() { return cx_; }

    const float &cy() { return cy_; }

    const float &bf() { return bf_; }

    const int &width() { return width_; }

    const int &height() { return height_; }

    const cv::Matx<float, 3, 4> &projMatL() { return projMatL_; }

    const cv::Matx<float, 3, 4> &projMatR() { return projMatR_; }

    const cv::Matx<float, 3, 3> &K() { return K_; }

    const cv::Point_<float> &principalPoint() { return principalPoint_; }

    const cv::Matx<float, 4, 1> &distCoeffs() { return distCoeffs_; }

    const gtsam::Cal3_S2Stereo::shared_ptr cal3Stereo() { return cal3Stereo_; }

private:
    float fx_;
    float fy_;
    float cx_;
    float cy_;
    float bf_;
    float base_;
    int width_;
    int height_;

    float k1_;
    float k2_;
    float p1_;
    float p2_;

    cv::Matx<float, 3, 4> projMatL_;
    cv::Matx<float, 3, 4> projMatR_;
    cv::Matx<float, 3, 3> K_;
    cv::Point_<float> principalPoint_;
    cv::Matx<float, 4, 1> distCoeffs_;

    gtsam::Cal3_S2Stereo::shared_ptr cal3Stereo_;
    gtsam::Cal3_S2::shared_ptr cal3L_, cal3R_;
};

#endif //VISUALODOMETRY_STEREOCAMERA_H



