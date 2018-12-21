#ifndef VISUAL_ODOM_H
#define VISUAL_ODOM_H

#include <string>
#include <opencv2/calib3d.hpp>
#include "feature.h"
#include "stereocamera.h"

class VisualOdometryStereo
{
public:
    explicit VisualOdometryStereo(StereoCamera stereoCamera) : stereoCamera_(std::move(stereoCamera)) {}
    ~VisualOdometryStereo() = default;

    void process(cv::Mat& rotation, cv::Mat& translation_stereo,
            cv::Mat& image_left_t1,
            cv::Mat& image_right_t1,
            cv::Mat& image_left_t0,
            cv::Mat& image_right_t0,
            std::vector<cv::Point2f>& points_left_t0,
            std::vector<cv::Point2f>& points_right_t0,
            std::vector<cv::Point2f>& points_left_t1,
            std::vector<cv::Point2f>& points_right_t1,
            std::vector<cv::Point2f>& points_left_t0_return,
            FeatureSet& current_features);

private:
    void removeInvalidPoints(std::vector<cv::Point2f>& points, const std::vector<bool>& status);
    void checkValidMatch(std::vector<cv::Point2f>& points, std::vector<cv::Point2f>& points_return, std::vector<bool>& status);

private:
    StereoCamera stereoCamera_;
    cv::Mat translationMonoIgnored_ = cv::Mat::zeros(3, 1, CV_64F);

    // Settings for solvePnpRansac
    const int iterationsCount_ = 500;        // number of Ransac iterations.
    const float reprojectionError_ = 2.0;    // maximum allowed distance to consider it an inlier.
    const float confidence_ = 0.95;          // RANSAC successful confidence.
    const bool useExtrinsicGuess_ = true;
    const int flags_ = cv::SOLVEPNP_P3P; // cv::SOLVEPNP_ITERATIVE;
    cv::Mat rvecIgnored_ = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat inliersIgnored_;

    // Settings for bucketing features
    const int bucketSize_ = 50;
    const int featuresPerBucket_ = 4;
};



#endif
