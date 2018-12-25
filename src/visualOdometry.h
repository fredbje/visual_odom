#ifndef VISUAL_ODOM_H
#define VISUAL_ODOM_H

#include <string>
#include <opencv2/calib3d.hpp>
#include "featureset.h"
#include "stereocamera.h"

typedef float FLOAT;

class VisualOdometryStereo
{
public:
    explicit VisualOdometryStereo(cv::FileStorage& fSettings) : stereoCamera_(fSettings) {}
    ~VisualOdometryStereo() = default;

    bool process(cv::Matx33d& rotation, cv::Vec3d& translation_stereo,
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

    void featureDetectionFast(const cv::Mat& image, std::vector<cv::Point2f>& points);

    void deleteUnmatchFeaturesCircle(std::vector<cv::Point2f>& points0, std::vector<cv::Point2f>& points1,
                                     std::vector<cv::Point2f>& points2, std::vector<cv::Point2f>& points3,
                                     std::vector<cv::Point2f>& points0_return,
                                     std::vector<uchar>& status0, std::vector<uchar>& status1,
                                     std::vector<uchar>& status2, std::vector<uchar>& status3,
                                     std::vector<int>& ages);

    void circularMatching(const cv::Mat& img_l_0, const cv::Mat& img_r_0,
            const cv::Mat& img_l_1, const cv::Mat& img_r_1,
            std::vector<cv::Point2f>& points_l_0, std::vector<cv::Point2f>& points_r_0,
            std::vector<cv::Point2f>& points_l_1, std::vector<cv::Point2f>& points_r_1,
            std::vector<cv::Point2f>& points_l_0_return,
            FeatureSet& current_features);

    void bucketingFeatures(int image_height, int image_width, FeatureSet& current_features, int bucket_size, int features_per_bucket);

    void appendNewFeatures(cv::Mat& image, FeatureSet& current_features);

private:
    void removeInvalidPoints(std::vector<cv::Point2f>& points, const std::vector<bool>& status);
    void checkValidMatch(std::vector<cv::Point2f>& points, std::vector<cv::Point2f>& points_return, std::vector<bool>& status);

private:
    StereoCamera stereoCamera_;
    cv::Mat translationMonoIgnored_ = cv::Mat::zeros(3, 1, CV_64F);

    // ---------------------------
    // Settings for solvePnpRansac
    // ---------------------------
    int cudaPnpRansac_ = true;
    int iterationsCount_ = 500;        // number of Ransac iterations.
    float reprojectionError_ = 2.0;    // maximum allowed distance to consider it an inlier.
    float confidence_ = 0.95;          // RANSAC successful confidence.
    bool useExtrinsicGuess_ = true;
    int flags_ = cv::SOLVEPNP_P3P; // cv::SOLVEPNP_ITERATIVE;
    cv::Mat rvec_ = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat inliersIgnored_;

    // -------------------------------
    // Settings for bucketing features
    // -------------------------------
    int bucketSize_ = 50;
    int featuresPerBucket_ = 4;

    // --------------------------
    // Settings for FAST detector
    // --------------------------
    int fastThreshold_ = 20;
    bool nonmaxSuppression_ = true;

    bool estimateRotation5Pt_ = true;

    // ------------------------------
    // Settings for circular matching
    // ------------------------------
    bool cudaCircularMatching_ = true;
    cv::Size winSize_ = cv::Size(21,21);
    cv::TermCriteria termcrit_ = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
    int maxLevel_ = 3;
    double minEigThreshold_ = 1e-4;
    int opticalFlowFlags_ = 0;
    int opticalFlowIters_ = 30;
};



#endif
