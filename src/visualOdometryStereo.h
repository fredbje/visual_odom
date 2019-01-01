#ifndef VISUAL_ODOM_H
#define VISUAL_ODOM_H

#include <string>
#include <opencv2/calib3d.hpp>
#include "featureset.h"
#include "stereocamera.h"
#include <gtsam/geometry/Pose3.h>

class VisualOdometryStereo
{
public:
    explicit VisualOdometryStereo(StereoCamera& stereoCamera) : stereoCamera_(stereoCamera)
    {
        rotation_ = cv::Matx<double , 3, 3>::eye();
        translation_ = cv::Matx<double, 3, 1>::zeros();
    }
    ~VisualOdometryStereo() = default;

    bool process(gtsam::Pose3& deltaT,
            const cv::Mat& imageLeftCurr,
            const cv::Mat& imageRightCurr,
            const cv::Mat& imageLeftPrev,
            const cv::Mat& imageRightPrev,
            std::vector<cv::Point2f>& pointsLeftPrev,
            std::vector<cv::Point2f>& pointsRightPrev,
            std::vector<cv::Point2f>& pointsLeftCurr,
            std::vector<cv::Point2f>& pointsRightCurr);

    void featureDetectionFast(const cv::Mat& image, std::vector<cv::Point2f>& points);

    void deleteUnmatchFeaturesCircle(std::vector<cv::Point2f>& points0, std::vector<cv::Point2f>& points1,
                                     std::vector<cv::Point2f>& points2, std::vector<cv::Point2f>& points3,
                                     std::vector<cv::Point2f>& points0_return,
                                     std::vector<uchar>& status0, std::vector<uchar>& status1,
                                     std::vector<uchar>& status2, std::vector<uchar>& status3,
                                     std::vector<int>& ages);

    void circularMatching(const cv::Mat& imageLeftCurr, const cv::Mat& imageRightCurr,
            const cv::Mat& imageLeftPrev, const cv::Mat& imageRightPrev,
            std::vector<cv::Point2f>& pointsLeftPrev, std::vector<cv::Point2f>& pointsRightPrev,
            std::vector<cv::Point2f>& pointsLeftCurr, std::vector<cv::Point2f>& pointsRightCurr,
            std::vector<cv::Point2f>& pointsLeftPrevReturn,
            std::vector<int>& ages);

    void bucketingFeatures(int image_height, int image_width, FeatureSet& currentFeatures, int bucketSize, unsigned int featuresPerBucket);

    void appendNewFeatures(const cv::Mat& image, FeatureSet& currentFeatures);

private:
    void removeInvalidPoints(std::vector<cv::Point2f>& points, const std::vector<bool>& status);
    void checkValidMatch(std::vector<cv::Point2f>& points, std::vector<cv::Point2f>& pointsReturn, std::vector<bool>& status);

private:
    StereoCamera stereoCamera_;
    cv::Matx<double, 3, 3> rotation_;
    cv::Matx<double, 3, 1> translation_;
    FeatureSet currentFeatures_;

    // ---------------------------
    // Settings for solvePnpRansac
    // ---------------------------
    int iterationsCount_ = 500;        // number of Ransac iterations.
    float reprojectionError_ = 2.0;    // maximum allowed distance to consider it an inlier.
    float confidence_ = 0.95;          // RANSAC successful confidence.
    bool useExtrinsicGuess_ = true;
    int flags_ = cv::SOLVEPNP_ITERATIVE;
    cv::Matx<double, 3, 1> rvec_ = cv::Matx<double, 3, 1>::zeros();
    cv::Mat inliersIgnored_;

    // -------------------------------
    // Settings for bucketing features
    // -------------------------------
    int bucketSize_ = 50;
    unsigned int featuresPerBucket_ = 4;

    // --------------------------
    // Settings for FAST detector
    // --------------------------
    int fastThreshold_ = 20;
    bool nonmaxSuppression_ = true;

    // ------------------------------------------
    // Settings for 5pt algorithm or PNP
    // ------------------------------------------
    bool estimateRotation5Pt_ = true;
    cv::Matx<double, 3, 1> translationMonoIgnored_ = cv::Matx<double, 3, 1>::zeros();

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
