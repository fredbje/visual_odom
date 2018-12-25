#ifndef VISUAL_ODOM_H
#define VISUAL_ODOM_H

#include <string>
#include <opencv2/calib3d.hpp>
#include "featureset.h"
#include "stereocamera.h"

typedef double PoseType;
typedef float PointType;
typedef float TimeType;
typedef float CamType;

class VisualOdometryStereo
{
public:
    explicit VisualOdometryStereo(cv::FileStorage& fSettings) : stereoCamera_(fSettings) {}
    ~VisualOdometryStereo() = default;

    bool process(cv::Matx<PoseType, 3, 3>& rotation, cv::Matx<PoseType, 3, 1>& translation_stereo,
            cv::Mat& imageLeftCurr,
            cv::Mat& imageRightCurr,
            cv::Mat& imageLeftPrev,
            cv::Mat& imageRightPrev,
            std::vector<cv::Point_<PointType>>& pointsLeftPrev,
            std::vector<cv::Point_<PointType>>& pointsRightPrev,
            std::vector<cv::Point_<PointType>>& pointsLeftCurr,
            std::vector<cv::Point_<PointType>>& pointsRightCurr,
            std::vector<cv::Point_<PointType>>& pointsLeftPrevReturn,
            FeatureSet<PointType>& currentFeatures);

    void featureDetectionFast(const cv::Mat& image, std::vector<cv::Point_<PointType>>& points);

    void deleteUnmatchFeaturesCircle(std::vector<cv::Point_<PointType>>& points0, std::vector<cv::Point_<PointType>>& points1,
                                     std::vector<cv::Point_<PointType>>& points2, std::vector<cv::Point_<PointType>>& points3,
                                     std::vector<cv::Point_<PointType>>& points0_return,
                                     std::vector<uchar>& status0, std::vector<uchar>& status1,
                                     std::vector<uchar>& status2, std::vector<uchar>& status3,
                                     std::vector<int>& ages);

    void circularMatching(const cv::Mat& imageLeftPrev, const cv::Mat& imageRightPrev,
            const cv::Mat& imageLeftCurr, const cv::Mat& imageRightCurr,
            std::vector<cv::Point_<PointType>>& pointsLeftPrev, std::vector<cv::Point_<PointType>>& pointsRightPrev,
            std::vector<cv::Point_<PointType>>& pointsLeftCurr, std::vector<cv::Point_<PointType>>& pointsRightCurr,
            std::vector<cv::Point_<PointType>>& pointsLeftPrevReturn,
            FeatureSet<PointType>& current_features);

    void bucketingFeatures(int image_height, int image_width, FeatureSet<PointType>& currentFeatures, int bucketSize, unsigned int featuresPerBucket);

    void appendNewFeatures(cv::Mat& image, FeatureSet<PointType>& currentFeatures);

private:
    void removeInvalidPoints(std::vector<cv::Point_<PointType>>& points, const std::vector<bool>& status);
    void checkValidMatch(std::vector<cv::Point_<PointType>>& points, std::vector<cv::Point_<PointType>>& pointsReturn, std::vector<bool>& status);

private:
    StereoCamera<CamType> stereoCamera_;
    cv::Matx<PoseType, 3, 1> translationMonoIgnored_ = cv::Matx<PoseType, 3, 1>::zeros();

    // ---------------------------
    // Settings for solvePnpRansac
    // ---------------------------
    int cudaPnpRansac_ = true;
    int iterationsCount_ = 500;        // number of Ransac iterations.
    float reprojectionError_ = 2.0;    // maximum allowed distance to consider it an inlier.
    float confidence_ = 0.95;          // RANSAC successful confidence.
    bool useExtrinsicGuess_ = true;
    int flags_ = cv::SOLVEPNP_P3P; // cv::SOLVEPNP_ITERATIVE;
    cv::Matx<PoseType, 3, 1> rvec_ = cv::Matx<PoseType, 3, 1>::zeros();
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
