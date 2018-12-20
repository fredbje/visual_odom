#include "visualOdometry.h"
#include "easylogging++.h"
#include "matrixutils.h"
#include "utils.h"

void VisualOdometryStereo::checkValidMatch(std::vector<cv::Point2f>& points, std::vector<cv::Point2f>& points_return, std::vector<bool>& status)
{
    bool isValid;
    for ( unsigned int i = 0; i < points.size(); i++ )
    {
        isValid =  ( std::abs( points[i].x - points_return[i].x ) < 1 )
                && ( std::abs( points[i].y - points_return[i].y ) < 1 );
        status.push_back(isValid);
    }
}

void VisualOdometryStereo::removeInvalidPoints(std::vector<cv::Point2f>& points, const std::vector<bool>& status)
{
    int index = 0;
    for( const bool& s : status )
    {
        if( !s )
        {
            points.erase(points.begin() + index);
        }
        else
        {
            index++;
        }
    }
}

void VisualOdometryStereo::process(int current_frame_id, std::string filepath,
                    cv::Mat& rotation, cv::Mat& translation_mono, cv::Mat& translation_stereo,
                    cv::Mat& image_left_t1,
                    cv::Mat& image_right_t1,
                    cv::Mat& image_left_t0,
                    cv::Mat& image_right_t0,
                    std::vector<cv::Point2f>& points_left_t0,
                    std::vector<cv::Point2f>& points_right_t0,
                    std::vector<cv::Point2f>& points_left_t1,
                    std::vector<cv::Point2f>& points_right_t1,
                    std::vector<cv::Point2f>& points_left_t0_return,
                    FeatureSet& current_features)
{
    // ----------------------------
    // Feature detection using FAST
    // ----------------------------

    if (current_features.size() < 2000)
    {
        // use all new features
        // featureDetectionFast(image_left_t0, current_features.points);     
        // current_features.ages = std::vector<int>(current_features.points.size(), 0);

        // append new features with old features
        appendNewFeatures(image_left_t0, current_features);   

        LOG(DEBUG) << "Current feature set size: " << current_features.points.size();
    }

    // --------------------------------------------------------
    // Feature tracking using KLT tracker, bucketing and circular matching
    // --------------------------------------------------------
    int bucket_size = 50;
    int features_per_bucket = 4;
    bucketingFeatures(image_left_t0.rows, image_left_t0.cols, current_features, bucket_size, features_per_bucket);

    points_left_t0 = current_features.points;
    
    circularMatching(image_left_t0, image_right_t0, image_left_t1, image_right_t1,
                     points_left_t0, points_right_t0, points_left_t1, points_right_t1, points_left_t0_return, current_features);

    std::vector<bool> status;
    checkValidMatch(points_left_t0, points_left_t0_return, status);

    removeInvalidPoints(points_left_t0, status);
    removeInvalidPoints(points_left_t0_return, status);
    removeInvalidPoints(points_left_t1, status);
    removeInvalidPoints(points_right_t0, status);

    current_features.points = points_left_t1;

    // -----------------------------------------------------------
    // Rotation(R) estimation using Nister's Five Points Algorithm
    // -----------------------------------------------------------

    //recovering the pose and the essential cv::matrix
    cv::Mat E, mask;
    E = cv::findEssentialMat(points_left_t1, points_left_t0, stereoCamera_.f(), stereoCamera_.principalPoint(), cv::RANSAC, 0.999, 1.0, mask);
    cv::recoverPose(E, points_left_t1, points_left_t0, rotation, translation_mono, stereoCamera_.f(), stereoCamera_.principalPoint(), mask);

    // ---------------------
    // Triangulate 3D Points
    // ---------------------
    cv::Mat points4D_t0;
    triangulatePoints( stereoCamera_.projMatL(), stereoCamera_.projMatR(), points_left_t0, points_right_t0, points4D_t0 );

    // ------------------------------------------------
    // Translation (t) estimation by use solvePnPRansac
    // ------------------------------------------------
    cv::Mat points3D_t0;
    convertPointsFromHomogeneous(points4D_t0.t(), points3D_t0);
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);  
    cv::Mat inliers;  
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);

    int iterationsCount = 500;        // number of Ransac iterations.
    float reprojectionError = 2.0;    // maximum allowed distance to consider it an inlier.
    float confidence = 0.95;          // RANSAC successful confidence.
    bool useExtrinsicGuess = true;
    int flags =cv::SOLVEPNP_ITERATIVE;

    cv::solvePnPRansac( points3D_t0, points_left_t1, stereoCamera_.K(), distCoeffs, rvec, translation_stereo,
                        useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
                        inliers, flags );

    LOG(DEBUG) << "Inliers size: " << inliers.size();
}


