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

void VisualOdometryStereo::process(cv::Mat& rotation, cv::Mat& translation_stereo,
        cv::Mat& imageLeftCurr,
        cv::Mat& imageRightCurr,
        cv::Mat& imageLeftPrev,
        cv::Mat& imageRightPrev,
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
        appendNewFeatures(imageLeftPrev, current_features);

        LOG(DEBUG) << "Current feature set size: " << current_features.points.size();
    }

    // -------------------------------------------------------------------
    // Feature tracking using KLT tracker, bucketing and circular matching
    // -------------------------------------------------------------------

    bucketingFeatures(stereoCamera_.height(), stereoCamera_.width(), current_features, bucketSize_, featuresPerBucket_);

    points_left_t0 = current_features.points;
    circularMatching(imageLeftPrev, imageRightPrev, imageLeftCurr, imageRightCurr,
                     points_left_t0, points_right_t0, points_left_t1, points_right_t1, points_left_t0_return, current_features);

    std::vector<bool> status;
    checkValidMatch(points_left_t0, points_left_t0_return, status);

    removeInvalidPoints(points_left_t0, status);
    removeInvalidPoints(points_left_t0_return, status);
    removeInvalidPoints(points_left_t1, status);
    removeInvalidPoints(points_right_t0, status);

    current_features.points = points_left_t1;

    // ---------------------
    // Triangulate 3D Points
    // ---------------------
    cv::Mat points4D_t0, points3D_t0;
    cv::triangulatePoints( stereoCamera_.projMatL(), stereoCamera_.projMatR(), points_left_t0, points_right_t0, points4D_t0 );
    cv::convertPointsFromHomogeneous(points4D_t0.t(), points3D_t0);

    // ---------------------------------------------
    // Rotation and translation estimation using PNP
    //----------------------------------------------
    cv::solvePnPRansac( points3D_t0, points_left_t1, stereoCamera_.K(), stereoCamera_.distCoeffs(), rvec_, translation_stereo,
                        useExtrinsicGuess_, iterationsCount_, reprojectionError_, confidence_,
                        inliersIgnored_, flags_ );

    if(stereoCamera_.fx() == stereoCamera_.fy())
    {
        // ------------------------------------------------------------------------------------
        // Rotation (R) estimation using Nister's Five Points Algorithm (yields better results)
        // ------------------------------------------------------------------------------------
        cv::Mat E, mask;
        E = cv::findEssentialMat(points_left_t1, points_left_t0, stereoCamera_.fx(), stereoCamera_.principalPoint(), cv::RANSAC, 0.999, 1.0, mask);
        cv::recoverPose(E, points_left_t1, points_left_t0, rotation, translationMonoIgnored_, stereoCamera_.fx(), stereoCamera_.principalPoint(), mask);
    }
    else
    {
        cv::Rodrigues(rvec_, rotation);
        rotation = rotation.t();
    }

    //void cuda::solvePnPRansac(const Mat& object, const Mat& image, const Mat& camera_mat, const Mat& dist_coef, Mat& rvec, Mat& tvec, bool use_extrinsic_guess=false, int num_iters=100, float max_dist=8.0, int min_inlier_count=100, vector<int>* inliers=NULL)
}


