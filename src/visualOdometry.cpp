#include "visualOdometry.h"
#include "easylogging++.h"
#include "matrixutils.h"
#include "utils.h"
#include "opencv2/video/tracking.hpp"
#include "bucket.h"


void VisualOdometryStereo::featureDetectionFast(const cv::Mat& image, std::vector<cv::Point2f>& points)
{
    std::vector<cv::KeyPoint> keypoints;
    cv::FAST(image, keypoints, fastThreshold_, nonmaxSuppression_);
    cv::KeyPoint::convert(keypoints, points, std::vector<int>());
}

void VisualOdometryStereo::deleteUnmatchFeaturesCircle(std::vector<cv::Point2f>& points0, std::vector<cv::Point2f>& points1,
                                 std::vector<cv::Point2f>& points2, std::vector<cv::Point2f>& points3,
                                 std::vector<cv::Point2f>& points0_return,
                                 std::vector<uchar>& status0, std::vector<uchar>& status1,
                                 std::vector<uchar>& status2, std::vector<uchar>& status3,
                                 std::vector<int>& ages)
{
    //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
    for (int &age : ages)
    {
        age += 1;
    }

    int indexCorrection = 0;
    for( unsigned int i = 0; i < status3.size(); i++)
    {
        assert( (i - indexCorrection) >= 0 );
        cv::Point2f pt0 = points0.at(i- indexCorrection);
        cv::Point2f pt1 = points1.at(i- indexCorrection);
        cv::Point2f pt2 = points2.at(i- indexCorrection);
        cv::Point2f pt3 = points3.at(i- indexCorrection);
        //cv::Point2f pt0_r = points0_return.at(i- indexCorrection);

        if((status3.at(i) == 0) || (pt3.x < 0) || (pt3.y < 0)
           || (status2.at(i) == 0) || (pt2.x < 0) || (pt2.y < 0)
           || (status1.at(i) == 0) || (pt1.x < 0) || (pt1.y < 0)
           || (status0.at(i) == 0) || (pt0.x < 0) || (pt0.y < 0))
        {
            if((pt0.x < 0) || (pt0.y < 0)
               || (pt1.x < 0) || (pt1.y < 0)
               || (pt2.x < 0) || (pt2.y < 0)
               || (pt3.x < 0) || (pt3.y < 0))
            {
                status3.at(i) = 0;
            }
            points0.erase (points0.begin() + (i - indexCorrection));
            points1.erase (points1.begin() + (i - indexCorrection));
            points2.erase (points2.begin() + (i - indexCorrection));
            points3.erase (points3.begin() + (i - indexCorrection));
            points0_return.erase (points0_return.begin() + (i - indexCorrection));

            ages.erase (ages.begin() + (i - indexCorrection));
            indexCorrection++;
        }
    }
}

void VisualOdometryStereo::circularMatching(cv::Mat img_l_0, cv::Mat img_r_0, cv::Mat img_l_1, cv::Mat img_r_1,
                      std::vector<cv::Point2f>& points_l_0, std::vector<cv::Point2f>& points_r_0,
                      std::vector<cv::Point2f>& points_l_1, std::vector<cv::Point2f>& points_r_1,
                      std::vector<cv::Point2f>& points_l_0_return,
                      FeatureSet& current_features)
{
    //this function automatically gets rid of points for which tracking fails
    std::vector<float> err;
    cv::Size winSize=cv::Size(21,21);
    cv::TermCriteria termcrit=cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

    std::vector<uchar> status0, status1, status2, status3;

    // void calcOpticalFlowPyrLK(InArr prevImg, InArr nextImg, InArr prevPts, InOutArr nextPts, OutArr status, OutArr err, Size winSize=Size(21,21), int maxLevel=3, TermCriteria criteria=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01), int flags=0, double minEigThreshold=1e-4 )
    cv::calcOpticalFlowPyrLK(img_l_0, img_r_0, points_l_0, points_r_0, status0, err, winSize, 3, termcrit, 0, 0.001);
    cv::calcOpticalFlowPyrLK(img_r_0, img_r_1, points_r_0, points_r_1, status1, err, winSize, 3, termcrit, 0, 0.001);
    cv::calcOpticalFlowPyrLK(img_r_1, img_l_1, points_r_1, points_l_1, status2, err, winSize, 3, termcrit, 0, 0.001);
    cv::calcOpticalFlowPyrLK(img_l_1, img_l_0, points_l_1, points_l_0_return, status3, err, winSize, 3, termcrit, 0, 0.001);

    deleteUnmatchFeaturesCircle(points_l_0, points_r_0, points_r_1, points_l_1, points_l_0_return,
                                status0, status1, status2, status3, current_features.ages);
}


void VisualOdometryStereo::bucketingFeatures(int image_height, int image_width, FeatureSet& current_features, int bucket_size, int features_per_bucket){
    // This function buckets features
    // image: only use for getting dimension of the image
    // bucket_size: bucket size in pixel is bucket_size*bucket_size
    // features_per_bucket: number of selected features per bucket
    int buckets_nums_height = image_height/bucket_size;
    int buckets_nums_width = image_width/bucket_size;
    //int buckets_number = buckets_nums_height * buckets_nums_width;

    std::vector<Bucket> Buckets;

    // initialize all the buckets
    for (int buckets_idx_height = 0; buckets_idx_height <= buckets_nums_height; buckets_idx_height++)
    {
        for (int buckets_idx_width = 0; buckets_idx_width <= buckets_nums_width; buckets_idx_width++)
        {
            Buckets.emplace_back(Bucket(features_per_bucket));
        }
    }

    // bucket all current features into buckets by their location
    int buckets_nums_height_idx, buckets_nums_width_idx, buckets_idx;
    for( unsigned int i = 0; i < current_features.points.size(); ++i )
    {
        buckets_nums_height_idx = static_cast<int>(current_features.points[i].y/bucket_size);
        buckets_nums_width_idx = static_cast<int>(current_features.points[i].x/bucket_size);
        buckets_idx = buckets_nums_height_idx*buckets_nums_width + buckets_nums_width_idx;
        Buckets[buckets_idx].add_feature(current_features.points[i], current_features.ages[i]);
    }

    // get features back from buckets
    current_features.clear();
    for (int buckets_idx_height = 0; buckets_idx_height <= buckets_nums_height; buckets_idx_height++)
    {
        for (int buckets_idx_width = 0; buckets_idx_width <= buckets_nums_width; buckets_idx_width++)
        {
            buckets_idx = buckets_idx_height*buckets_nums_width + buckets_idx_width;
            Buckets[buckets_idx].get_features(current_features);
        }
    }
    LOG(DEBUG) << "current features number after bucketing: " << current_features.size();
}

void VisualOdometryStereo::appendNewFeatures(cv::Mat& image, FeatureSet& current_features){
    std::vector<cv::Point2f>  points_new;
    featureDetectionFast(image, points_new);
    current_features.points.insert(current_features.points.end(), points_new.begin(), points_new.end());
    std::vector<int>  ages_new(points_new.size(), 0);
    current_features.ages.insert(current_features.ages.end(), ages_new.begin(), ages_new.end());
}


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
        // Rotation (R) estimation using Nister's Five Points Algorithm (yields better results) but little slower
        // ------------------------------------------------------------------------------------
        cv::Mat E, mask;
        E = cv::findEssentialMat(points_left_t1, points_left_t0, stereoCamera_.K(), cv::RANSAC, 0.999, 1.0, mask);
        cv::recoverPose(E, points_left_t1, points_left_t0, rotation, translationMonoIgnored_, stereoCamera_.fx(), stereoCamera_.principalPoint(), mask);
    }
    else
    {
        cv::Rodrigues(rvec_, rotation);
        rotation = rotation.t();
    }

    //void cuda::solvePnPRansac(const Mat& object, const Mat& image, const Mat& camera_mat, const Mat& dist_coef, Mat& rvec, Mat& tvec, bool use_extrinsic_guess=false, int num_iters=100, float max_dist=8.0, int min_inlier_count=100, vector<int>* inliers=NULL)
}


