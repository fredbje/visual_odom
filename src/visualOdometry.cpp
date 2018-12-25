#include "visualOdometry.h"
#include "easylogging++.h"
#include "matrixutils.h"
#include "utils.h"
#include "opencv2/video/tracking.hpp"
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaoptflow.hpp>
#include "bucket.h"

void download(const cv::cuda::GpuMat& d_mat, std::vector<cv::Point_<PointType>>& vec)
{
    vec.resize(d_mat.cols);
    cv::Mat mat(1, d_mat.cols, CV_32FC2, (void*)&vec[0]);
    d_mat.download(mat);
}

void download(const cv::cuda::GpuMat& d_mat, std::vector<uchar>& vec)
{
    vec.resize(d_mat.cols);
    cv::Mat mat(1, d_mat.cols, CV_8UC1, (void*)&vec[0]);
    d_mat.download(mat);
}

void VisualOdometryStereo::featureDetectionFast(const cv::Mat& image, std::vector<cv::Point_<PointType>>& points)
{
    std::vector<cv::KeyPoint> keypoints;
    cv::FAST(image, keypoints, fastThreshold_, nonmaxSuppression_);
    cv::KeyPoint::convert(keypoints, points, std::vector<int>());
}

void VisualOdometryStereo::deleteUnmatchFeaturesCircle(std::vector<cv::Point_<PointType>>& points0, std::vector<cv::Point_<PointType>>& points1,
                                 std::vector<cv::Point_<PointType>>& points2, std::vector<cv::Point_<PointType>>& points3,
                                 std::vector<cv::Point_<PointType>>& points0_return,
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
        cv::Point_<PointType> pt0 = points0.at(i- indexCorrection);
        cv::Point_<PointType> pt1 = points1.at(i- indexCorrection);
        cv::Point_<PointType> pt2 = points2.at(i- indexCorrection);
        cv::Point_<PointType> pt3 = points3.at(i- indexCorrection);
        //cv::Point_<PointType> pt0_r = points0_return.at(i- indexCorrection);

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

void VisualOdometryStereo::circularMatching(const cv::Mat& imageLeftPrev, const cv::Mat& imageRightPrev,
        const cv::Mat& imageLeftCurr, const cv::Mat& imageRightCurr,
        std::vector<cv::Point_<PointType>>& pointsLeftPrev, std::vector<cv::Point_<PointType>>& pointsRightPrev,
        std::vector<cv::Point_<PointType>>& pointsLeftCurr, std::vector<cv::Point_<PointType>>& pointsRightCurr,
        std::vector<cv::Point_<PointType>>& pointsLeftPrevReturn,
        FeatureSet<PointType>& currentFeatures)
{
    //this function automatically gets rid of points for which tracking fails

    std::vector<uchar> status0, status1, status2, status3;

   if(cudaCircularMatching_)
   {
       cv::cuda::GpuMat d_imageLeftPrev(imageLeftPrev);
       cv::cuda::GpuMat d_imageRightPrev(imageRightPrev);
       cv::cuda::GpuMat d_imageLeftCurr(imageLeftCurr);
       cv::cuda::GpuMat d_imageRightCurr(imageRightCurr);

       cv::cuda::GpuMat d_pointsLeftPrev(pointsLeftPrev);
       cv::cuda::GpuMat d_pointsRightPrev(pointsRightPrev);
       cv::cuda::GpuMat d_pointsLeftCurr(pointsLeftCurr);
       cv::cuda::GpuMat d_pointsRightCurr(pointsRightCurr);
       cv::cuda::GpuMat d_pointsLeftPrevReturn(pointsLeftPrevReturn);

       cv::cuda::GpuMat d_status0, d_status1, d_status2, d_status3;

       cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse = cv::cuda::SparsePyrLKOpticalFlow::create(winSize_, maxLevel_, opticalFlowIters_);
       d_pyrLK_sparse->calc(d_imageLeftPrev,  d_imageRightPrev, d_pointsLeftPrev,  d_pointsRightPrev,      d_status0);
       d_pyrLK_sparse->calc(d_imageRightPrev, d_imageRightCurr, d_pointsRightPrev, d_pointsRightCurr,      d_status1);
       d_pyrLK_sparse->calc(d_imageRightCurr, d_imageLeftCurr,  d_pointsRightCurr, d_pointsLeftCurr,       d_status2);
       d_pyrLK_sparse->calc(d_imageLeftCurr,  d_imageLeftPrev,  d_pointsLeftCurr,  d_pointsLeftPrevReturn, d_status3);

       download(d_pointsRightPrev,      pointsRightPrev);
       download(d_pointsRightCurr,      pointsRightCurr);
       download(d_pointsLeftCurr,       pointsLeftCurr);
       download(d_pointsLeftPrevReturn, pointsLeftPrevReturn);

       download(d_status0, status0);
       download(d_status1, status1);
       download(d_status2, status2);
       download(d_status3, status3);
   }
   else
   {
       std::vector<float> err;

       cv::calcOpticalFlowPyrLK(imageLeftPrev,  imageRightPrev, pointsLeftPrev,  pointsRightPrev,      status0, err, winSize_, maxLevel_, termcrit_, opticalFlowFlags_, minEigThreshold_);
       cv::calcOpticalFlowPyrLK(imageRightPrev, imageRightCurr, pointsRightPrev, pointsRightCurr,      status1, err, winSize_, maxLevel_, termcrit_, opticalFlowFlags_, minEigThreshold_);
       cv::calcOpticalFlowPyrLK(imageRightCurr, imageLeftCurr,  pointsRightCurr, pointsLeftCurr,       status2, err, winSize_, maxLevel_, termcrit_, opticalFlowFlags_, minEigThreshold_);
       cv::calcOpticalFlowPyrLK(imageLeftCurr,  imageLeftPrev,  pointsLeftCurr,  pointsLeftPrevReturn, status3, err, winSize_, maxLevel_, termcrit_, opticalFlowFlags_, minEigThreshold_);
   }

    deleteUnmatchFeaturesCircle(pointsLeftPrev, pointsRightPrev, pointsRightCurr, pointsLeftCurr, pointsLeftPrevReturn,
                                status0, status1, status2, status3, currentFeatures.ages);
}


void VisualOdometryStereo::bucketingFeatures(int image_height, int image_width, FeatureSet<PointType>& currentFeatures, int bucketSize, unsigned int featuresPerBucket){
    // This function buckets features
    // image: only use for getting dimension of the image
    // bucket_size: bucket size in pixel is bucket_size*bucket_size
    // features_per_bucket: number of selected features per bucket
    int buckets_nums_height = image_height/bucketSize;
    int buckets_nums_width = image_width/bucketSize;
    //int buckets_number = buckets_nums_height * buckets_nums_width;

    std::vector<Bucket<PointType>> Buckets;

    // initialize all the buckets
    for (int buckets_idx_height = 0; buckets_idx_height <= buckets_nums_height; buckets_idx_height++)
    {
        for (int buckets_idx_width = 0; buckets_idx_width <= buckets_nums_width; buckets_idx_width++)
        {
            Buckets.emplace_back(Bucket<PointType>(featuresPerBucket));
        }
    }

    // bucket all current features into buckets by their location
    int buckets_nums_height_idx, buckets_nums_width_idx, buckets_idx;
    for( unsigned int i = 0; i < currentFeatures.points.size(); ++i )
    {
        buckets_nums_height_idx = static_cast<int>(currentFeatures.points[i].y/bucketSize);
        buckets_nums_width_idx = static_cast<int>(currentFeatures.points[i].x/bucketSize);
        buckets_idx = buckets_nums_height_idx*buckets_nums_width + buckets_nums_width_idx;
        Buckets[buckets_idx].add_feature(currentFeatures.points[i], currentFeatures.ages[i]);
    }

    // get features back from buckets
    currentFeatures.clear();
    for (int buckets_idx_height = 0; buckets_idx_height <= buckets_nums_height; buckets_idx_height++)
    {
        for (int buckets_idx_width = 0; buckets_idx_width <= buckets_nums_width; buckets_idx_width++)
        {
            buckets_idx = buckets_idx_height*buckets_nums_width + buckets_idx_width;
            Buckets[buckets_idx].get_features(currentFeatures);
        }
    }
    LOG(DEBUG) << "current features number after bucketing: " << currentFeatures.size();
}

void VisualOdometryStereo::appendNewFeatures(cv::Mat& image, FeatureSet<PointType>& currentFeatures){
    std::vector<cv::Point_<PointType>>  points_new;
    featureDetectionFast(image, points_new);
    currentFeatures.points.insert(currentFeatures.points.end(), points_new.begin(), points_new.end());
    std::vector<int>  ages_new(points_new.size(), 0);
    currentFeatures.ages.insert(currentFeatures.ages.end(), ages_new.begin(), ages_new.end());
}


void VisualOdometryStereo::checkValidMatch(std::vector<cv::Point_<PointType>>& points, std::vector<cv::Point_<PointType>>& points_return, std::vector<bool>& status)
{
    bool isValid;
    for ( unsigned int i = 0; i < points.size(); i++ )
    {
        isValid =  ( std::abs( points[i].x - points_return[i].x ) < 1 )
                && ( std::abs( points[i].y - points_return[i].y ) < 1 );
        status.push_back(isValid);
    }
}

void VisualOdometryStereo::removeInvalidPoints(std::vector<cv::Point_<PointType>>& points, const std::vector<bool>& status)
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

bool VisualOdometryStereo::process(cv::Matx<PoseType, 3, 3>& rotation, cv::Matx<PoseType, 3, 1>& translation_stereo,
        cv::Mat& imageLeftCurr,
        cv::Mat& imageRightCurr,
        cv::Mat& imageLeftPrev,
        cv::Mat& imageRightPrev,
        std::vector<cv::Point_<PointType>>& pointsLeftPrev,
        std::vector<cv::Point_<PointType>>& pointsRightPrev,
        std::vector<cv::Point_<PointType>>& pointsLeftCurr,
        std::vector<cv::Point_<PointType>>& pointsRightCurr,
        std::vector<cv::Point_<PointType>>& pointsLeftPrevReturn,
        FeatureSet<PointType>& currentFeatures)
{
    // ----------------------------
    // Feature detection using FAST
    // ----------------------------
    if (currentFeatures.size() < 2000)
    {
        // use all new features
        // featureDetectionFast(image_left_t0, current_features.points);     
        // current_features.ages = std::vector<int>(current_features.points.size(), 0);

        // append new features with old features
        appendNewFeatures(imageLeftPrev, currentFeatures);

        LOG(DEBUG) << "Current feature set size: " << currentFeatures.points.size();
    }

    // -------------------------------------------------------------------
    // Feature tracking using KLT tracker, bucketing and circular matching
    // -------------------------------------------------------------------

    bucketingFeatures(stereoCamera_.height(), stereoCamera_.width(), currentFeatures, bucketSize_, featuresPerBucket_);

    pointsLeftPrev = currentFeatures.points;
    circularMatching(imageLeftPrev, imageRightPrev, imageLeftCurr, imageRightCurr,
                     pointsLeftPrev, pointsRightPrev, pointsLeftCurr, pointsRightCurr, pointsLeftPrevReturn, currentFeatures);

    std::vector<bool> status;
    checkValidMatch(pointsLeftPrev, pointsLeftPrevReturn, status);

    removeInvalidPoints(pointsLeftPrev, status);
    removeInvalidPoints(pointsLeftPrevReturn, status);
    removeInvalidPoints(pointsLeftCurr, status);
    removeInvalidPoints(pointsRightPrev, status);

    currentFeatures.points = pointsLeftCurr;

    // ---------------------
    // Triangulate 3D Points
    // ---------------------
    cv::Mat points4D_t0, points3D_t0;
    cv::triangulatePoints( stereoCamera_.projMatL(), stereoCamera_.projMatR(), pointsLeftPrev, pointsRightPrev, points4D_t0 );
    cv::convertPointsFromHomogeneous(points4D_t0.t(), points3D_t0);

    // ---------------------------------------------
    // Rotation and translation estimation using PNP
    //----------------------------------------------

    cv::solvePnPRansac( points3D_t0, pointsLeftCurr, stereoCamera_.K(), stereoCamera_.distCoeffs(), rvec_, translation_stereo,
                        useExtrinsicGuess_, iterationsCount_, reprojectionError_, confidence_, inliersIgnored_, flags_ );

    if(estimateRotation5Pt_)
    {
        // ------------------------------------------------------------------------------------
        // Rotation (R) estimation using Nister's Five Points Algorithm (yields better results, but little slower)
        // ------------------------------------------------------------------------------------
        cv::Mat E, mask;
        E = cv::findEssentialMat(pointsLeftCurr, pointsLeftPrev, stereoCamera_.K(), cv::RANSAC, 0.999, 1.0, mask);
        cv::recoverPose(E, pointsLeftCurr, pointsLeftPrev, rotation, translationMonoIgnored_, stereoCamera_.fx(), stereoCamera_.principalPoint(), mask);
    }
    else
    {
        cv::Rodrigues(rvec_, rotation);
        rotation = rotation.t();
    }

    return true;
}


