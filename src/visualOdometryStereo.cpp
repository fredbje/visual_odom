#include "visualOdometryStereo.h"
#include "easylogging++.h"
#include "matrixutils.h"
#include "utils.h"
#include "opencv2/video/tracking.hpp"
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaoptflow.hpp>
#include <thread>
#include "bucket.h"

// ---------------
// GTSAM includes
// ---------------
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/inference/Symbol.h>


float computeFeatureFlow(const std::vector<cv::Point2f>& pointsPrev, const std::vector<cv::Point2f>& pointsCurr)
{
    float total_flow = 0.0;

    size_t pointSetSize = pointsPrev.size();

    if(pointSetSize != pointsCurr.size())
    {
        LOG(ERROR) << "Point sets must be of equal size";
        return total_flow;
    }

    for (size_t i = 0; i < pointSetSize; ++i)
    {
        float x_diff = pointsCurr[i].x - pointsPrev[i].x;
        float y_diff = pointsCurr[i].y - pointsPrev[i].y;
        total_flow += sqrt(x_diff * x_diff + y_diff * y_diff);
    }


    return total_flow / pointSetSize;
}


void download(const cv::cuda::GpuMat& d_mat, std::vector<cv::Point2f>& vec)
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

    unsigned int indexCorrection = 0;
    for( unsigned int i = 0; i < status3.size(); i++)
    {
        assert( (i - indexCorrection) >= 0 );
        cv::Point2f pt0 = points0.at(i- indexCorrection);
        cv::Point2f pt1 = points1.at(i- indexCorrection);
        cv::Point2f pt2 = points2.at(i- indexCorrection);
        cv::Point2f pt3 = points3.at(i- indexCorrection);
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

void VisualOdometryStereo::circularMatching(const cv::Mat& imageLeftCurr, const cv::Mat& imageRightCurr,
        const cv::Mat& imageLeftPrev, const cv::Mat& imageRightPrev,
        std::vector<cv::Point2f>& pointsLeftPrev, std::vector<cv::Point2f>& pointsRightPrev,
        std::vector<cv::Point2f>& pointsLeftCurr, std::vector<cv::Point2f>& pointsRightCurr,
        std::vector<cv::Point2f>& pointsLeftPrevReturn,
        std::vector<int>& ages)
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
       cv::cuda::Stream stream;
       cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse = cv::cuda::SparsePyrLKOpticalFlow::create(winSize_, maxLevel_, opticalFlowIters_);
       d_pyrLK_sparse->calc(d_imageLeftPrev,  d_imageRightPrev, d_pointsLeftPrev,  d_pointsRightPrev,      d_status0, cv::noArray(), stream);
       stream.waitForCompletion();
       d_pyrLK_sparse->calc(d_imageRightPrev, d_imageRightCurr, d_pointsRightPrev, d_pointsRightCurr,      d_status1, cv::noArray(), stream);
       stream.waitForCompletion();
       d_pyrLK_sparse->calc(d_imageRightCurr, d_imageLeftCurr,  d_pointsRightCurr, d_pointsLeftCurr,       d_status2, cv::noArray(), stream);
       stream.waitForCompletion();
       d_pyrLK_sparse->calc(d_imageLeftCurr,  d_imageLeftPrev,  d_pointsLeftCurr,  d_pointsLeftPrevReturn, d_status3, cv::noArray(), stream);
       stream.waitForCompletion();

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
                                status0, status1, status2, status3, ages);
}


void VisualOdometryStereo::bucketingFeatures(int imageHeight, int imageWidth, FeatureSet& currentFeatures, int bucketSize, unsigned int featuresPerBucket){
    // This function buckets features
    // image: only use for getting dimension of the image
    // bucket_size: bucket size in pixel is bucket_size*bucket_size
    // features_per_bucket: number of selected features per bucket
    int numBucketsHeight = imageHeight / bucketSize;
    int numBucketsWidth  = imageWidth / bucketSize;
    //int buckets_number = buckets_nums_height * buckets_nums_width;

    std::vector<Bucket> buckets;

    // initialize all the buckets
    for (int bucketIdxHeight = 0; bucketIdxHeight <= numBucketsHeight; bucketIdxHeight++)
    {
        for (int bucketIdxWidth = 0; bucketIdxWidth <= numBucketsWidth; bucketIdxWidth++)
        {
            buckets.emplace_back(Bucket(featuresPerBucket));
        }
    }

    // bucket all current features into buckets by their location
    int buckets_nums_height_idx, buckets_nums_width_idx, bucketIdx;
    for( unsigned int i = 0; i < currentFeatures.points.size(); ++i )
    {
        buckets_nums_height_idx = static_cast<int>(currentFeatures.points[i].y/bucketSize);
        buckets_nums_width_idx = static_cast<int>(currentFeatures.points[i].x/bucketSize);
        bucketIdx = buckets_nums_height_idx*numBucketsWidth + buckets_nums_width_idx;
        buckets[bucketIdx].add_feature(currentFeatures.points[i], currentFeatures.ages[i]);
    }

    // get features back from buckets
    currentFeatures.clear();
    for (int bucketIdxHeight = 0; bucketIdxHeight <= numBucketsHeight; bucketIdxHeight++)
    {
        for (int bucketIdxWidth = 0; bucketIdxWidth <= numBucketsWidth; bucketIdxWidth++)
        {
            bucketIdx = bucketIdxHeight*numBucketsWidth + bucketIdxWidth;
            buckets[bucketIdx].get_features(currentFeatures);
        }
    }
    LOG(DEBUG) << "current features number after bucketing: " << currentFeatures.size();
}

void VisualOdometryStereo::appendNewFeatures(const cv::Mat& image, FeatureSet& currentFeatures){
    std::vector<cv::Point2f>  points_new;
    featureDetectionFast(image, points_new);
    currentFeatures.points.insert(currentFeatures.points.end(), points_new.begin(), points_new.end());
    std::vector<int>  ages_new(points_new.size(), 0);
    currentFeatures.ages.insert(currentFeatures.ages.end(), ages_new.begin(), ages_new.end());
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

bool VisualOdometryStereo::process(gtsam::Pose3& deltaT, float& averageFlow,
        const cv::Mat& imageLeftCurr,
        const cv::Mat& imageRightCurr,
        const cv::Mat& imageLeftPrev,
        const cv::Mat& imageRightPrev,
        std::vector<cv::Point2f>& pointsLeftPrev,
        std::vector<cv::Point2f>& pointsRightPrev,
        std::vector<cv::Point2f>& pointsLeftCurr,
        std::vector<cv::Point2f>& pointsRightCurr)
{
    // ----------------------------
    // Feature detection using FAST
    // ----------------------------
    if (currentFeatures_.size() < 2000)
    {
        // use all new features
        // featureDetectionFast(image_left_t0, current_features.points);     
        // current_features.ages = std::vector<int>(current_features.points.size(), 0);

        // append new features with old features
        appendNewFeatures(imageLeftPrev, currentFeatures_);

        LOG(DEBUG) << "Current feature set size: " << currentFeatures_.points.size();
    }

    // -------------------------------------------------------------------
    // Feature tracking using KLT tracker, bucketing and circular matching
    // -------------------------------------------------------------------

    bucketingFeatures(stereoCamera_.height(), stereoCamera_.width(), currentFeatures_, bucketSize_, featuresPerBucket_);

    pointsLeftPrev = currentFeatures_.points;
    std::vector<cv::Point2f> pointsLeftPrevReturn;
    circularMatching(imageLeftCurr, imageRightCurr, imageLeftPrev, imageRightPrev, pointsLeftPrev, pointsRightPrev, pointsLeftCurr, pointsRightCurr, pointsLeftPrevReturn, currentFeatures_.ages);

    std::vector<bool> status;
    checkValidMatch(pointsLeftPrev, pointsLeftPrevReturn, status);

    std::thread t1(&VisualOdometryStereo::removeInvalidPoints, this, std::ref(pointsLeftPrev),       std::cref(status));
    std::thread t2(&VisualOdometryStereo::removeInvalidPoints, this, std::ref(pointsLeftPrevReturn), std::cref(status));
    std::thread t3(&VisualOdometryStereo::removeInvalidPoints, this, std::ref(pointsLeftCurr),       std::cref(status));
    removeInvalidPoints(pointsRightPrev, status);
    t1.join();
    t2.join();
    t3.join();

    currentFeatures_.points = pointsLeftCurr;

    // ---------------------
    // Triangulate 3D Points
    // ---------------------
    cv::Mat points4DPrev, points3DPrev;
    try
    {
        cv::triangulatePoints( stereoCamera_.projMatL(), stereoCamera_.projMatR(), pointsLeftPrev, pointsRightPrev, points4DPrev );
        cv::convertPointsFromHomogeneous(points4DPrev.t(), points3DPrev);
    }
    catch ( cv::Exception& e )
    {
        const char *err_msg = e.what();
        LOG(ERROR) << "Tracking failed. Exception caught: " << err_msg;
        deltaT = gtsam::Pose3();
        averageFlow = 0.f;
        return false;
    }

    // ---------------------------------------------
    // Rotation and translation estimation using PNP
    //----------------------------------------------
    // This function outputs {t}_T_{t-1}, but we want {t-1}_T_{t}
    // TODO Minimize reprojection error wrt scale of translation only.
    try
    {
        cv::solvePnPRansac( points3DPrev, pointsLeftCurr, stereoCamera_.K(), stereoCamera_.distCoeffs(), rvec_, translation_,
                            useExtrinsicGuess_, iterationsCount_, reprojectionError_, confidence_, inliersIgnored_, flags_ );
        translation_ = - translation_;
    }
    catch ( cv::Exception& e )
    {
        const char *err_msg = e.what();
        LOG(ERROR) << "Tracking failed. Exception caught: " << err_msg;
        deltaT = gtsam::Pose3();
        averageFlow = 0.f;
        return false;
    }

    if(estimateRotation5Pt_)
    {
        // -------------------------------------------------------------------------------------------------------
        // Rotation (R) estimation using Nister's Five Points Algorithm (yields better results, but little slower)
        // -------------------------------------------------------------------------------------------------------
        try
        {
            cv::Mat E, mask;
            E = cv::findEssentialMat(pointsLeftCurr, pointsLeftPrev, stereoCamera_.K(), cv::RANSAC, 0.999, 1.0, mask);
            cv::recoverPose(E, pointsLeftCurr, pointsLeftPrev, rotation_, translationMonoIgnored_, stereoCamera_.fx(), stereoCamera_.principalPoint(), mask);
        }
        catch ( cv::Exception& e )
        {
            const char *err_msg = e.what();
            LOG(WARNING) << "Rotation estimation with 5pt algorithm failed. Using rotation from solvePnp. Exeption caught: " << err_msg;
            cv::Rodrigues(rvec_, rotation_);
            rotation_ = rotation_.t();
        }

    }
    else
    {
        cv::Rodrigues(rvec_, rotation_);
        rotation_ = rotation_.t();
        averageFlow = 0.f;
    }


    cv::Vec3d rotation_euler = rotationMatrixToEulerAngles(rotation_);
    if (abs(rotation_euler[1]) < 0.1 && abs(rotation_euler[0]) < 0.1 && abs(rotation_euler[2]) < 0.1)
    {
        double scale = sqrt((translation_(0))*(translation_(0))
                            + (translation_(1))*(translation_(1))
                            + (translation_(2))*(translation_(2))) ;

        if (scale > 0.05 && scale < 10)
        {
            deltaT = gtsam::Pose3((gtsam::Matrix(4, 4) <<
                    rotation_(0, 0), rotation_(0, 1), rotation_(0, 2), translation_(0),
                    rotation_(1, 0), rotation_(1, 1), rotation_(1, 2), translation_(1),
                    rotation_(2, 0), rotation_(2, 1), rotation_(2, 2), translation_(2),
                    0.0, 0.0, 0.0, 1.0).finished());
        }
        else
        {
            deltaT = gtsam::Pose3();
            LOG(WARNING) << "Scale below 0.1, or incorrect translation";
        }
    }
    else
    {
        LOG(WARNING) << "Too large rotation";
    }

    averageFlow = computeFeatureFlow(pointsLeftPrev, pointsLeftCurr);

/*
    // --------------------------------------
    // GTSAM CODE
    // --------------------------------------
    //read stereo measurements and construct smart factors
    const gtsam::noiseModel::Isotropic::shared_ptr model = gtsam::noiseModel::Isotropic::Sigma(3,1);
    gtsam::SmartStereoProjectionPoseFactor::shared_ptr factor;
    gtsam::Values initial_estimate;
    gtsam::NonlinearFactorGraph graph;
    for(unsigned int i = 0; i < pointsLeftPrev.size(); i++)
    {
        if( (std::abs(pointsLeftPrev[i].y - pointsRightPrev[i].y) < 2) && (std::abs(pointsLeftCurr[i].y - pointsRightCurr[i].y) < 2) )
        {
            graph.push_back(factor);
            factor = gtsam::SmartStereoProjectionPoseFactor::shared_ptr(new gtsam::SmartStereoProjectionPoseFactor(model));
            factor->add(gtsam::StereoPoint2(pointsLeftPrev[i].x, pointsRightPrev[i].x, (pointsLeftPrev[i].y + pointsRightPrev[i].y) / 2.f ), 1, stereoCamera_.cal3Stereo());
            factor->add(gtsam::StereoPoint2(pointsLeftCurr[i].x, pointsRightCurr[i].x, (pointsLeftCurr[i].y + pointsRightCurr[i].y) / 2.f ), 2, stereoCamera_.cal3Stereo());
            LOG(INFO) << "Added factor";
        }
    }
    initial_estimate.insert(1, gtsam::Pose3());
    initial_estimate.insert(2, deltaT.inverse());
    //constrain the first pose such that it cannot change from its original value during optimization
    // NOTE: NonlinearEquality forces the optimizer to use QR rather than Cholesky
    // QR is much slower than Cholesky, but numerically more stable
    graph.emplace_shared<gtsam::NonlinearEquality<gtsam::Pose3> >(1, gtsam::Pose3());
    gtsam::LevenbergMarquardtParams params;
//    params.verbosityLM = gtsam::LevenbergMarquardtParams::TRYLAMBDA;
//    params.verbosity = gtsam::NonlinearOptimizerParams::ERROR;
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate, params);
    gtsam::Values result = optimizer.optimize();
    deltaT = result.at<gtsam::Pose3>(2);

 */

    return true;
}

bool VisualOdometryStereo::process(gtsam::Pose3& deltaT, float& averageFlow,
             const cv::Mat& imageLeftCurr,
             const cv::Mat& imageRightCurr,
             const cv::Mat& imageLeftPrev,
             const cv::Mat& imageRightPrev)
{
    std::vector< cv::Point2f > pointsLeftPrev, pointsRightPrev, pointsLeftCurr, pointsRightCurr;
    return process(deltaT, averageFlow, imageLeftCurr, imageRightCurr, imageLeftPrev, imageRightPrev,
            pointsLeftPrev, pointsRightPrev, pointsLeftCurr, pointsRightCurr);
}

void VisualOdometryStereo::saveSettings(const std::string& settingsFile)
{
    std::fstream f;
    f.open(settingsFile, std::ios_base::app);
    if(f.is_open())
    {
        f << "////////////////// System Settings //////////////////" << std::endl;

        f << "\t////////////////// Settings for Key Frame Decision //////////////////" << std::endl;
        f << "\topticalFlowThreshold: " << opticalFlowThreshold << std::endl;

        f << "\t////////////////// Settings for solvePnpRansac //////////////////" << std::endl;
        f << "\titerationsCount: " << iterationsCount_ << std::endl;
        f << "\treprojectionError: " << reprojectionError_ << std::endl;
        f << "\tconfidence: " << confidence_ << std::endl;
        f << "\tuseExtrinsicGuess: " << (useExtrinsicGuess_ ? "true" : "false") << std::endl;

        f << "\tflags: ";
        switch (flags_)
        {
            case cv::SOLVEPNP_ITERATIVE:
                f << "SOLVEPNP_ITERATIVE";
                break;
            case cv::SOLVEPNP_EPNP:
                f << "SOLVEPNP_EPNP";
                break;
            case cv::SOLVEPNP_AP3P:
                f << "SOLVEPNP_AP3P";
                break;
            case cv::SOLVEPNP_DLS:
                f << "SOLVEPNP_DLS";
                break;
            case cv::SOLVEPNP_UPNP:
                f << "SOLVEPNP_UPNP";
                break;
            case cv::SOLVEPNP_P3P:
                f << "SOLVEPNP_P3P";
                break;
        }
        f << std::endl;

        f << "\t////////////////// Settings for Bucketing //////////////////" << std::endl;
        f << "\tbucketSize: " << bucketSize_ << std::endl;
        f << "\tfeaturesPerBucket: " << featuresPerBucket_ << std::endl;

        f << "\t////////////////// Settings for FAST Detector //////////////////" << std::endl;
        f << "\tfastThreshold: " << fastThreshold_ << std::endl;
        f << "\tnonmaxSuppression: " << (nonmaxSuppression_ ? "true" : "false") << std::endl;

        f << "\t////////////////// Settings for 5pt algorithm //////////////////" << std::endl;
        f << "\testimateRotation5pt: " << (estimateRotation5Pt_ ? "true" : "false") << std::endl;

        f << "\t////////////////// Settings for Circular Matching //////////////////" << std::endl;
        f << "\tcuraCircularMatching: " << (cudaCircularMatching_ ? "true" : "false") << std::endl;
        f << "\twinSize: " << winSize_ << std::endl;
        f << "\tmaxLevel: " << maxLevel_ << std::endl;
        f << "\tminEigThreshold: " << minEigThreshold_ << std::endl;
        f << "\topticalFlowIters: " << opticalFlowIters_ << std::endl;
        f.close();
    }
    else
    {
        LOG(ERROR) << "VisualOdometryStereo could not open " << settingsFile;
    }
}
