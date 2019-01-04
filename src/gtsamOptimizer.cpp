#define _USE_MATH_DEFINES
#include <cmath> // Can axess pi as M_PI
#include <fstream>
#include <iomanip> // setprecision

//#include <gtsam/nonlinear/Marginals.h>
//#include <opencv2/core/persistence.hpp>
//#include <opencv2/calib3d/calib3d.hpp> // For findEssentialMat

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam/slam/StereoFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/slam/PoseRotationPrior.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/dataset.h> // For save and load functions
//#include <gtsam/slam/BearingFactor.h>
//#include <gtsam/slam/SmartProjectionPoseFactor.h>
//#include <gtsam/geometry/EssentialMatrix.h>
//#include <gtsam/slam/EssentialMatrixConstraint.h>

//#include <gtsam_unstable/geometry/Similarity3.h>

//#include "vertigo/betweenFactorSwitchable.h"
//#include "vertigo/switchVariableLinear.h"

#include "gtsamOptimizer.h"
#include "utils.h"

GtsamOptimizer::GtsamOptimizer(const StereoCamera& stereoCamera, const gtsam::Pose3& imu_T_cam)
: stereoCamera_(stereoCamera), localOriginSet_(false), firstPoseInitialized_(false)
{
    mOdometryNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.05)).finished()); // 10cm std on x,y,z 0.05 rad on roll,pitch,yaw
    mLoopClosureNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.3), gtsam::Vector3::Constant(1.0)).finished());
    gpsNoise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 5.0, 5.0, 5.0).finished());

    mParameters.relinearizeThreshold = 0.01;
    mParameters.relinearizeSkip = 1;
    mIsam = gtsam::ISAM2(mParameters);
}

GtsamOptimizer::~GtsamOptimizer() {
    std::cout << "GtsamOptimizer destructor called." << std::endl;
}


void GtsamOptimizer::addPose(const gtsam::Pose3& estimate, const unsigned int& id, const double& timestamp)
{
    mNewValues.insert(gtsam::Symbol('x', id), estimate);
    poseIds_.push_back(id);
    timestamps_.push_back(timestamp);
    if(!firstPoseInitialized_)
    {
        gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3(0.2, 0.2, 0.2), gtsam::Vector3(1, 1, 1)).finished()); // Assuming 0.2 rad in roll, pitch, yaw
        mNewFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(gtsam::Symbol('x', id), estimate, priorPoseNoise);
        firstPoseInitialized_ = true;
    }
}

void GtsamOptimizer::addPose(const gtsam::Pose3& estimate, const unsigned int& id, const double& timestamp, const oxts& navdata)
{
    addPose(estimate, id, timestamp);
    addGpsPrior(id, navdata);
}

void GtsamOptimizer::addGpsPrior(const unsigned int& id, const oxts& navdata)
{
    if(!localOriginSet_)
    {
        enuProjection_.Reset(navdata.lat, navdata.lon, navdata.alt);
        localOriginSet_ = true;
        LOG(INFO) << "Initial global coordinates: " << std::setprecision(14) << navdata.lat << ", " << navdata.lon << ", " << navdata.alt;
    }

    double e, n, u;
    enuProjection_.Forward(navdata.lat, navdata.lon, navdata.alt, e, n, u);
    gtsam::Point3 enu(e, n, u);
    gtsam::GPSFactor gpsFactor(gtsam::Symbol('x', id), enu, gpsNoise_);
    mNewFactors.emplace_shared<gtsam::GPSFactor>(gpsFactor);
}

void GtsamOptimizer::addRelativePoseConstraint(const gtsam::Pose3& deltaT, unsigned int idFrom, unsigned int idTo, bool isLoopClosureConstraint)
{
    if(isLoopClosureConstraint)
    {
        mNewFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(gtsam::Symbol('x', idFrom), gtsam::Symbol('x', idTo), deltaT, mLoopClosureNoise);
    }
    else
    {
        mNewFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(gtsam::Symbol('x', idFrom), gtsam::Symbol('x', idTo), deltaT, mOdometryNoise);
    }
}

void GtsamOptimizer::optimize()
{
    mIsam.update(mNewFactors, mNewValues);
    mCurrentEstimate = mIsam.calculateEstimate();
    mNewFactors.resize(0);
    mNewValues.clear();
}

std::vector<gtsam::Pose3> GtsamOptimizer::getCurrentEstimate()
{
    std::vector<gtsam::Pose3> poses;
    for(const auto& poseId : poseIds_) {
        gtsam::Pose3 tempPose = mCurrentEstimate.at<gtsam::Pose3>(gtsam::Symbol('x', poseId));
        poses.emplace_back(tempPose);
    }
    return poses;
}

void GtsamOptimizer::saveTrajectoryLatLon(const std::string& outputFile) {
    LOG(INFO) << "Saving GPS track to file...";
    std::ofstream f;
    f.open(outputFile);

    for(const auto& poseId : poseIds_) {
        gtsam::Pose3 tempPose = mCurrentEstimate.at<gtsam::Pose3>(gtsam::Symbol('x', poseId));
        double lat, lon, h;
        enuProjection_.Reverse(tempPose.x(), tempPose.y(), tempPose.z(), lat, lon, h);
        f << std::setprecision(14) << lat << " " << lon << "\n";
    }
}

void GtsamOptimizer::saveGraphAndValues(const std::string& outputFile)
{
    gtsam::writeG2o(mIsam.getFactorsUnsafe(), mCurrentEstimate, outputFile);
}