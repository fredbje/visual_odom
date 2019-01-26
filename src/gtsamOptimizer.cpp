#define _USE_MATH_DEFINES
#include <cmath> // Can axess pi as M_PI
#include <fstream>
#include <iomanip> // setprecision

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

#include "vertigo/betweenFactorSwitchable.h"
#include "vertigo/switchVariableLinear.h"
#include "vertigo/gpsfactorswitchable.h"

#include "gtsamOptimizer.h"
#include "utils.h"

GtsamOptimizer::GtsamOptimizer(const StereoCamera& stereoCamera, const gtsam::Pose3& imu_T_cam)
: stereoCamera_(stereoCamera), localOriginSet_(false), firstPoseInitialized_(false), trajectoryInitializedInGlobalFrame_(false)
{
}

GtsamOptimizer::~GtsamOptimizer() {
    LOG(INFO) << "GtsamOptimizer destructor called.";
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

    static unsigned int count = 0;
    if (count++ % 10 != 0)
        return;

    double e, n, u;
    enuProjection_.Forward(navdata.lat, navdata.lon, navdata.alt, e, n, u);
    gtsam::Point3 enu(e, n, u);

    if(!trajectoryInitializedInGlobalFrame_)
    {
        gpsMeasurementBuffer_.push_back(std::make_pair(id, enu));

        double minEast, maxEast, minNorth, maxNorth;
        bool firstIteration = true;
        for( const auto& measurement : gpsMeasurementBuffer_ )
        {
            double east = measurement.second.x();
            double north = measurement.second.y();

            if( firstIteration )
            {
                minEast = east;
                maxEast = east;
                minNorth = north;
                maxNorth = north;
                firstIteration = false;
                continue;
            }

            if(east < minEast)
                minEast = east;
            else if(east > maxEast)
                maxEast = east;

            if(north < minNorth)
                minNorth = north;
            else if(north > maxNorth)
                maxNorth = north;
        }
        if(maxEast - minEast > 30.0 && maxNorth - minNorth > 30.0)
        {
            factorsToRemove_.push_back(0);

            for( const auto& measurement : gpsMeasurementBuffer_ )
            {
                gtsam::GPSFactor gpsFactor(gtsam::Symbol('x', measurement.first), measurement.second, gpsNoise_);
                mNewFactors.emplace_shared<gtsam::GPSFactor>(gpsFactor);
            }
            gpsMeasurementBuffer_.clear();
            trajectoryInitializedInGlobalFrame_ = true;
        }
    }
    else
    {


        //if (count++ % 100 == 0)
        //    enu = enu + gtsam::Point3(30, 0, 0);

        if( useSwitchableGpsConstraints_ )
        {
            double switchPrior = 1.0;
            static unsigned int switchIdGps = 0;
            mNewValues.insert(gtsam::Symbol('g', switchIdGps), vertigo::SwitchVariableLinear(switchPrior));
            mNewFactors.add(gtsam::PriorFactor<vertigo::SwitchVariableLinear>(gtsam::Symbol('g', switchIdGps), vertigo::SwitchVariableLinear(switchPrior), switchPriorNoise_));
            mNewFactors.emplace_shared<vertigo::GpsFactorSwitchableLinear>(gtsam::Symbol('x', id), gtsam::Symbol('g', switchIdGps++), enu, gpsNoise_);

        }
        else
        {
            gtsam::GPSFactor gpsFactor(gtsam::Symbol('x', id), enu, gpsNoise_);
            mNewFactors.emplace_shared<gtsam::GPSFactor>(gpsFactor);
        }
    }
}

void GtsamOptimizer::addRelativePoseConstraint(const gtsam::Pose3& deltaT, unsigned int idFrom, unsigned int idTo, bool isLoopClosureConstraint)
{
    if(isLoopClosureConstraint)
    {
        if( useSwitchableLoopConstraints_ )
        {
            double switchPrior = 1.0;
            static unsigned int switchId = 0;
            mNewValues.insert(gtsam::Symbol('s', switchId), vertigo::SwitchVariableLinear(switchPrior));
            mNewFactors.add(gtsam::PriorFactor<vertigo::SwitchVariableLinear>(gtsam::Symbol('s', switchId), vertigo::SwitchVariableLinear(switchPrior), switchPriorNoise_));
            mNewFactors.emplace_shared<vertigo::BetweenFactorSwitchableLinear<gtsam::Pose3>>(gtsam::Symbol('x', idFrom), gtsam::Symbol('x', idTo), gtsam::Symbol('s', switchId++), deltaT, loopClosureNoise_);
        }
        else
        {
            mNewFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(gtsam::Symbol('x', idFrom), gtsam::Symbol('x', idTo), deltaT, loopClosureNoise_);
        }
    }
    else
    {
        mNewFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(gtsam::Symbol('x', idFrom), gtsam::Symbol('x', idTo), deltaT, odometryNoise_);
    }
}

void GtsamOptimizer::optimize()
{
    iSAM2_.update(mNewFactors, mNewValues, factorsToRemove_);
    factorsToRemove_.clear();
    mCurrentEstimate = iSAM2_.calculateEstimate();
    mNewFactors.resize(0);
    mNewValues.clear();
}

std::vector<gtsam::Pose3> GtsamOptimizer::getCurrentTrajectoryEstimate()
{
    std::vector<gtsam::Pose3> poses;
    for(const auto& poseId : poseIds_) {
        gtsam::Pose3 tempPose = mCurrentEstimate.at<gtsam::Pose3>(gtsam::Symbol('x', poseId));
        poses.emplace_back(tempPose);
    }
    return poses;
}

gtsam::Pose3 GtsamOptimizer::getCurrentPoseEstimate(unsigned int frameId)
{
    return mCurrentEstimate.at<gtsam::Pose3>(gtsam::Symbol('x', frameId));
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
    gtsam::writeG2o(iSAM2_.getFactorsUnsafe(), mCurrentEstimate, outputFile);
}

void GtsamOptimizer::loadGraphAndValues(const std::string& inputFile)
{
    gtsam::NonlinearFactorGraph::shared_ptr actualGraph;
    gtsam::Values::shared_ptr actualValues;
    bool is3D = true;
    boost::tie(actualGraph, actualValues) = gtsam::readG2o(inputFile, is3D);
    // TODO actually use loaded graph and values
}

void GtsamOptimizer::saveSettings(const std::string& settingsFile)
{
    std::fstream f;
    f.open(settingsFile, std::ios_base::app);
    if(f.is_open())
    {
        f << "////////////////// GtsamOptimizer Settings //////////////////" << std::endl;
        f << "useSwitchableLoopConstraints: " << (useSwitchableLoopConstraints_ ? "true" : "false") << std::endl;
#ifdef USE_GN_PARAMS
        f << "OptimizationParams: GaussNewton" << std::endl;
        f << "wildfireThreshold: " << wildfireThreshold_ << std::endl;
#else
        f << "OptimizationParams: DogLeg" << std::endl;
        f << "initialDelta: " << initialDelta_ << std::endl;
        f << "wildfireThreshold: " << wildfireThreshold_;
        f << "adaptationMode: ";
        switch (adaptationMode_) {
        case gtsam::DoglegOptimizerImpl::SEARCH_EACH_ITERATION:
            f << "SEARCH_EACH_ITERATION";
            break;
        case gtsam::DoglegOptimizerImpl::SEARCH_REDUCE_ONLY:
            f << "SEARCH_REDUCE_ONLY";
            break;
        case gtsam::DoglegOptimizerImpl::ONE_STEP_PER_ITERATION:
            f << "ONE_STEP_PER_ITERATION";
        }
        f << std::endl;
        f << "verbose: " << (verbose_ ? "true" : "false") << std::endl;
#endif
        //f << "RelinearizationThreshold: " << relinearizationThreshold_ << std::endl;
        f << "relinearizeSkip: " << relinearizeSkip_ << std::endl;
        f << "enableRelinearization: " << (enableRelinearization_ ? "true" : "false") << std::endl;
        f << "evaluateNonLinearError: " << (evaluateNonlinearError_ ? "true" : "false") << std::endl;
        f << "factorization: ";
        switch (factorization_)
        {
        case gtsam::ISAM2Params::CHOLESKY:
            f << "CHOLESKY";
            break;
        case gtsam::ISAM2Params::QR:
            f << "QR";
            break;
        }
        f << std::endl;
        f << "cacheLinearizedFactors: " << (cacheLinearizedFactors_ ? "true" : "false") << std::endl;
    }
    else
    {
        LOG(ERROR) << "GtsamOptimizer could not open " << settingsFile;
    }
}
