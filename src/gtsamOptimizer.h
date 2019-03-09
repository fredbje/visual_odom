#ifndef GTSAM_OPTIMIZER_H
#define GTSAM_OPTIMIZER_H

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
//#include <gtsam/geometry/StereoPoint2.h>
//#include <gtsam/geometry/Cal3_S2Stereo.h>
//#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h> // For noisemodel
#include <gtsam/nonlinear/ISAM2.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>


#include <Eigen/Core>
#include <Eigen/Dense>

#include "oxts.h"
#include "matrixutils.h"

class GtsamOptimizer {
private:
    struct Graph
    {
        gtsam::ISAM2 iSAM2_;
        gtsam::Values currentEstimate_;
        gtsam::FastVector<size_t> factorsToRemove_;

        bool firstPoseInitialized_ = false;
        bool trajectoryInitializedInGlobalFrame_ = false;
        std::vector<unsigned int> poseIds_;
        std::vector<double> timestamps_;
        gtsam::NonlinearFactorGraph newFactors_;
        gtsam::Values newValues_;
        std::vector<std::pair<unsigned int, gtsam::Point3>> gpsMeasurementBuffer_;

        Graph(gtsam::ISAM2 iSAM2) : iSAM2_(iSAM2) {}
    };
public:

    GtsamOptimizer(const gtsam::Pose3& imu_T_cam);

    ~GtsamOptimizer();

    void addGpsPrior(const unsigned int& id, const oxts& navdata);

    void addPose(const gtsam::Pose3& estimate, const unsigned int& id, const double& timestamp);

    void addPose(const gtsam::Pose3& estimate, const unsigned int& id, const double& timestamp, const oxts& navdata);

    void addRelativePoseConstraint(const gtsam::Pose3& deltaT, unsigned int idFrom, unsigned int idTo, bool isLoopClosureConstraint);

    void optimize();

    std::vector<gtsam::Pose3> getCurrentTrajectoryEstimate();

    gtsam::Pose3 getCurrentPoseEstimate(unsigned int frameId);

    void setTrackLost();

    void saveTrajectoryLatLon(const std::string& outputFile);

    void saveTrajectoryLatLon(const std::string& outputFile, const std::vector<gtsam::Pose3>& poses);

    void saveGraphAndValues(const std::string& outputFile);

    void loadGraphAndValues(const std::string& inputFile);

    void saveSettings(const std::string& settingsFile);

private:
    bool isGpsBufferDiverse(const std::vector<std::pair<unsigned int, gtsam::Point3>>& gpsMeasurementBuffer);

    void insertGpsBufferInGraph(Graph& graph, unsigned int& switchIdGps);

    unsigned int getGraphIdFromFrameId(unsigned int frameId);

private:
    // --------
    // Settings
    // --------
    bool useSwitchableLoopConstraints_ = true;
    bool useSwitchableGpsConstraints_ = false;

#define USE_GN_PARAMS
#ifdef USE_GN_PARAMS
    double wildfireThreshold_ = 0.001; // Continue updating the linear delta only when changes are above this threshold (default: 0.001)
    gtsam::ISAM2Params::OptimizationParams optimizationParams_ = gtsam::ISAM2GaussNewtonParams(wildfireThreshold_);
#else
    double initialDelta_ = 1.0;
    double wildfireThreshold_ = 1e-5;
    gtsam::DoglegOptimizerImpl::TrustRegionAdaptationMode adaptationMode_ = gtsam::DoglegOptimizerImpl::SEARCH_EACH_ITERATION; //SEARCH_REDUCE_ONLY
    bool verbose_ = false;
    gtsam::ISAM2Params::OptimizationParams optimizationParams_ = gtsam::ISAM2DoglegParams(initialDelta_, wildfireThreshold_, adaptationMode_, verbose_);
#endif

    gtsam::ISAM2Params::RelinearizationThreshold relinearizationThreshold_ = 0.1;
    int relinearizeSkip_ = 10;
    bool enableRelinearization_ = true;
    bool evaluateNonlinearError_ = false;
    gtsam::ISAM2Params::Factorization factorization_ = gtsam::ISAM2Params::CHOLESKY;
    bool cacheLinearizedFactors_ = true;
    gtsam::KeyFormatter keyFormatter_ = gtsam::DefaultKeyFormatter;

    gtsam::ISAM2Params iSAM2Params_ = gtsam::ISAM2Params(optimizationParams_,
                                                        relinearizationThreshold_,
                                                        relinearizeSkip_,
                                                        enableRelinearization_,
                                                        evaluateNonlinearError_,
                                                        factorization_,
                                                        cacheLinearizedFactors_,
                                                        keyFormatter_);

    gtsam::noiseModel::Isotropic::shared_ptr measurementNoise2D_;
    gtsam::noiseModel::Isotropic::shared_ptr measurementNoise3D_;
    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.05)).finished()); // 10cm std on x,y,z 0.05 rad on roll,pitch,yaw;
    gtsam::noiseModel::Diagonal::shared_ptr gpsNoise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 5.0, 5.0, 5.0).finished());
    gtsam::noiseModel::Diagonal::shared_ptr loopClosureNoise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.3), gtsam::Vector3::Constant(1.0)).finished());
    gtsam::noiseModel::Diagonal::shared_ptr switchPriorNoise_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(1.0));



    std::vector<unsigned int> poseIds_;
    std::vector<double> timestamps_;

    unsigned int currentGraphId_;

    std::vector<Graph> graphs_;

    bool localOriginSet_ = false;
    GeographicLib::LocalCartesian enuProjection_;

};
#endif // GTSAM_OPTIMIZER_H
