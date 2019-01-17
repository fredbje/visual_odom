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
//#include "loopDetector.h"
#include "stereocamera.h"

#include "stereocamera.h"

class GtsamOptimizer {
public:

    GtsamOptimizer(const StereoCamera& stereoCamera, const gtsam::Pose3& imu_T_cam);

    ~GtsamOptimizer();

    void addGpsPrior(const unsigned int& id, const oxts& navdata);

    void addPose(const gtsam::Pose3& estimate, const unsigned int& id, const double& timestamp);

    void addPose(const gtsam::Pose3& estimate, const unsigned int& id, const double& timestamp, const oxts& navdata);

    void addRelativePoseConstraint(const gtsam::Pose3& deltaT, unsigned int idFrom, unsigned int idTo, bool isLoopClosureConstraint);

    void optimize();

    std::vector<gtsam::Pose3> getCurrentTrajectoryEstimate();

    gtsam::Pose3 getCurrentPoseEstimate(unsigned int frameId);

    // From the matched feature pair to previous and current sterepoints
    /*
    void getMatchedPairs(const libviso2::Matcher::p_match &match,
                      gtsam::StereoPoint2 &p1,
                      gtsam::StereoPoint2 &p2);
    */
    void saveTrajectoryLatLon(const std::string& outputFile);

    void saveGraphAndValues(const std::string& outputFile);

    void saveSettings(const std::string& settingsFile);

private:
    StereoCamera stereoCamera_;

    std::vector<unsigned int> poseIds_;
    std::vector<double> timestamps_;
    //size_t mPoseId, mLandmarkId, mSwitchId;

    gtsam::noiseModel::Isotropic::shared_ptr mMeasurementNoise2D, mMeasurementNoise3D;
    gtsam::noiseModel::Diagonal::shared_ptr mOdometryNoise, gpsNoise_, mLoopClosureNoise;
    gtsam::StereoCamera mStereoCamera;

    gtsam::NonlinearFactorGraph mNewFactors;
    gtsam::Values mNewValues;
    gtsam::Values mCurrentEstimate;

    gtsam::ISAM2Params mParameters;
    gtsam::ISAM2 mIsam;

    bool localOriginSet_;
    bool firstPoseInitialized_;

    //GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
    GeographicLib::LocalCartesian enuProjection_;
};
#endif // GTSAM_OPTIMIZER_H
