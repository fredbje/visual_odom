#include "system.h"
#include "easylogging++.h"
#include "matrixutils.h"
#include "utils.h"

System::System(cv::FileStorage& fSettings, const std::string& vocabularyFile, const gtsam::Pose3& imuTcam)
        : stereoCamera_(fSettings), frameId_(0), vos_(stereoCamera_), optimizer(stereoCamera_, imuTcam), loopDetector(vocabularyFile, fSettings)
{
    mapDrawerThread_ = std::thread(&MapDrawer::run, &mapDrawer_);
}

System::System(cv::FileStorage& fSettings, const std::string& vocabularyFile, const gtsam::Pose3& imuTcam, const std::vector<gtsam::Pose3>& gtPoses)
: System(fSettings, vocabularyFile, imuTcam)
{
    mapDrawer_.setGtPoses(gtPoses);
}

System::~System()
{
    mapDrawer_.requestFinish();
    mapDrawerThread_.join();
}

void System::process(const cv::Mat& imageLeftCurr, const cv::Mat& imageRightCurr, const oxts& navData, const double& timestamp)
{

    DLoopDetector::DetectionResult loopResult;
    std::thread tLoopDetection(&LoopDetector::process, &loopDetector, imageLeftCurr, imageRightCurr, std::ref(loopResult));

    if(!imageLeftPrev_.empty())
    {
        pointsLeftPrev_.clear(); pointsRightPrev_.clear(); pointsLeftCurr_.clear(); pointsRightCurr_.clear();

        if (vos_.process(deltaT_,
                         imageLeftCurr, imageRightCurr,
                         imageLeftPrev_, imageRightPrev_,
                         pointsLeftPrev_,
                         pointsRightPrev_,
                         pointsLeftCurr_,
                         pointsRightCurr_)) {

            framePose_ = framePose_ * deltaT_;
            optimizer.addPose(framePose_, frameId_, timestamp);
            unsigned int lastFrameId = frameId_ - 1;
            optimizer.addRelativePoseConstraint(deltaT_, lastFrameId, frameId_);
        }
    }
    else
    {
        assert(imageRightPrev_.empty());
        framePose_ = gtsam::Pose3();
        optimizer.addPose(framePose_, frameId_, timestamp);
    }

    tLoopDetection.join();
    LOG(INFO) << loopResult.detection();
    timestamps_.push_back(timestamp);

    optimizer.optimize();
    poses_ = optimizer.getCurrentEstimate();
    framePose_ = poses_.back();
    mapDrawer_.updateAllPoses(poses_);

    // -----------------------------------------
    // Prepare image for next frame
    // -----------------------------------------
    displayFrame(imageLeftCurr, pointsLeftPrev_, pointsLeftCurr_);
    imageLeftPrev_ = imageLeftCurr;
    imageRightPrev_ = imageRightCurr;
    frameId_++;
}

void System::save()
{
    saveTrajectoryRpg(std::string("/home/fbjerkas/src/rpg_trajectory_evaluation/results/kitti/00_visual_odom_only/stamped_traj_estimate.txt"), poses_, timestamps_);
}