#include "system.h"
#include "easylogging++.h"
#include "matrixutils.h"
#include "utils.h"

System::System(cv::FileStorage& fSettings)
        : vos_(fSettings)
{
    mapDrawerThread_ = std::thread(&MapDrawer::run, &mapDrawer_);
}

System::System(cv::FileStorage& fSettings, const std::vector<cv::Matx<double, 4, 4>> &gtPoses)
: System(fSettings)
{
    mapDrawer_.setGtPoses(gtPoses);
}

System::~System()
{
    mapDrawer_.requestFinish();
    mapDrawerThread_.join();
}

void System::process(const cv::Mat& imageLeftCurr, const cv::Mat& imageRightCurr, const double& timestamp)
{
    if(!imageLeftPrev_.empty())
    {
        pointsLeftPrev_.clear(); pointsRightPrev_.clear(); pointsLeftCurr_.clear(); pointsRightCurr_.clear();

        if (vos_.process(rotation_, translation_,
                         imageLeftCurr, imageRightCurr,
                         imageLeftPrev_, imageRightPrev_,
                         pointsLeftPrev_,
                         pointsRightPrev_,
                         pointsLeftCurr_,
                         pointsRightCurr_)) {

            cv::Vec3d rotation_euler = rotationMatrixToEulerAngles(rotation_);

            if (abs(rotation_euler[1]) < 0.1 && abs(rotation_euler[0]) < 0.1 && abs(rotation_euler[2]) < 0.1) {
                integrateOdometryStereo(frame_pose_, rotation_, translation_);
            } else {
                LOG(WARNING) << "Too large rotation";
            }

            displayFrame(imageLeftCurr, pointsLeftPrev_, pointsLeftCurr_);
        }
    }
    else
    {
        assert(imageRightPrev_.empty());
        rotation_ = cv::Matx<PoseType, 3, 3>::eye();
        translation_ = cv::Matx<PoseType, 3, 1>::zeros();
        frame_pose_ = cv::Matx<PoseType, 4, 4>::eye();
    }

    timestamps_.push_back(timestamp);
    poses_.push_back(frame_pose_);
    mapDrawer_.updatePoses(poses_);

    // -----------------------------------------
    // Prepare image for next frame
    // -----------------------------------------
    imageLeftPrev_ = imageLeftCurr;
    imageRightPrev_ = imageRightCurr;
}

void System::save()
{
    saveTrajectoryRpg(std::string("/home/fbjerkas/src/rpg_trajectory_evaluation/results/kitti/00_visual_odom_only/stamped_traj_estimate.txt"), poses_, timestamps_);
}