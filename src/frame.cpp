#include "frame.h"

Frame::Frame(const unsigned int& frameId,
             const double& timestamp,
             const gtsam::Pose3& pose,
             const oxts& navData)
    : frameId_(frameId), timestamp_(timestamp), pose_(pose), navData_(navData)
{

}

void Frame::updatePose(gtsam::Pose3 pose)
{
    pose_ = pose;
}

const gtsam::Pose3& Frame::getPose() const
{
    return pose_;
}

/*
void Frame::setReferenceKeyFrame(const KeyFrame& referenceKeyFrame)
{
    referenceKeyFrame_ = referenceKeyFrame;
}
*/
