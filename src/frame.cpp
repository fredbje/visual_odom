#include "frame.h"

Frame::Frame(const unsigned int& frameId,
             const unsigned int& refFrameId,
             const double& timestamp,
             const gtsam::Pose3& pose,
             const gtsam::Pose3& pose2Ref,
             const oxts& navData,
             bool isRef
             )
    : frameId_(frameId), refFrameId_(refFrameId), timestamp_(timestamp), pose_(pose), pose2Ref_(pose2Ref), navData_(navData), isRef_(isRef)
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

unsigned int Frame::getFrameId() const
{
    return frameId_;
}

unsigned int Frame::getRefFrameId() const
{
    return refFrameId_;
}

const gtsam::Pose3& Frame::getPose2Ref() const
{
    return pose2Ref_;
}

bool Frame::isRef() const
{
    return isRef_;
}
/*
void Frame::setReferenceKeyFrame(const KeyFrame& referenceKeyFrame)
{
    referenceKeyFrame_ = referenceKeyFrame;
}
*/
