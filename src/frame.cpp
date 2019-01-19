#include "frame.h"

Frame::Frame(const unsigned int& frameId,
             const double& timestamp,
             const gtsam::Pose3& pose,
             //const gtsam::Pose3& pose2Ref,
             const oxts& navData
             //bool isRef
             )
    : frameId_(frameId), timestamp_(timestamp), pose_(pose), /*pose2Ref_(pose2Ref),*/ navData_(navData) /*, isRef_(isRef)*/
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

/*
const gtsam::Pose3& Frame::getPose2Ref() const
{
    return pose2Ref_;
}
*/

/*
void Frame::setReferenceKeyFrame(const KeyFrame& referenceKeyFrame)
{
    referenceKeyFrame_ = referenceKeyFrame;
}
*/
