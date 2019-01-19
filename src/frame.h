#ifndef FRAME_H
#define FRAME_H

#include <string>
#include <gtsam/geometry/Pose3.h>
#include "oxts.h"

//class KeyFrame;

class Frame
{
public:
    Frame(const unsigned int& frameId,
          const unsigned int& refFrameId,
          const double& timestamp,
          const gtsam::Pose3& pose,
          const gtsam::Pose3& pose2Ref,
          const oxts& navData,
          bool isRef
          );

    void updatePose(gtsam::Pose3 pose);

    const gtsam::Pose3& getPose() const;

    unsigned int getFrameId() const;

    unsigned int getRefFrameId() const;

    const gtsam::Pose3& getPose2Ref() const;

    bool isRef() const;

    //void setReferenceKeyFrame(const KeyFrame& referenceKeyFrame);

private:
    unsigned int frameId_;
    unsigned int refFrameId_;
    double timestamp_;
    gtsam::Pose3 pose_;
    const gtsam::Pose3 pose2Ref_;
    const oxts navData_;
    bool isRef_;


    //    bool isKeyframe_;
    //    const KeyFrame& referenceKeyFrame_;
    //    gtsam::Pose3 pose2KeyFrame_;
    //    std::string filenameLeftImage_, filenameRightImage_;
};

#endif // FRAME_H
