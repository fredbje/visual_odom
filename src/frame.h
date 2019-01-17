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
          const double& timestamp,
          const gtsam::Pose3& pose,
          const oxts& navData);

    void updatePose(gtsam::Pose3 pose);

    const gtsam::Pose3& getPose() const;

    //void setReferenceKeyFrame(const KeyFrame& referenceKeyFrame);

private:
    unsigned int frameId_;
    double timestamp_;
    gtsam::Pose3 pose_;
    const oxts navData_;


    //    bool isKeyframe_;
    //    const KeyFrame& referenceKeyFrame_;
    //    gtsam::Pose3 pose2KeyFrame_;
    //    std::string filenameLeftImage_, filenameRightImage_;
};

#endif // FRAME_H
