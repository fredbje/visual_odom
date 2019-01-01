#ifndef SYSTEM_H
#define SYSTEM_H

#include <opencv2/core.hpp>
#include <thread>
#include "visualOdometryStereo.h"
#include "mapDrawer.h"
#include "oxts.h"
#include "stereocamera.h"
#include "gtsamOptimizer.h"
#include "loopDetector.h"

class System
{
public:
    System(cv::FileStorage& fSettings, const std::string& vocabularyFile, const gtsam::Pose3& imuTcam);
    System(cv::FileStorage& fSettings, const std::string& vocabularyFile, const gtsam::Pose3& imuTcam, const std::vector<gtsam::Pose3>& gtPoses);
    ~System();

    void process(const cv::Mat& imageLeft, const cv::Mat& imageRight, const oxts& navData, const double& timestamp);

    void save();
private:
    StereoCamera stereoCamera_;

    cv::Mat imageLeftPrev_, imageRightPrev_;

    gtsam::Pose3 framePose_;
    gtsam::Pose3 deltaT_;
    std::vector<gtsam::Pose3> poses_;

    std::vector< cv::Point2f > pointsLeftPrev_, pointsRightPrev_, pointsLeftCurr_, pointsRightCurr_;   //vectors to store the coordinates of the feature points

    std::vector<double> timestamps_;

    unsigned int frameId_;

    VisualOdometryStereo vos_;
    MapDrawer mapDrawer_;
    std::thread mapDrawerThread_;

    GtsamOptimizer optimizer;

    LoopDetector loopDetector;



};

#endif //SYSTEM_H
