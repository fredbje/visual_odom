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
#include "frame.h"

class System
{
public:
    System(cv::FileStorage& fSettings, const std::string& vocabularyFile, const gtsam::Pose3& imuTcam);
    System(cv::FileStorage& fSettings, const std::string& vocabularyFile, const gtsam::Pose3& imuTcam, const std::vector<gtsam::Pose3>& gtPoses);
    ~System();

    void process(const cv::Mat& imageLeft, const cv::Mat& imageRight, const oxts& navData, const double& timestamp);

    void saveSettings(const std::string& settingsPath);

    void saveVoTimes(const std::string& outfile);

    void saveOverallTimes(const std::string& outFile);

    void saveLoopTimes(const std::string& outFile);

    void saveOptimizationTimes(const std::string& outFile);

    void save();

    enum class State
    {
        WaitingForFirstImage,
        Initialized
    };

private:
    // -----------------------
    // Settings
    // -----------------------
    bool optimize_ = true;
    bool closeLoops_ = true;
    bool useMapViewer_ = true;
    bool useFrameViewer_ = true;

    StereoCamera stereoCamera_;

    cv::Mat imageLeftPrev_, imageRightPrev_;
    cv::Mat imageLeftMatch_, imageRightMatch_;
    std::thread imageLeftLoaderThread_;
    std::thread imageRightLoaderThread_;

    gtsam::Pose3 framePose_;
    gtsam::Pose3 deltaTOdom_;
    gtsam::Pose3 deltaTMatch_;
    std::vector<gtsam::Pose3> poses_;
    std::vector<gtsam::Pose3> gtPoses_;
    std::vector<Frame> frames_;

    std::vector< cv::Point2f > pointsLeftPrev_, pointsRightPrev_, pointsLeftCurr_, pointsRightCurr_;

    std::vector<double> timestamps_;

    unsigned int frameIdCurr_, frameIdPrev_;
    int frameIdMatch_;

    VisualOdometryStereo vosOdom_;
    VisualOdometryStereo vosLoop_;
    MapDrawer mapDrawer_;
    std::thread mapDrawerThread_;

    GtsamOptimizer optimizer_;

    DLoopDetector::DetectionResult loopResult_;
    LoopDetector* loopDetector_;
    unsigned int numLoops_;

    State state_;

    std::mutex mutexPoses_;

    std::vector<float> voTimes_;
    std::vector<float> loopTimes_;
    std::vector<float> optimizationTimes_;
    std::vector<float> overallTimes_;


};

#endif //SYSTEM_H
