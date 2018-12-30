#ifndef SYSTEM_H
#define SYSTEM_H

#include <opencv2/core.hpp>
#include <thread>
#include "visualOdometryStereo.h"
#include "mapDrawer.h"

typedef double PoseType;
typedef float PointType;
typedef float TimeType;
typedef float CamType;

class System
{
public:
    System(cv::FileStorage& fSettings);
    System(cv::FileStorage& fSettings, const std::vector<cv::Matx<double, 4, 4>> &gtPoses);
    ~System();

    void process(const cv::Mat& imageLeft, const cv::Mat& imageRight, const double& timestamp);

    void save();
private:
    cv::Mat imageLeftPrev_, imageRightPrev_;

    cv::Matx<PoseType, 3, 3> rotation_;
    cv::Matx<PoseType, 3, 1> translation_;
    cv::Matx<PoseType, 4, 4> frame_pose_;
    std::vector<cv::Matx<PoseType, 4, 4>> poses_;

    std::vector< cv::Point_<PointType> > pointsLeftPrev_, pointsRightPrev_, pointsLeftCurr_, pointsRightCurr_;   //vectors to store the coordinates of the feature points

    std::vector<double> timestamps_;

    VisualOdometryStereo vos_;
    MapDrawer mapDrawer_;
    std::thread mapDrawerThread_;

};

#endif //SYSTEM_H
