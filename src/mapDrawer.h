#ifndef MAP_DRAWER_H
#define MAP_DRAWER_H

#include <string>
#include <iostream>
#include <mutex>
//#include <Eigen/Eigen>
//#include <boost/thread.hpp>

//#include "libviso2/matrix.h"

//#include <Eigen/Core>
//#include <Eigen/Dense>

// For drawing
#include <pangolin/pangolin.h>

class MapDrawer{
public:

    MapDrawer();

    MapDrawer(const std::vector<cv::Matx<double, 4, 4>> &gtPoses);

    ~MapDrawer();

    void setGtPoses(const std::vector<cv::Matx<double, 4, 4>> &gtPoses);

    void updatePoses(const std::vector<cv::Matx<double, 4, 4>>& poses);

    void run();

    void requestFinish();

private:
    cv::Matx<double, 4, 4> pose_;
    std::vector<cv::Matx<double, 4, 4>> poses_;
    std::vector<cv::Matx<double, 4, 4>> gtPoses_;

    enum Color { red, green, blue };
    void drawCamera(pangolin::OpenGlMatrix &Twc, Color color);
    void drawLines(pangolin::OpenGlMatrix T1, pangolin::OpenGlMatrix T2, Color color);

    // Return global translation matrix
    pangolin::OpenGlMatrix getOpenGlMatrix(cv::Matx<double, 4, 4> pose);

    std::mutex mutexPoses_, mutexFinish_;

    bool checkFinish();
    bool finishRequested_ = false;

    double viewpointX_ = 0;
    double viewpointY_ = -100;
    double viewpointZ_ = -0.1;
    double viewpointF_ = 2000;

};

#endif //MAP_DRAWER_H