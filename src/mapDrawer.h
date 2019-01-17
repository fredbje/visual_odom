#ifndef MAP_DRAWER_H
#define MAP_DRAWER_H

#include <string>
#include <iostream>
#include <mutex>
#include <gtsam/geometry/Pose3.h>
#include "frame.h"
// For drawing
#include <pangolin/pangolin.h>

class MapDrawer{
public:
    MapDrawer(const std::vector<Frame>& frames, const std::vector<gtsam::Pose3>& gtPoses, std::mutex& mutexPoses);

    ~MapDrawer();

    void run();

    void requestFinish();

private:
    const std::vector<Frame>& frames_;
    const std::vector<gtsam::Pose3>& gtPoses_;

    enum Color { red, green, blue };
    void drawCamera(pangolin::OpenGlMatrix &Twc, Color color);
    void drawLines(pangolin::OpenGlMatrix T1, pangolin::OpenGlMatrix T2, Color color);

    std::mutex mutexFinish_;
    std::mutex& mutexPoses_;

    bool checkFinish();
    bool finishRequested_ = false;

    double viewpointX_ = 0;
    double viewpointY_ = -100;
    double viewpointZ_ = -0.1;
    double viewpointF_ = 2000;

};

#endif //MAP_DRAWER_H
