#ifndef MAP_DRAWER_H
#define MAP_DRAWER_H

#include <string>
#include <iostream>
#include <mutex>
#include <gtsam/geometry/Pose3.h>

// For drawing
#include <pangolin/pangolin.h>

class MapDrawer{
public:

    MapDrawer();

    MapDrawer(const std::vector<gtsam::Pose3> &gtPoses);

    ~MapDrawer();

    void setGtPoses(const std::vector<gtsam::Pose3> &gtPoses);

    void updateAllPoses(const std::vector<gtsam::Pose3>& poses);

    void updateNewPoses(const std::vector<gtsam::Pose3>& poses);

    void updateLastPose(const gtsam::Pose3& pose);

    void run();

    void requestFinish();

private:
    std::vector<gtsam::Pose3> poses_;
    std::vector<gtsam::Pose3> gtPoses_;

    enum Color { red, green, blue };
    void drawCamera(pangolin::OpenGlMatrix &Twc, Color color);
    void drawLines(pangolin::OpenGlMatrix T1, pangolin::OpenGlMatrix T2, Color color);

    // Return global translation matrix
    pangolin::OpenGlMatrix getOpenGlMatrix(const gtsam::Pose3& pose);

    std::mutex mutexPoses_, mutexFinish_;

    bool checkFinish();
    bool finishRequested_ = false;

    double viewpointX_ = 0;
    double viewpointY_ = -100;
    double viewpointZ_ = -0.1;
    double viewpointF_ = 2000;

};

#endif //MAP_DRAWER_H