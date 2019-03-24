#ifndef UTILS_H
#define UTILS_H

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "utils.h"
#include "matrixutils.h"
#include "easylogging++.h"
#include <gtsam/geometry/Pose3.h>

/*
void displayMap(int frame_id, cv::Mat& trajectory, cv::Matx<T, 3, 1>& translation, std::vector<cv::Matx<T, 4, 4>>& pose_gt_mat, bool show_gt)
{
    // draw estimated trajectory
    int x = int(translation(0)) + 300;
    int y = int(translation(2)) + 100;
    circle(trajectory, cv::Point(x, y), 1, CV_RGB(255,0,0), 2);

    if (show_gt)
    {
        // draw ground truth trajectory
        cv::Matx<T, 3, 1> pose_gt;
        pose_gt(0) = pose_gt_mat[frame_id](0, 3);
        pose_gt(1) = pose_gt_mat[frame_id](1, 3);
        pose_gt(2) = pose_gt_mat[frame_id](2, 3);
        x = int(pose_gt(0)) + 300;
        y = int(pose_gt(2)) + 100;
        circle(trajectory, cv::Point(x, y) ,1, CV_RGB(255,255,0), 2);
    }

    cv::imshow( "Trajectory", trajectory );
    cv::waitKey(1);
}
*/

inline void loadImageLeft(cv::Mat& image, int frame_id, std::string filepath){
    LOG(DEBUG) << "Loading left image";
    char file[200];
    sprintf(file, "image_0/%06d.png", frame_id);

    std::string filename = filepath + std::string(file);

    image = cv::imread(filename, cv::IMREAD_GRAYSCALE);
    if(image.empty())
    {
        LOG(WARNING) << "Could not load image from " << filename;
    }
}

inline void loadImageRight(cv::Mat& image, int frame_id, std::string filepath){
    LOG(DEBUG) << "Loading right image";
    char file[200];
    sprintf(file, "image_1/%06d.png", frame_id);

    std::string filename = filepath + std::string(file);

    image = cv::imread(filename, cv::IMREAD_GRAYSCALE);
    if(image.empty())
    {
        LOG(WARNING) << "Could not load image from " << filename;
    }
}

inline void displayFrame(const cv::Mat& image, const std::vector<cv::Point2f>& pointsPrev, const std::vector<cv::Point2f>& pointsCurr)
{
    static cv::Mat vis;
    cv::cvtColor(image, vis, cv::COLOR_GRAY2BGR, 3);
    assert(pointsPrev.size() == pointsCurr.size());
    for(unsigned int i = 0; i < pointsPrev.size(); i++)
    {
        circle(vis, cv::Point2f(pointsPrev[i].x, pointsPrev[i].y), 2, CV_RGB(0, 255, 0));
        circle(vis, cv::Point2f(pointsCurr[i].x, pointsCurr[i].y), 2, CV_RGB(255, 0, 0));
        cv::line(vis, pointsPrev[i], pointsCurr[i], CV_RGB(0, 255, 0));
    }
    cv::imshow("vis ", vis);
    // Wait 1ms (shortest time possible)
    cv::waitKey(1);
}

inline void saveTrajectoryRpg(std::string filename, const std::vector<gtsam::Pose3>& poses, const std::vector<double>& timestamps)
{
    std::ofstream fout(filename.c_str(), std::ofstream::out);
    if(!fout.is_open())
    {
        LOG(ERROR) << "Could not open " << filename;
        return;
    }

    double pi = 3.141592653589793238463;
    gtsam::Pose3 T = gtsam::Pose3(gtsam::Rot3::Rz(pi) * gtsam::Rot3::Ry(-pi/2) * gtsam::Rot3::Rz(pi/2), gtsam::Point3());

    std::cout << "T: " << T << std::endl;

    fout << "# time x y z qx qy qz qw" << std::endl;
    for(unsigned int i = 0; i < timestamps.size(); i++)
    {
        const gtsam::Pose3& pose = T*poses[i];
        gtsam::Quaternion q = pose.rotation().toQuaternion();

        fout << timestamps[i] << " " << pose.x() << " " << pose.y() << " " << pose.z() << " "
             << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }
    fout.close();
}


#endif // UTILS_H
