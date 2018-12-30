#ifndef UTILS_H
#define UTILS_H

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "utils.h"
#include "matrixutils.h"
#include "easylogging++.h"

template <typename T>
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

template <typename T>
inline void integrateOdometryStereo(cv::Matx<T, 4, 4>& T_wc, const cv::Matx<T, 3, 3>& R_delta, const cv::Matx<T, 3, 1>& t_delta)
{
    cv::Matx<T, 4, 4> T_delta(
            R_delta(0, 0), R_delta(0, 1), R_delta(0, 2), t_delta(0),
            R_delta(1, 0), R_delta(1, 1), R_delta(1, 2), t_delta(1),
            R_delta(2, 0), R_delta(2, 1), R_delta(2, 2), t_delta(2),
            0.0, 0.0, 0.0, 1.0);

    double scale = sqrt((t_delta(0))*(t_delta(0))
                        + (t_delta(1))*(t_delta(1))
                        + (t_delta(2))*(t_delta(2))) ;

    LOG(INFO) << "scale: " << scale;

    // T_delta = {t-1}^T_{t}. Right multiply with last pose to get {0}^T_{t}
    if (scale > 0.05 && scale < 10)
    {
        T_wc = T_wc * T_delta;
    }
    else
    {
        LOG(WARNING) << "Scale below 0.1, or incorrect translation";
    }
}

template <typename T>
inline void displayFrame(const cv::Mat& image, const std::vector<cv::Point_<T>>& pointsPrev, const std::vector<cv::Point_<T>>& pointsCurr)
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
    cv::waitKey(1);
}

template <typename T1, typename T2>
void saveTrajectoryRpg(std::string filename, const std::vector<cv::Matx<T1, 4, 4>>& gtPoses, const std::vector<T2>& timestamps)
{
    std::ofstream fout(filename.c_str(), std::ofstream::out);
    if(!fout.is_open())
    {
        LOG(ERROR) << "Could not open " << filename;
        return;
    }

    fout << "# time x y z qx qy qz qw" << std::endl;
    for(unsigned int i = 0; i < gtPoses.size(); i++)
    {
        cv::Matx<T1, 4, 4> pose = gtPoses[i];
        cv::Matx<T1, 3, 1> translation(pose(0, 3), pose(1, 3), pose(2, 3));
        cv::Matx<T1, 3, 3> rotation(pose(0, 0), pose(0, 1), pose(0, 2), pose(1, 0), pose(1, 1), pose(1, 2), pose(2, 0), pose(2, 1), pose(2, 2));
        cv::Matx<T1, 4, 1> q = rotMat2Quat(rotation);
        fout << timestamps[i] << " " << translation(0) << " " << translation(1) << " " << translation(2) << " "
             << q(1) << " " << q(2) << " " << q(3) << " " << q(0) << std::endl;
    }
    fout.close();
}


#endif // UTILS_H
