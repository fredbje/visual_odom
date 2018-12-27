#ifndef UTILS_H
#define UTILS_H

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "utils.h"
#include "easylogging++.h"

template <typename T>
void display(int frame_id, cv::Mat& trajectory, cv::Matx<T, 3, 1>& translation, std::vector<cv::Matx<T, 4, 4>>& pose_gt_mat, bool show_gt)
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
void integrateOdometryStereo(cv::Matx<T, 4, 4>& T_wc, const cv::Matx<T, 3, 3>& R_delta, const cv::Matx<T, 3, 1>& t_delta)
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

#endif // UTILS_H
