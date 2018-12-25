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
    circle(trajectory, cv::Point(x, y) ,1, CV_RGB(255,0,0), 2);

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
    // print info

    // rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
    // sprintf(text, "FPS: %02f", fps);
    // putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

    cv::imshow( "Trajectory", trajectory );
    cv::waitKey(1);
}

template <typename T>
void integrateOdometryStereo(cv::Matx<T, 4, 4>& frame_pose, const cv::Matx<T, 3, 3>& rotation, const cv::Matx<T, 3, 1>& translation_stereo)
{
    cv::Matx<T, 4, 4> rigid_body_transformation(
            rotation(0, 0), rotation(0, 1), rotation(0, 2), translation_stereo(0),
            rotation(1, 0), rotation(1, 1), rotation(1, 2), translation_stereo(1),
            rotation(2, 0), rotation(2, 1), rotation(2, 2), translation_stereo(2),
            0.0, 0.0, 0.0, 1.0);
    //cv::Matx14d addup(0.0, 0.0, 0.0, 1.0);

    //cv::hconcat(rotation, translation_stereo, rigid_body_transformation);
    //cv::vconcat(rigid_body_transformation, addup, rigid_body_transformation);

    double scale = sqrt((translation_stereo(0))*(translation_stereo(0))
                        + (translation_stereo(1))*(translation_stereo(1))
                        + (translation_stereo(2))*(translation_stereo(2))) ;

    // frame_pose = frame_pose * rigid_body_transformation;
    LOG(INFO) << "scale: " << scale;

    // if ((scale>0.1)&&(translation_stereo.at<double>(2) > translation_stereo.at<double>(0)) && (translation_stereo.at<double>(2) > translation_stereo.at<double>(1)))
    if (scale > 0.05 && scale < 10)
    {
        frame_pose = frame_pose * rigid_body_transformation;
    }
    else
    {
        LOG(WARNING) << "Scale below 0.1, or incorrect translation";
    }
}


#endif // UTILS_H
