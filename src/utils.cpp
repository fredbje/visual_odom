#include "utils.h"
#include "evaluate_odometry.h"
#include "easylogging++.h"

void loadImageLeft(cv::Mat& image_gray, int frame_id, std::string filepath){
    char file[200];
    sprintf(file, "image_0/%06d.png", frame_id);

    std::string filename = filepath + std::string(file);

    image_gray = cv::imread(filename, cv::IMREAD_GRAYSCALE);
}

void loadImageRight(cv::Mat& image_gray, int frame_id, std::string filepath){
    char file[200];
    sprintf(file, "image_1/%06d.png", frame_id);

    std::string filename = filepath + std::string(file);

    image_gray = cv::imread(filename, cv::IMREAD_GRAYSCALE);
}

void display(int frame_id, cv::Mat& trajectory, cv::Mat& pose, std::vector<cv::Mat>& pose_gt_mat, float fps, bool show_gt)
{
    // draw estimated trajectory 
    int x = int(pose.at<double>(0)) + 300;
    int y = int(pose.at<double>(2)) + 100;
    circle(trajectory, cv::Point(x, y) ,1, CV_RGB(255,0,0), 2);

    if (show_gt)
    {
      // draw ground truth trajectory 
      cv::Mat pose_gt = cv::Mat::zeros(1, 3, CV_32F);
      
      pose_gt.at<float>(0) = pose_gt_mat[frame_id].at<float>(0, 3);
      pose_gt.at<float>(1) = pose_gt_mat[frame_id].at<float>(1, 3);
      pose_gt.at<float>(2) = pose_gt_mat[frame_id].at<float>(2, 3);
      x = int(pose_gt.at<float>(0)) + 300;
      y = int(pose_gt.at<float>(2)) + 100;
      circle(trajectory, cv::Point(x, y) ,1, CV_RGB(255,255,0), 2);
    }
    // print info

    // rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
    // sprintf(text, "FPS: %02f", fps);
    // putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

    cv::imshow( "Trajectory", trajectory );
    cv::waitKey(1);
}

void integrateOdometryStereo(int frame_i, cv::Mat& frame_pose, const cv::Mat& rotation, const cv::Mat& translation_stereo)
{
    cv::Mat rigid_body_transformation;
    cv::Mat addup = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);

    cv::hconcat(rotation, translation_stereo, rigid_body_transformation);
    cv::vconcat(rigid_body_transformation, addup, rigid_body_transformation);

    double scale = sqrt((translation_stereo.at<double>(0))*(translation_stereo.at<double>(0)) 
                        + (translation_stereo.at<double>(1))*(translation_stereo.at<double>(1))
                        + (translation_stereo.at<double>(2))*(translation_stereo.at<double>(2))) ;

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
















