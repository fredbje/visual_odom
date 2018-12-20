/*
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <ctype.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include <opencv2/core/eigen.hpp>

#include "feature.h"
#include "utils.h"
#include "evaluate_odometry.h"
#include "visualOdometry.h"

#include "matrixutils.h"
 */

#include "evaluate/evaluate_odometry.h"
#include "utils.h"
#include "visualOdometry.h"
#include "matrixutils.h"
#include "easylogging++.h"

INITIALIZE_EASYLOGGINGPP

using namespace std;

int main(int argc, char **argv)
{
    el::Loggers::configureFromGlobal("../configurations/easylogging.conf");

    // -----------------------------------------
    // Load images and configurations parameters
    // -----------------------------------------
    bool display_ground_truth = false;
    std::vector<Matrix> pose_matrix_gt;
    if(argc == 4)
    {   display_ground_truth = true;
        LOG(INFO) << "Display ground truth trajectory";
        // load ground truth pose
        string filename_pose = string(argv[3]);
        pose_matrix_gt = loadPoses(filename_pose);
    }
    if(argc < 3)
    {
        LOG(ERROR) << "Usage: ./run path_to_sequence path_to_calibration [optional]path_to_ground_truth_pose";
        return 1;
    }

    // Sequence
    string filepath = string(argv[1]);
    LOG(INFO) << "Filepath: " << filepath;

    // Camera configurations
    string strSettingPath = string(argv[2]);
    LOG(INFO) << "Calibration Filepath: " << strSettingPath;

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    float bf = fSettings["Camera.bf"];

    cv::Mat projMatrl = (cv::Mat_<float>(3, 4) << fx, 0., cx, 0., 0., fy, cy, 0., 0,  0., 1., 0.);
    cv::Mat projMatrr = (cv::Mat_<float>(3, 4) << fx, 0., cx, bf, 0., fy, cy, 0., 0,  0., 1., 0.);
    LOG(INFO) << "P_left: " << endl << projMatrl;
    LOG(INFO) << "P_right: " << endl << projMatrr;

    // -----------------------------------------
    // Initialize variables
    // -----------------------------------------
    cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat translation_mono = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat translation_stereo = cv::Mat::zeros(3, 1, CV_64F);

    cv::Mat pose = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat Rpose = cv::Mat::eye(3, 3, CV_64F);
    
    cv::Mat frame_pose = cv::Mat::eye(4, 4, CV_64F);
    // cv::hconcat(cv::Mat::eye(4, 4, CV_64F), cv::Mat::zeros(3, 1, CV_64F), frame_pose);
    // cv::vconcat(frame_pose, cv::Mat::zeros(1, 4, CV_64F), frame_pose);

    LOG(INFO) << "frame_pose " << frame_pose;

    cv::Mat trajectory = cv::Mat::zeros(600, 1200, CV_8UC3);

    FeatureSet current_features;

    int init_frame_id = 0;

    // ------------
    // Load first images
    // ------------
    cv::Mat image_left_t0_color,  image_left_t0;

    loadImageLeft(image_left_t0_color,  image_left_t0, init_frame_id, filepath);
    
    cv::Mat image_right_t0_color, image_right_t0;  
    loadImageRight(image_right_t0_color, image_right_t0, init_frame_id, filepath);

    float fps;

    // -----------------------------------------
    // Run visual odometry
    // -----------------------------------------
    // initializeImagesFeatures(init_frame_id, filepath, image_l, image_r, current_features);

    clock_t tic = clock();

    for (int frame_id = init_frame_id; frame_id < 9000; frame_id++)
    {

        LOG(DEBUG) << "frame_id " << frame_id;

        visualOdometry(frame_id, filepath,
                       projMatrl, projMatrr,
                       rotation, translation_mono, translation_stereo, 
                       image_left_t0, image_right_t0,
                       current_features);

        cv::Vec3f rotation_euler = rotationMatrixToEulerAngles(rotation);
        LOG(DEBUG) << "rotation: " << rotation_euler;
        LOG(DEBUG) << "translation: " << translation_stereo.t();

        if(abs(rotation_euler[1])<0.1 && abs(rotation_euler[0])<0.1 && abs(rotation_euler[2])<0.1)
        {
            integrateOdometryStereo(frame_id, frame_pose, rotation, translation_stereo);

        } else {

            LOG(WARNING) << "Too large rotation";
        }

        Rpose =  frame_pose(cv::Range(0, 3), cv::Range(0, 3));
        cv::Vec3f Rpose_euler = rotationMatrixToEulerAngles(Rpose);
        LOG(DEBUG) << "Rpose_euler" << Rpose_euler;

        cv::Mat pose = frame_pose.col(3).clone();

        clock_t toc = clock();
        fps = float(frame_id-init_frame_id)/(toc-tic)*CLOCKS_PER_SEC;

        pose = -pose;
        LOG(DEBUG) << "Pose" << pose.t();
        LOG(DEBUG) << "FPS: " << fps;

        display(frame_id, trajectory, pose, pose_matrix_gt, fps, display_ground_truth);
    }

    return 0;
}

