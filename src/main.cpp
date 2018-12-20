#include "evaluate/evaluate_odometry.h"
#include "utils.h"
#include "visualOdometry.h"
#include "matrixutils.h"
#include "stereocamera.h"
#include "easylogging++.h"

INITIALIZE_EASYLOGGINGPP

int main(int argc, char **argv)
{
    el::Loggers::configureFromGlobal("../configurations/easylogging.conf");

    // -----------------------------------------
    // Load images and configurations parameters
    // -----------------------------------------
    bool display_ground_truth = false;
    std::vector<cv::Mat> pose_matrix_gt;
    if(argc == 4)
    {   display_ground_truth = true;
        LOG(INFO) << "Display ground truth trajectory";
        // load ground truth pose
        std::string filename_pose = std::string(argv[3]);
        pose_matrix_gt = loadPoses(filename_pose);
    }
    if(argc < 3)
    {
        LOG(ERROR) << "Usage: ./run path_to_sequence path_to_calibration [optional]path_to_ground_truth_pose";
        return 1;
    }

    // Sequence
    std::string filepath = std::string(argv[1]);
    LOG(INFO) << "Filepath: " << filepath;

    // Camera configurations
    std::string strSettingPath = std::string(argv[2]);
    LOG(INFO) << "Calibration Filepath: " << strSettingPath;

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    StereoCamera stereoCamera(fSettings["Camera.f"],
            fSettings["Camera.cx"],
            fSettings["Camera.cy"],
            fSettings["Camera.bf"]);

    LOG(INFO) << "P_left: " << std::endl << stereoCamera.projMatL();
    LOG(INFO) << "P_right: " << std::endl << stereoCamera.projMatR();

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
    cv::Mat image_left_t0, image_right_t0;
    loadImageLeft(image_left_t0, init_frame_id, filepath);
    loadImageRight(image_right_t0, init_frame_id, filepath);

    float fps;

    // -----------------------------------------
    // Run visual odometry
    // -----------------------------------------
    // initializeImagesFeatures(init_frame_id, filepath, image_l, image_r, current_features);

    clock_t tic = clock();
    VisualOdometryStereo vos(stereoCamera);
    for (int frame_id = init_frame_id; frame_id < 9000; frame_id++)
    {
        LOG(DEBUG) << "frame_id " << frame_id;

        // ------------
        // Load images
        // ------------
        cv::Mat image_left_t1, image_right_t1;
        loadImageLeft(image_left_t1, frame_id + 1, filepath);
        loadImageRight(image_right_t1, frame_id + 1, filepath);

        std::vector<cv::Point2f>  points_left_t0, points_right_t0, points_left_t1, points_right_t1, points_left_t0_return;   //vectors to store the coordinates of the feature points
        vos.process(frame_id, filepath,
                rotation, translation_mono, translation_stereo,
                image_left_t1, image_right_t1,
                image_left_t0, image_right_t0,
                points_left_t0,
                points_right_t0,
                points_left_t1,
                points_right_t1,
                points_left_t0_return,
                current_features);

        cv::Vec3f rotation_euler = rotationMatrixToEulerAngles(rotation);
        LOG(DEBUG) << "rotation: " << rotation_euler;
        LOG(DEBUG) << "translation: " << translation_stereo.t();

        if( abs(rotation_euler[1]) < 0.1 && abs(rotation_euler[0]) < 0.1 && abs(rotation_euler[2]) < 0.1 )
        {
            integrateOdometryStereo(frame_id, frame_pose, rotation, translation_stereo);
        }
        else
        {
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

        // -----------------------------------------
        // Prepare image for next frame
        // -----------------------------------------
        image_left_t0 = image_left_t1;
        image_right_t0 = image_right_t1;

        // -----------------------------------------
        // Display
        // -----------------------------------------

        int radius = 2;
        // cv::Mat vis = image_left_t0.clone();

        cv::Mat vis;

        cv::cvtColor(image_left_t1, vis, cv::COLOR_GRAY2BGR, 3);


        for ( const auto& point_left_t0 : points_left_t0 )
        {
            circle(vis, cv::Point2f(point_left_t0.x, point_left_t0.y), radius, CV_RGB(0,255,0));
        }

        for ( const auto& point_left_t1 : points_left_t1 )
        {
            circle(vis, cv::Point2f(point_left_t1.x, point_left_t1.y), radius, CV_RGB(255,0,0));
        }

        assert(points_left_t0.size() == points_left_t1.size());
        for ( unsigned int i = 0; i < points_left_t1.size(); i++ )
        {
            cv::line(vis, points_left_t0[i], points_left_t1[i], CV_RGB(0,255,0));
        }

        imshow("vis ", vis );
    }

    return 0;
}

