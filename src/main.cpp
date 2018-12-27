#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "utils.h"
#include "visualOdometry.h"
#include "matrixutils.h"
#include "stereocamera.h"
#include "loadFunctions.h"
#include "mapDrawer.h"
#include <thread>
#include "easylogging++.h"

INITIALIZE_EASYLOGGINGPP


int main(int argc, char **argv)
{
    if(argc < 1)
    {
        LOG(ERROR) << "Usage: ./run path_to_settings";
        return 1;
    }

    // Settings file
    std::string strSettingPath = std::string(argv[1]);
    LOG(INFO) << "Calibration Filepath: " << strSettingPath;

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    if(!fSettings.isOpened())
    {
        LOG(ERROR) << "Could not open settings file";
    }

    if(fSettings["SequenceDirectory"].empty()
        || fSettings["TimestampsFile"].empty()
        || fSettings["GroundTruthFile"].empty()
        || fSettings["OxtsDirectory"].empty()
        || fSettings["Cam2ImuCalibrationFile"].empty()
        || fSettings["VocabularyFile"].empty()
        || fSettings["LogSettingsFile"].empty())
    {
        LOG(ERROR) << "Could not read settings file";
        return 1;
    }

    std::string sequenceDirectory = fSettings["SequenceDirectory"];
    std::string timestampsFile = fSettings["TimestampsFile"];
    std::string gtPosesFile = fSettings["GroundTruthFile"];
    std::string oxtsDirectory = fSettings["OxtsDirectory"];
    std::string cam2ImuCalibrationFile = fSettings["Cam2ImuCalibrationFile"];
    std::string vocabularyFile = fSettings["VocabularyFile"];
    std::string logSettingsFile = fSettings["LogSettingsFile"];

    el::Loggers::configureFromGlobal(logSettingsFile.c_str());

    std::vector<TimeType> timestamps;
    if(!loadTimeStamps(timestampsFile, timestamps))
    {
        LOG(ERROR) << "Could not load timestamps";
        return 1;
    }

    std::vector<oxts> oxtsData;
    if(!loadOxtsData(oxtsDirectory, oxtsData))
    {
        LOG(ERROR) << "Could not load oxts data";
        return 1;
    }

    cv::Matx<PoseType, 4, 4> imu_T_cam;
    if(!loadCam2ImuTransform(cam2ImuCalibrationFile, imu_T_cam))
    {
        LOG(ERROR) << "Could not load imu_T_cam matrix";
        return 1;
    }

    // -----------------------------------------
    // Load images and configurations parameters
    // -----------------------------------------
    bool displayGroundTruth = true;
    std::vector<cv::Matx<PoseType, 4, 4>> gtPoses;
    if(displayGroundTruth)
    {
        LOG(INFO) << "Display ground truth trajectory";

        if(!loadGtPoses(gtPosesFile, gtPoses))
        {
            LOG(ERROR) << "Could not open ground truth poses which was requested";
            return 1;
        }
    }

    // -----------------------------------------
    // Initialize variables
    // -----------------------------------------
    cv::Matx<PoseType, 3, 3> rotation = cv::Matx<PoseType, 3, 3>::eye();
    cv::Matx<PoseType, 3, 1> translation_stereo = cv::Matx<PoseType, 3, 1>::zeros();

    cv::Matx<PoseType, 3, 1> translation = cv::Matx<PoseType, 3, 1>::zeros();

    cv::Matx<PoseType, 4, 4> frame_pose = cv::Matx<PoseType, 4, 4>::eye();

    LOG(INFO) << "Frame_pose " << frame_pose;

    //cv::Mat trajectoryPlot = cv::Mat::zeros(600, 1200, CV_8UC3);

    FeatureSet<PointType> current_features;

    int init_frame_id = 0;

    // ------------
    // Load first images
    // ------------
    std::vector<std::string> imageFileNamesLeft, imageFileNamesRight;
    if(!loadImageFileNames(sequenceDirectory, imageFileNamesLeft, imageFileNamesRight))
    {
        LOG(ERROR) << "Could not load image file names.";
        return 1;
    }

    // -----------------------------------------
    // Run visual odometry
    // -----------------------------------------

    std::vector<cv::Matx<PoseType, 4, 4>> poses;
    poses.push_back(frame_pose);

    auto mapDrawer = new MapDrawer(gtPoses);
    std::thread mapDrawerThread(&MapDrawer::run, mapDrawer);
    mapDrawer->updatePoses(poses);

    float fps;
    clock_t tic = clock();
    VisualOdometryStereo vos(fSettings);
    cv::Mat imageLeftCurr, imageRightCurr, vis;
    int pointRadius = 2;

    for (unsigned int frame_id = 0; frame_id < 500/*imageFileNamesLeft.size()*/; frame_id++) //int frame_id = init_frame_id; frame_id < 9000; frame_id++) {
    {
        LOG(DEBUG) << "frame_id " << frame_id;

        // ------------
        // Load images
        // ------------
        loadImages(imageLeftCurr, imageRightCurr, imageFileNamesLeft[frame_id], imageFileNamesRight[frame_id]);

        std::vector< cv::Point_<PointType> > pointsLeftPrev, pointsRightPrev, pointsLeftCurr, pointsRightCurr;   //vectors to store the coordinates of the feature points
        if (vos.process(rotation, translation_stereo,
                        imageLeftCurr, imageRightCurr,
                        pointsLeftPrev,
                        pointsRightPrev,
                        pointsLeftCurr,
                        pointsRightCurr,
                        current_features)) {

            cv::Vec3d rotation_euler = rotationMatrixToEulerAngles(rotation);
            LOG(DEBUG) << "rotation: " << rotation_euler;
            LOG(DEBUG) << "translation: " << translation_stereo.t();

            if (abs(rotation_euler[1]) < 0.1 && abs(rotation_euler[0]) < 0.1 && abs(rotation_euler[2]) < 0.1) {
                integrateOdometryStereo(frame_pose, rotation, translation_stereo);
            } else {
                LOG(WARNING) << "Too large rotation";
            }
            poses.emplace_back(frame_pose);
            mapDrawer->updatePoses(poses);

            translation(0) = frame_pose(0, 3);
            translation(1) = frame_pose(1, 3);
            translation(2) = frame_pose(2, 3);

            clock_t toc = clock();
            fps = float(frame_id - init_frame_id) / (toc - tic) * CLOCKS_PER_SEC;

            translation = -translation;
            LOG(DEBUG) << "Translation" << translation.t();
            LOG(DEBUG) << "FPS: " << fps;

            /*
            display(frame_id, trajectoryPlot, translation, gtPoses, displayGroundTruth);
            */

            // -----------------------------------------
            // Display
            // -----------------------------------------
            cv::cvtColor(imageLeftCurr, vis, cv::COLOR_GRAY2BGR, 3);

            assert(pointsLeftPrev.size() == pointsLeftCurr.size());
            for(unsigned int i = 0; i < pointsLeftPrev.size(); i++)
            {
                circle(vis, cv::Point2f(pointsLeftPrev[i].x, pointsLeftPrev[i].y), pointRadius, CV_RGB(0, 255, 0));
                circle(vis, cv::Point2f(pointsLeftCurr[i].x, pointsLeftCurr[i].y), pointRadius, CV_RGB(255, 0, 0));
                cv::line(vis, pointsLeftPrev[i], pointsLeftCurr[i], CV_RGB(0, 255, 0));
            }

            cv::imshow("vis ", vis);
            cv::waitKey(1);
        }
    }
    mapDrawer->requestFinish();
    mapDrawerThread.join();
    delete mapDrawer;

    // -----------------------------------------
    // Write trajectory to file
    // -----------------------------------------

    std::ofstream fout("/home/fbjerkas/src/rpg_trajectory_evaluation/results/kitti/00/stamped_groundtruth.txt", std::ofstream::out);
    fout << "# time x y z qx qy qz qw" << std::endl;
    for(unsigned int i = 0; i < gtPoses.size(); i++)
    {
        cv::Matx<PoseType, 4, 4> T = gtPoses[i];
        PoseType x = T(0, 3);
        PoseType y = T(1, 3);
        PoseType z = T(2, 3);
        cv::Matx<PoseType, 3, 3> R(T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0), T(2, 1), T(2, 2));
        cv::Matx<PoseType, 4, 1> q = rotMat2Quat(R);
        fout << timestamps[i] << " " << x << " " << y << " " << z << " " << q(1) << " " << q(2) << " " << q(3) << " " << q(0) << std::endl;
    }
    fout.close();

    fout.open("/home/fbjerkas/src/rpg_trajectory_evaluation/results/kitti/00/stamped_traj_estimate.txt", std::ofstream::out);
    fout << "# time x y z qx qy qz qw" << std::endl;
    for(unsigned int i = 0; i < poses.size(); i++)
    {
        cv::Matx<PoseType, 4, 4> T = poses[i];
        PoseType x = T(0, 3);
        PoseType y = T(1, 3);
        PoseType z = T(2, 3);
        cv::Matx<PoseType, 3, 3> R(T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0), T(2, 1), T(2, 2));
        cv::Matx<PoseType, 4, 1> q = rotMat2Quat(R);
        fout << timestamps[i] << " " << x << " " << y << " " << z << " " << q(1) << " " << q(2) << " " << q(3) << " " << q(0) << std::endl;
    }
    fout.close();


    return 0;
}

