#include "visualOdometryStereo.h"
#include "matrixutils.h"
#include "loadFunctions.h"
#include "easylogging++.h"
#include "system.h"

INITIALIZE_EASYLOGGINGPP

int main(int argc, char **argv)
{
    // Settings file
    std::string strSettingPath = std::string(argv[1]);
    LOG(INFO) << "Calibration Filepath: " << strSettingPath;

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    if(!fSettings.isOpened())
    {
        LOG(ERROR) << "Could not open settings file";
    }

    std::string sequenceDirectory = "/home/fbjerkas/datasets/kitti-gray/sequences/00";
    std::string timestampsFile = "/home/fbjerkas/datasets/kitti-gray/sequences/00/times.txt";
    std::string gtPosesFile = "/home/fbjerkas/datasets/kitti-poses/dataset/poses/00.txt";
    std::string oxtsDirectory = "/home/fbjerkas/datasets/2011_10_03/2011_10_03_drive_0027_sync/oxts/data";
    std::string cam2ImuCalibrationFile = "/home/fbjerkas/datasets/2011_10_03/2011_10_03/calib_cam_to_imu.txt";
    std::string vocabularyFile = "/home/fbjerkas/src/visual_odom/vocabulary/ORBvoc.txt";
    std::string logSettingsFile = "/home/fbjerkas/src/visual_odom/configurations/easylogging.conf";

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

    gtsam::Pose3 imu_T_cam;
    if(!loadCam2ImuTransform(cam2ImuCalibrationFile, imu_T_cam))
    {
        LOG(ERROR) << "Could not load imu_T_cam matrix";
        return 1;
    }

    std::vector<gtsam::Pose3> gtPoses;
    if(!loadGtPoses(gtPosesFile, gtPoses))
    {
        LOG(ERROR) << "Could not open ground truth poses which was requested";
        return 1;
    }

    std::vector<std::string> imageFileNamesLeft, imageFileNamesRight;
    if(!loadImageFileNames(sequenceDirectory, imageFileNamesLeft, imageFileNamesRight))
    {
        LOG(ERROR) << "Could not load image file names.";
        return 1;
    }


    System SLAM(fSettings, imu_T_cam, gtPoses);
    cv::Mat imageLeft, imageRight;
    unsigned int frameIdInitial = 0;
    unsigned int frameIdFinal = imageFileNamesLeft.size();
    float avgFps, currFps;
    clock_t firstTic = clock();
    for (unsigned int frameId = frameIdInitial; frameId < frameIdFinal; frameId++)
    {
        clock_t tic = clock();
        loadImages(imageLeft, imageRight, imageFileNamesLeft[frameId], imageFileNamesRight[frameId]);

        SLAM.process(imageLeft, imageRight, oxtsData[frameId], timestamps[frameId]);

        clock_t toc = clock();
        currFps = 1.f / (toc - tic) * CLOCKS_PER_SEC;
        avgFps = float(frameId - frameIdInitial) / (toc - firstTic) * CLOCKS_PER_SEC;

        LOG(DEBUG) << "Current FPS: " << currFps;
        LOG(DEBUG) << "Average FPS: " << avgFps;
    }

    return 0;
}

