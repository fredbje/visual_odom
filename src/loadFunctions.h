#ifndef SFO_LOADFUNCTIONS_H
#define SFO_LOADFUNCTIONS_H

#include <vector>
#include <string>
#include <opencv2/core/mat.hpp>
#include "oxts.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/filesystem.hpp>
#include <opencv2/highgui.hpp>
#include "loadFunctions.h"
#include "easylogging++.h"
#include "matrixutils.h"
#include "gtsam/geometry/Pose3.h"

namespace bfs = boost::filesystem;

bool loadImageFileNames(const std::string &strSequenceDir, std::vector<std::string> &vstrLeftImages,
                        std::vector<std::string> &vstrRightImages)
{
    if(!bfs::is_directory(strSequenceDir))
    {
        LOG(ERROR) << strSequenceDir << " is not a directory.";
        return false;
    }

    LOG(INFO) << "Looking for images in " << strSequenceDir;

    bfs::path pathLeftImageDir = bfs::path(strSequenceDir) / "image_0";
    bfs::path pathRightImageDir = bfs::path(strSequenceDir) / "image_1";

    if(!bfs::is_directory(pathLeftImageDir))
    {
        LOG(ERROR) << pathLeftImageDir << " is not a directory";
        return false;
    }
    else if(!bfs::is_directory(pathRightImageDir))
    {
        LOG(ERROR) << pathRightImageDir << " is not a directory";
        return false;
    }

    for(auto& direntryFile : bfs::directory_iterator(pathLeftImageDir))
    {
        if(direntryFile.path().extension() == ".png")
        {
            vstrLeftImages.emplace_back(direntryFile.path().string());
        }
        else
        {
            LOG(ERROR) << direntryFile << " is not a png image";
            return false;
        }
    }

    for(auto& direntryFile : bfs::directory_iterator(pathRightImageDir))
    {
        if(direntryFile.path().extension() == ".png")
        {
            vstrRightImages.emplace_back(direntryFile.path().string());
        }
        else
        {
            LOG(ERROR) << direntryFile << " is not a png image";
            return false;
        }
    }

    if(vstrLeftImages.size() != vstrRightImages.size())
    {
        LOG(ERROR) << "The left and right image directories contains unequal amounts of images.";
        return false;
    }

    std::sort(vstrLeftImages.begin(), vstrLeftImages.end());
    std::sort(vstrRightImages.begin(), vstrRightImages.end());
    return true;
}

bool loadImages(cv::Mat &imgLeft, cv::Mat &imgRight, const std::string &strLeftImage,
                const std::string &strRightImage)
{
    imgLeft = cv::imread(strLeftImage, cv::IMREAD_GRAYSCALE);
    imgRight = cv::imread(strRightImage, cv::IMREAD_GRAYSCALE);

    if (imgLeft.empty())
    {
        LOG(ERROR) << "Couldn't read image from" << strLeftImage;
        return false;
    }
    else if(imgRight.empty())
    {
        LOG(ERROR) << "Couldn't read image from" << strRightImage;
        return false;
    }
    return true;
}

template <typename T>
bool loadTimeStamps(const std::string &strTimestampsFile, std::vector<T> &vTimestamps)
{

    if(!bfs::is_regular_file(strTimestampsFile))
    {
        LOG(ERROR) << strTimestampsFile << " is not a file.";
        return false;
    }

    std::ifstream fTimes;
    fTimes.open(strTimestampsFile.c_str());
    if(!fTimes.is_open())
    {
        LOG(ERROR) << "Could not open " << strTimestampsFile;
        return false;
    }

    while(!fTimes.eof())
    {
        std::string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            T t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }
    return true;
}

bool loadGtPoses(const std::string &strGtPosesFile, std::vector<gtsam::Pose3> &vGtPoses)
{
    if(!bfs::is_regular_file(bfs::path(strGtPosesFile)))
    {
        LOG(ERROR) << "Could not open GT poses at: " << strGtPosesFile;
        return false;
    }

    LOG(INFO) << "Loading GT poses from: " << strGtPosesFile;

    std::ifstream f;
    std::string strLine;

    f.open(strGtPosesFile.c_str(), std::ifstream::in);
    if(!f.is_open())
    {
        LOG(ERROR) << "Error, could not open ground truth pose file!";
        return false;
    }
/*
    // Data collected from oxts data frame 1.
    cv::Mat enu_R_imu = rotMatX(static_cast<float>(navdata0.roll))
            * rotMatY(static_cast<float>(navdata0.pitch))
            * rotMatZ(static_cast<float>(navdata0.yaw));

    cv::Mat enu_T_imu = cv::Mat::eye(4, 4, CV_32F);
    enu_R_imu.copyTo(enu_T_imu(cv::Rect(0, 0, enu_R_imu.cols, enu_R_imu.rows)));
*/
    while(std::getline(f, strLine))
    {
        std::istringstream iss(strLine);
        double r11, r12, r13, r21, r22, r23, r31, r32, r33, t1, t2, t3;
        if(!(iss >> r11 >> r12 >> r13 >> t1 >> r21 >> r22 >> r23 >> t2 >> r31 >> r32 >> r33 >> t3))
        {
            break;
        }

        Eigen::Matrix4d tempPose;
        tempPose(0, 0) = r11; tempPose(0, 1) = r12; tempPose(0, 2) = r13; tempPose(0, 3) = t1;
        tempPose(1, 0) = r21; tempPose(1, 1) = r22; tempPose(1, 2) = r23; tempPose(1, 3) = t2;
        tempPose(2, 0) = r31; tempPose(2, 1) = r32; tempPose(2, 2) = r33; tempPose(2, 3) = t3;
        tempPose(3, 0) = 0.0; tempPose(3, 1) = 0.0; tempPose(3, 2) = 0.0; tempPose(3, 3) = 1.0;
        vGtPoses.emplace_back(gtsam::Pose3(tempPose));
    }
    LOG(INFO) << "Finished loading GT poses.";
    return true;
}

bool loadOxtsData(const std::string &strOxtsDir, std::vector<oxts> &vOxtsData)
{
    if(!bfs::is_directory(strOxtsDir))
    {
        LOG(ERROR) << strOxtsDir << " is not a directory.";
        return false;
    }
    std::vector<std::string> vstrFileNames;
    for(auto &direntryFile : bfs::directory_iterator(bfs::path(strOxtsDir)))
    {
        if (direntryFile.path().extension() == ".txt")
        {
            vstrFileNames.emplace_back(direntryFile.path().string());
        }
        else
        {
            LOG(ERROR) << direntryFile << " does not have a .txt extension.";
            return false;
        }
    }
    std::sort(vstrFileNames.begin(), vstrFileNames.end());
    for(const auto &strFileName : vstrFileNames)
    {
        std::ifstream f;
        std::string strLine;
        f.open(strFileName, std::ifstream::in);
        if(!f.is_open())
        {
            LOG(ERROR) << "Error, could not open " << strFileName;
            return false;
        }
        std::getline(f, strLine);
        std::istringstream iss(strLine);
        oxts oxtsData;
        double navsat, numsats, posmode, velmode, orimode;
        if (!(iss >> oxtsData.lat
                  >> oxtsData.lon
                  >> oxtsData.alt
                  >> oxtsData.roll
                  >> oxtsData.pitch
                  >> oxtsData.yaw
                  >> oxtsData.vn
                  >> oxtsData.ve
                  >> oxtsData.vf
                  >> oxtsData.vl
                  >> oxtsData.vu
                  >> oxtsData.ax
                  >> oxtsData.ay
                  >> oxtsData.az
                  >> oxtsData.af
                  >> oxtsData.al
                  >> oxtsData.au
                  >> oxtsData.wx
                  >> oxtsData.wy
                  >> oxtsData.wz
                  >> oxtsData.wf
                  >> oxtsData.wl
                  >> oxtsData.wu
                  >> oxtsData.pos_accuracy
                  >> oxtsData.vel_accuracy
                  >> navsat
                  >> numsats
                  >> posmode
                  >> velmode
                  >> orimode)) {
            LOG(ERROR) << "Could not read from " << strFileName;
            return false;
        }
        oxtsData.navsat = static_cast<int>(navsat);
        oxtsData.numsats = static_cast<int>(numsats);
        oxtsData.posmode = static_cast<int>(posmode);
        oxtsData.velmode = static_cast<int>(velmode);
        oxtsData.orimode = static_cast<int>(orimode);
        vOxtsData.push_back(oxtsData);
    }
    return true;
}

bool loadCam2ImuTransform(const std::string& cam2ImuCalibFile, gtsam::Pose3& imu_T_cam)
{
    if(!bfs::is_regular_file(cam2ImuCalibFile))
    {
        LOG(ERROR) << cam2ImuCalibFile << " is not a file.";
        return false;
    }

    std::ifstream f;
    std::string strLine;
    f.open(cam2ImuCalibFile.c_str(), std::ifstream::in);
    if (!f.is_open())
    {
        LOG(ERROR) << "Could not open " << cam2ImuCalibFile;
        return false;
    }

    if (!std::getline(f, strLine))
    {
        LOG(ERROR) << "Could not read from " << cam2ImuCalibFile;
        return false;
    }

    std::istringstream iss(strLine);
    double r11, r12, r13, r21, r22, r23, r31, r32, r33, t1, t2, t3;
    if (!(iss >> r11 >> r12 >> r13 >> t1 >> r21 >> r22 >> r23 >> t2 >> r31 >> r32 >> r33 >> t3))
    {
        LOG(ERROR) << "Could not read stream from " << cam2ImuCalibFile;
        return false;
    }

    Eigen::Matrix4d temp;
    temp(0, 0) = r11; temp(0, 1) = r12; temp(0, 2) = r13; temp(0, 3) = t1;
    temp(1, 0) = r21; temp(1, 1) = r22; temp(1, 2) = r23; temp(1, 3) = t2;
    temp(2, 0) = r31; temp(2, 1) = r32; temp(2, 2) = r33; temp(2, 3) = t3;
    temp(3, 0) = 0.0; temp(3, 1) = 0.0; temp(3, 2) = 0.0; temp(3, 3) = 1.0;
    imu_T_cam = gtsam::Pose3(temp);

    LOG(INFO) << "Loaded camera to imu frame transformation.";
    return true;
}

#endif //SFO_LOADFUNCTIONS_H
