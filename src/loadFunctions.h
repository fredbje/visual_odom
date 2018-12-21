#ifndef SFO_LOADFUNCTIONS_H
#define SFO_LOADFUNCTIONS_H

#include <vector>
#include <string>
#include <opencv2/core/mat.hpp>
#include "oxts.h"

bool loadImageFileNames(const std::string &strSequenceDir, std::vector<std::string> &vstrLeftImages,
                        std::vector<std::string> &vstrRightImages);
bool loadImages(cv::Mat &imgLeft, cv::Mat &imgRight, const std::string &strLeftImage,
                const std::string &strRightImage);
bool loadTimeStamps(const std::string &strTimestampsFile, std::vector<double> &vTimestamps);
bool loadGtPoses(const std::string &strGtPosesFile, std::vector<cv::Mat> &vGtPoses);
bool loadOxtsData(const std::string &strOxtsDir, std::vector<oxts> &vOxtsData);
bool loadCam2ImuTransform(const std::string& cam2ImuCalibFile, cv::Mat& imu_T_cam);

#endif //SFO_LOADFUNCTIONS_H
