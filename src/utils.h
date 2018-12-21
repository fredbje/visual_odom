#ifndef UTILS_H
#define UTILS_H

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

#include "feature.h"
*/

#include <vector>
#include <string>
#include <opencv2/core.hpp>

std::vector<cv::Mat> loadPoses(std::string file_name);

void display(int frame_id, cv::Mat& trajectory, cv::Mat& pose, std::vector<cv::Mat>& pose_gt, float fps, bool showgt);

void integrateOdometryStereo(cv::Mat& frame_pose, const cv::Mat& rotation,
                            const cv::Mat& translation_stereo);

#endif // UTILS_H
