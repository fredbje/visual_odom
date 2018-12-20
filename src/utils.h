#ifndef UTILS_H
#define UTILS_H

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
#include "matrix.h"

void loadImageLeft(cv::Mat& image_gray, int frame_id, std::string filepath);

void loadImageRight(cv::Mat& image_gray, int frame_id, std::string filepath);

void display(int frame_id, cv::Mat& trajectory, cv::Mat& pose, std::vector<cv::Mat>& pose_gt, float fps, bool showgt);

void integrateOdometryStereo(int frame_id, cv::Mat& frame_pose, const cv::Mat& rotation, 
                            const cv::Mat& translation_stereo);

#endif // UTILS_H
