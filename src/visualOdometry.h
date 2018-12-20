#ifndef VISUAL_ODOM_H
#define VISUAL_ODOM_H

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

#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include <opencv2/core/eigen.hpp>


#include "bucket.h"
#include "utils.h"
*/

#include <string>
#include "feature.h"

void visualOdometry(int current_frame_id, std::string filepath,
                    cv::Mat& projMatrl, cv::Mat& projMatrr,
                    cv::Mat& rotation, cv::Mat& translation_mono, cv::Mat& translation_stereo, 
                    cv::Mat& image_left_t0,
                    cv::Mat& image_right_t0,
                    FeatureSet& current_features);

#endif
