#ifndef FEATURE_H
#define FEATURE_H

#include <vector>
#include <opencv2/core/types.hpp>

struct FeatureSet {
    std::vector<cv::Point2f> points;
    std::vector<int> ages;
    unsigned long size(){
        return points.size();
    }
    void clear(){
        points.clear();
        ages.clear();
    }
 };

void featureDetectionFast(cv::Mat image, std::vector<cv::Point2f>& points);

void deleteUnmatchFeaturesCircle(std::vector<cv::Point2f>& points0, std::vector<cv::Point2f>& points1,
                          std::vector<cv::Point2f>& points2, std::vector<cv::Point2f>& points3,
                          std::vector<cv::Point2f>& points0_return,
                          std::vector<uchar>& status0, std::vector<uchar>& status1,
                          std::vector<uchar>& status2, std::vector<uchar>& status3,
                          std::vector<int>& ages);

void circularMatching(cv::Mat img_l_0, cv::Mat img_r_0, cv::Mat img_l_1, cv::Mat img_r_1,
                      std::vector<cv::Point2f>& points_l_0, std::vector<cv::Point2f>& points_r_0,
                      std::vector<cv::Point2f>& points_l_1, std::vector<cv::Point2f>& points_r_1,
                      std::vector<cv::Point2f>& points_l_0_return,
                      FeatureSet& current_features);

void bucketingFeatures(int image_height, int image_width, FeatureSet& current_features, int bucket_size, int features_per_bucket);

void appendNewFeatures(cv::Mat& image, FeatureSet& current_features);

#endif
