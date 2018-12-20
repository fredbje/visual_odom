#ifndef VISUAL_ODOM_H
#define VISUAL_ODOM_H

#include <string>
#include "feature.h"
#include "stereocamera.h"

class VisualOdometryStereo
{
public:
    explicit VisualOdometryStereo(StereoCamera stereoCamera) : stereoCamera_(std::move(stereoCamera)) {}
    ~VisualOdometryStereo() = default;

    void process(int current_frame_id, std::string filepath,
                        cv::Mat& rotation, cv::Mat& translation_mono, cv::Mat& translation_stereo,
                        cv::Mat& image_left_t1,
                        cv::Mat& image_right_t1,
                        cv::Mat& image_left_t0,
                        cv::Mat& image_right_t0,
                        std::vector<cv::Point2f>& points_left_t0,
                        std::vector<cv::Point2f>& points_right_t0,
                        std::vector<cv::Point2f>& points_left_t1,
                        std::vector<cv::Point2f>& points_right_t1,
                        std::vector<cv::Point2f>& points_left_t0_return,
                        FeatureSet& current_features);

private:
    void removeInvalidPoints(std::vector<cv::Point2f>& points, const std::vector<bool>& status);
    void checkValidMatch(std::vector<cv::Point2f>& points, std::vector<cv::Point2f>& points_return, std::vector<bool>& status);

private:
    StereoCamera stereoCamera_;
};



#endif
