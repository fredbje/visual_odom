#ifndef FEATURESET_H
#define FEATURESET_H

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


#endif
