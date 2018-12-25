#ifndef FEATURESET_H
#define FEATURESET_H

#include <vector>
#include <opencv2/core/types.hpp>

template <typename T>
struct FeatureSet {
    std::vector<cv::Point_<T>> points;
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
