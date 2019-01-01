#ifndef SFO_ORBEXTRACTOR_H
#define SFO_ORBEXTRACTOR_H

#include <vector>
#include <string>

#include <DBoW2/include/DBoW2.h> // defines OrbVocabulary
#include <opencv2/core.hpp>

//#include "ORB_SLAM2/ORBextractor.h" // This class is a wrapper around the ORBextractor class from ORB_SLAM2

// ----------------------------------------------------------------------------

void changeStructure(const cv::Mat &plain, std::vector<cv::Mat> &out);

class OrbExtractor {
public:
    void operator()(const cv::Mat &im, std::vector<cv::KeyPoint> &keys, std::vector<DBoW2::FORB::TDescriptor> &descriptors) const;
    explicit OrbExtractor(const cv::FileStorage& fSettings);
    ~OrbExtractor();
private:
    //ORB_SLAM2::ORBextractor *mpOrb;
    cv::Ptr<cv::ORB> mpOrb;
};

// ----------------------------------------------------------------------------

#endif // SFO_ORBEXTRACTOR_H
