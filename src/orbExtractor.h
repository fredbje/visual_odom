#ifndef SFO_ORBEXTRACTOR_H
#define SFO_ORBEXTRACTOR_H

#include <vector>
#include <string>

#include <DBoW2/include/DBoW2.h> // defines OrbVocabulary
#include <opencv2/core.hpp>

void changeStructure(const cv::Mat &plain, std::vector<cv::Mat> &out);

class OrbExtractor {
public:
    OrbExtractor();
    ~OrbExtractor();
    void operator()(const cv::Mat &im, std::vector<cv::KeyPoint> &keys, std::vector<DBoW2::FORB::TDescriptor> &descriptors) const;
    void saveSettings(const std::string& settingsFile);
private:
    // ----------------
    // Settings for ORB
    // ----------------
    int	nfeatures_ = 500;       // The maximum number of features to retain.
    float scaleFactor_ = 1.2f;  // Pyramid decimation ratio, greater than 1. scaleFactor==2 means the classical pyramid
    int nlevels_ = 8;           // The number of pyramid levels.
    int edgeThreshold_ = 31;    // This is size of the border where the features are not detected. It should roughly match the patchSize parameter.
    int firstLevel_ = 0;        // The level of pyramid to put source image to. Previous layers are filled with upscaled source image.
    int WTA_K_ = 2;             // The number of points that produce each element of the oriented BRIEF descriptor.
    cv::ORB::ScoreType scoreType_ = cv::ORB::HARRIS_SCORE; // FAST_SCORE is alternative value of the parameter that produces slightly less stable keypoints, but it is a little faster to compute.
    int patchSize_ = 31;        // size of the patch used by the oriented BRIEF descriptor.
    int fastThreshold_ = 20;

    cv::Ptr<cv::ORB> mpOrb;
};

// ----------------------------------------------------------------------------

#endif // SFO_ORBEXTRACTOR_H
