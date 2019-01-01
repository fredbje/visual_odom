#ifndef SFO_LOOPDETECTOR_H
#define SFO_LOOPDETECTOR_H

#include <iostream>
#include <vector>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <DBoW2/include/DBoW2.h>
#include "DLoopDetector/include/DLoopDetector.h"
#include "orbExtractor.h"

// ----------------------------------------------------------------------------

class LoopDetector {
public:
    LoopDetector(const std::string &strVocabularyFile, const cv::FileStorage& fSettings);

    ~LoopDetector();

    // Add image to database and check for loops. Returns -1 if no loop, id of matched frame otherwise.
    void process(const cv::Mat &imgLeft, const cv::Mat &imgRight, DLoopDetector::DetectionResult &result);

    void saveDatabase(const std::string &strDatabaseFile);
    void loadDatabase(const std::string &strDatabaseFile);

protected:
    OrbLoopDetector::Parameters *mpParams;

    OrbVocabulary *mpVoc;
    OrbLoopDetector *mpDetector;
    OrbExtractor *mpExtractor;

    std::vector<cv::KeyPoint> mvKeys;
    std::vector<DBoW2::FORB::TDescriptor> mvDescriptors;
};

#endif //SFO_LOOPDETECTOR_H
