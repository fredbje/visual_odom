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
    LoopDetector();
    LoopDetector(const std::string &strVocabularyFile, const cv::FileStorage& fSettings);

    ~LoopDetector();

    // Add image to database and check for loops. Returns -1 if no loop, id of matched frame otherwise.
    void process(const cv::Mat &image, DLoopDetector::DetectionResult &result);

    void saveDatabase(const std::string &strDatabaseFile);
    void loadDatabase(const std::string &strDatabaseFile);

protected:
    OrbLoopDetector::Parameters *mpParams;

    OrbVocabulary *mpVoc;
    OrbLoopDetector *mpDetector;
    OrbExtractor *mpExtractor;

    std::vector<cv::KeyPoint> mvKeys;
    std::vector<DBoW2::FORB::TDescriptor> mvDescriptors;

private:
    // --------------------------
    // Settings
    // --------------------------
    bool use_nss_ = true; // use normalized similarity score instead of raw score. Default true
    float alpha_ = 0.3; // nss threshold. Default 0.3
    int k_ = 3; // a loop must be consistent with 3 previous matches. Default 3
    DLoopDetector::GeometricalCheck geom_check_ = DLoopDetector::GEOM_DI; // use direct index for geometrical checking. Default GEOM_DI
    int di_levels_ = 3; // use three direct index levels. Default 0

    // To verify loops you can select one of the next geometrical checkings:
    // GEOM_EXHAUSTIVE: correspondence points are computed by comparing all
    //    the features between the two images.
    // GEOM_FLANN: as above, but the comparisons are done with a Flann structure,
    //    which makes them faster. However, creating the flann structure may
    //    be slow.
    // GEOM_DI: the direct index is used to select correspondence points between
    //    those features whose vocabulary node at a certain level is the same.
    //    The level at which the comparison is done is set by the parameter
    //    di_levels:
    //      di_levels = 0 -> features must belong to the same leaf (word).
    //         This is the fastest configuration and the most restrictive one.
    //      di_levels = l (l < L) -> node at level l starting from the leaves.
    //         The higher l, the slower the geometrical checking, but higher
    //         recall as well.
    //         Here, L stands for the depth levels of the vocabulary tree.
    //      di_levels = L -> the same as the exhaustive technique.
    // GEOM_NONE: no geometrical checking is done.
    //
    // In general, with a 10^6 vocabulary, GEOM_DI with 2 <= di_levels <= 4
    // yields the best results in recall/time.
    // Check the T-RO paper for more information.


    bool initialized_ = false;
};

#endif //SFO_LOOPDETECTOR_H
