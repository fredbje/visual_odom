#include <DLib/include/DUtils/DUtils.h>
#include <DLib/include/DUtilsCV/DUtilsCV.h>
#include <DLib/include/DVision/DVision.h>
#include "loopDetector.h"

// ----------------------------------------------------------------------------
LoopDetector::LoopDetector() = default;

LoopDetector::LoopDetector(const std::string &strVocabularyFile, const cv::FileStorage& fSettings) {

    int height = fSettings["Camera.height"];
    int width = fSettings["Camera.width"];
    float frequency = fSettings["Camera.fps"];
    bool use_nss = true; // use normalized similarity score instead of raw score. Default true
    float alpha = 0.3; // nss threshold. Default 0.3
    int k = 3; // a loop must be consistent with 3 previous matches. Default 3
    DLoopDetector::GeometricalCheck geom_check = DLoopDetector::GEOM_DI; // use direct index for geometrical checking. Default GEOM_DI
    int di_levels = 3; // use three direct index levels. Default 0

    mpParams = new OrbLoopDetector::Parameters(height, width, frequency, use_nss, alpha, k, geom_check, di_levels);

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
    //
    // Load the vocabulary to use
    std::cout << "Loading vocabulary..." << std::endl;
    mpVoc = new OrbVocabulary(strVocabularyFile);

    mpDetector = new OrbLoopDetector(*mpVoc, *mpParams);

    mpExtractor = new OrbExtractor(fSettings);

    initialized_ = true;
}

// ---------------------------------------------------------------------------

LoopDetector::~LoopDetector(){
    std::cout << "LoopDetector destructor called." << std::endl;
    if(initialized_)
    {
        delete mpDetector;
        delete mpVoc;
        delete mpExtractor;
        delete mpParams;
    }
}

// ---------------------------------------------------------------------------


void LoopDetector::process(const cv::Mat &image, DLoopDetector::DetectionResult &result) {
    if(!initialized_)
    {
        return;
    }

    // get features
    mpExtractor->operator()(image, mvKeys, mvDescriptors);

    mpDetector->detectLoop(mvKeys, mvDescriptors, result);
    if(result.detection()) {
        //std::cout << result.F << std::endl;
    }
}

// ---------------------------------------------------------------------------

void LoopDetector::saveDatabase(const std::string &strDatabaseFile) {
    if(!initialized_)
    {
        return;
    }

    std::cout << "Saving database to " << strDatabaseFile << "..." << std::endl;
    mpDetector->getDatabase().save(strDatabaseFile);

}

// ---------------------------------------------------------------------------


void LoopDetector::loadDatabase(const std::string &strDatabaseFile) {
    if(!initialized_)
    {
        return;
    }

    std::cout << "Loading database from " << strDatabaseFile << ". This may take a while..." << std::endl;
    OrbDatabase db;
    db.load(strDatabaseFile);
    mpDetector->setDatabase(db);
}

