#include <DLib/include/DUtils/DUtils.h>
#include <DLib/include/DUtilsCV/DUtilsCV.h>
#include <DLib/include/DVision/DVision.h>
#include "loopDetector.h"
#include "easylogging++.h"

LoopDetector::LoopDetector() = default;

LoopDetector::LoopDetector(const std::string &strVocabularyFile, const cv::FileStorage& fSettings) {

    int height = fSettings["Camera.height"];
    int width = fSettings["Camera.width"];
    float frequency = fSettings["Camera.fps"];

    mpParams = new OrbLoopDetector::Parameters(height, width, frequency, use_nss_, alpha_, k_, geom_check_, di_levels_);

    // Load the vocabulary to use
    LOG(INFO) << "Loading vocabulary...";
    mpVoc = new OrbVocabulary(strVocabularyFile);

    mpDetector = new OrbLoopDetector(*mpVoc, *mpParams);

    mpExtractor = new OrbExtractor();

    initialized_ = true;
}

LoopDetector::~LoopDetector(){
    LOG(INFO) << "LoopDetector destructor called.";
    if(initialized_)
    {
        delete mpDetector;
        delete mpVoc;
        delete mpExtractor;
        delete mpParams;
    }
}

void LoopDetector::process(const cv::Mat &image, DLoopDetector::DetectionResult &result) {
    if(!initialized_)
    {
        return;
    }

    // get features
    mpExtractor->operator()(image, mvKeys, mvDescriptors);

    mpDetector->detectLoop(mvKeys, mvDescriptors, result);
}

void LoopDetector::saveDatabase(const std::string &strDatabaseFile) {
    if(!initialized_)
    {
        return;
    }

    LOG(INFO) << "Saving database to " << strDatabaseFile << "...";
    mpDetector->getDatabase().save(strDatabaseFile);

}

void LoopDetector::loadDatabase(const std::string &strDatabaseFile) {
    if(!initialized_)
    {
        return;
    }

    LOG(INFO) << "Loading database from " << strDatabaseFile << ". This may take a while...";
    OrbDatabase db;
    db.load(strDatabaseFile);
    mpDetector->setDatabase(db);
}

