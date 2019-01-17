#include <DLib/include/DUtils/DUtils.h>
#include <DLib/include/DUtilsCV/DUtilsCV.h>
#include <DLib/include/DVision/DVision.h>
#include "loopDetector.h"
#include "easylogging++.h"

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
}

LoopDetector::~LoopDetector(){
    LOG(INFO) << "LoopDetector destructor called.";

    delete mpDetector;
    delete mpVoc;
    delete mpExtractor;
    delete mpParams;
}

void LoopDetector::process(const cv::Mat &image, DLoopDetector::DetectionResult &result)
{
    // get features
    mpExtractor->operator()(image, mvKeys, mvDescriptors);

    mpDetector->detectLoop(mvKeys, mvDescriptors, result);
}

void LoopDetector::saveDatabase(const std::string &strDatabaseFile)
{
    LOG(INFO) << "Saving database to " << strDatabaseFile << "...";
    mpDetector->getDatabase().save(strDatabaseFile);

}

void LoopDetector::loadDatabase(const std::string &strDatabaseFile)
{
    LOG(INFO) << "Loading database from " << strDatabaseFile << ". This may take a while...";
    OrbDatabase db;
    db.load(strDatabaseFile);
    mpDetector->setDatabase(db);
}

void LoopDetector::saveSettings(const std::string& settingsFile)
{  
    mpExtractor->saveSettings(settingsFile);
    std::fstream f;
    f.open(settingsFile, std::ios_base::app);
    if(f.is_open())
    {
        f <<  "////////////////// LoopDetector Settings //////////////////" << std::endl;
        f << "use_nss: " << (use_nss_ ? "true" : "false") << std::endl;
        f << "alpha: " << alpha_ << std::endl;
        f << "k: " << k_ << std::endl;
        f << "geom_check_: ";
        switch (geom_check_)
        {
            case DLoopDetector::GEOM_DI:
                f << "GEOM_DI";
                break;
            case DLoopDetector::GEOM_EXHAUSTIVE:
                f << "GEOM_EXHAUSTIVE";
                break;
            case DLoopDetector::GEOM_FLANN:
                f << "GEOM_FLANN";
                break;
            case DLoopDetector::GEOM_NONE:
                f << "GEOM_NONE";
                break;
        }
        f << std::endl;
        f << "di_levels: " << di_levels_ << std::endl;
        f.close();
    }
    else
    {
        LOG(ERROR) << "LoopDetector could not open" << settingsFile;
    }
}
