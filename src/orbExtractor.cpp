#include <iostream>
#include "orbExtractor.h"
#include "easylogging++.h"

OrbExtractor::OrbExtractor()
{
    mpOrb = cv::ORB::create(nfeatures_, scaleFactor_, nlevels_, edgeThreshold_, firstLevel_, WTA_K_, scoreType_, patchSize_, fastThreshold_);
}

OrbExtractor::~OrbExtractor() = default;

void OrbExtractor::operator() (const cv::Mat &im, std::vector<cv::KeyPoint> &keys, std::vector<DBoW2::FORB::TDescriptor> &vDescriptors) const
{
    cv::Mat mask;
    cv::Mat descriptors;

    //mpOrb->operator()(im, mask, keys, descriptors);
    mpOrb->detectAndCompute(im, mask, keys, descriptors);
    changeStructure(descriptors, vDescriptors);
}

void OrbExtractor::saveSettings(const std::string &settingsFile)
{
    std::fstream f;
    f.open(settingsFile, std::ios_base::app);
    if(f.is_open()) {
        f << "////////////////// OrbExtractor Settings //////////////////" << std::endl;
        f << "nFeatures: " << nfeatures_ << std::endl;
        f << "scaleFactor: " << scaleFactor_ << std::endl;
        f << "nLevels: " << nlevels_ << std::endl;
        f << "edgeThreshold: " << edgeThreshold_ << std::endl;
        f << "firstLevel: " << firstLevel_ << std::endl;
        f << "WTA_K: " << WTA_K_ << std::endl;
        f << "scoreType: ";
        switch (scoreType_)
        {
            case cv::ORB::HARRIS_SCORE:
                f << "HARRIS_SCORE";
                break;
            case cv::ORB::FAST_SCORE:
                f << "FAST_SCORE";
                break;
        }
        f << std::endl;
        f << "patchSize: " << patchSize_ << std::endl;
        f << "fastThreshold: " << fastThreshold_ << std::endl;
        f.close();
    }
    else
    {
        LOG(ERROR) << "OrbExtractor could not open " << settingsFile;
    }
}

void changeStructure(const cv::Mat &plain, std::vector<cv::Mat> &out)
{
    out.resize(plain.rows);

    for(int i = 0; i < plain.rows; ++i)
    {
        out[i] = plain.row(i);
    }
}
