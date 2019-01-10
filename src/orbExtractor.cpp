#include <iostream>
#include "orbExtractor.h"

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

void changeStructure(const cv::Mat &plain, std::vector<cv::Mat> &out)
{
    out.resize(plain.rows);

    for(int i = 0; i < plain.rows; ++i)
    {
        out[i] = plain.row(i);
    }
}
