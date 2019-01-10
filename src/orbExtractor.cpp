#include <iostream>
#include "orbExtractor.h"

OrbExtractor::OrbExtractor(const cv::FileStorage& fSettings)
{
    /*
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];
    */

    /*
     * ORB::create(int	nfeatures = 500,    The maximum number of features to retain.
     *           float scaleFactor = 1.2f,  Pyramid decimation ratio, greater than 1. scaleFactor==2 means the classical pyramid
     *           int nlevels = 8,           The number of pyramid levels.
     *           int edgeThreshold = 31,    This is size of the border where the features are not detected. It should roughly match the patchSize parameter.
     *           int firstLevel = 0,        The level of pyramid to put source image to. Previous layers are filled with upscaled source image.
     *           int WTA_K = 2,             The number of points that produce each element of the oriented BRIEF descriptor.
     *           int scoreType = ORB::HARRIS_SCORE, FAST_SCORE is alternative value of the parameter that produces slightly less stable keypoints, but it is a little faster to compute.
     *           int patchSize = 31,        size of the patch used by the oriented BRIEF descriptor.
     *           int fastThreshold = 20
     * )
     */

    //mpOrb = new ORB_SLAM2::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    mpOrb = cv::ORB::create();
}

// ----------------------------------------------------------------------------

OrbExtractor::~OrbExtractor()
{
    //delete mpOrb;
}

// ----------------------------------------------------------------------------

void OrbExtractor::operator() (const cv::Mat &im, std::vector<cv::KeyPoint> &keys, std::vector<DBoW2::FORB::TDescriptor> &vDescriptors) const
{
    cv::Mat mask;
    cv::Mat descriptors;

    //mpOrb->operator()(im, mask, keys, descriptors);
    mpOrb->detectAndCompute(im, mask, keys, descriptors);
    changeStructure(descriptors, vDescriptors);
}

// ----------------------------------------------------------------------------

void changeStructure(const cv::Mat &plain, std::vector<cv::Mat> &out)
{
    out.resize(plain.rows);

    for(int i = 0; i < plain.rows; ++i)
    {
        out[i] = plain.row(i);
    }
}
