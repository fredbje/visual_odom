#ifndef BUCKET_H
#define BUCKET_H

#include "feature.h"

class Bucket
{
public:
    int id;
    unsigned int max_size;

    FeatureSet features;

    Bucket(unsigned int size);
    ~Bucket();

    void add_feature(cv::Point2f, int);
    void get_features(FeatureSet&);

    unsigned long size();
};

#endif
