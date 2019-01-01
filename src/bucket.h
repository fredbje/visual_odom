#ifndef BUCKET_H
#define BUCKET_H

#include "featureset.h"

class Bucket
{
public:
    Bucket(unsigned int max_size) : max_size_(max_size) { }

    ~Bucket() = default;

    void add_feature(cv::Point2f point, int age)
    {
        // won't add feature with age > 10;
        int age_threshold = 10;
        if( age < age_threshold )
        {
            // insert any feature before bucket is full
            if( size() < max_size_ )
            {
                features_.points.push_back(point);
                features_.ages.push_back(age);
            }
            else
                // insert feature with old age and remove youngest one
            {
                int age_min = features_.ages[0];
                unsigned int age_min_idx = 0;

                for( unsigned int i = 0; i < size(); i++ )
                {
                    if (age < age_min)
                    {
                        age_min = age;
                        age_min_idx = i;
                    }
                }
                features_.points[age_min_idx] = point;
                features_.ages[age_min_idx] = age;
            }
        }
    }

    void get_features(FeatureSet& current_features)
    {
        current_features.points.insert(current_features.points.end(), features_.points.begin(), features_.points.end());
        current_features.ages.insert(current_features.ages.end(), features_.ages.begin(), features_.ages.end());
    }

    unsigned long size() { return features_.points.size(); }

private:
    unsigned int max_size_;
    FeatureSet features_;
};
#endif
