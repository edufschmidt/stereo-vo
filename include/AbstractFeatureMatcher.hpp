#ifndef __bfvo_abstract_feature_descriptor_hpp__
#define __bfvo_abstract_feature_descriptor_hpp__

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <memory>
#include <string>
#include <iostream>

#include <Frame.hpp>

class AbstractFeatureMatcher {

    public:
        virtual std::vector<DMatch> matchFeatures(Frame *frame1, Frame *frame2)=0;

};

#endif


