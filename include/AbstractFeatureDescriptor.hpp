#ifndef __bfvo_abstract_feature_descriptor_hpp__
#define __bfvo_abstract_feature_descriptor_hpp__

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <memory>
#include <string>
#include <iostream>

#include <Frame.hpp>

class AbstractFeatureDescriptor {

    public:
        virtual void describeFeatures(std::shared_ptr<Frame> frame)=0;

};

#endif

