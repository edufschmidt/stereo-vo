#ifndef __bfvo_abstract_feature_detector_hpp__
#define __bfvo_abstract_feature_detector_hpp__

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <memory>
#include <string>
#include <iostream>

#include <Frame.hpp>

class AbstractFeatureDetector {

    public:
        virtual void detectFeatures(std::shared_ptr<Frame> frame)=0;

};

#endif
