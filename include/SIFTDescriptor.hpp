#ifndef __bfvo_sift_descriptor_hpp__
#define __bfvo_sift_descriptor_hpp__

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <opencv2/legacy/legacy.hpp>
#include "/usr/local/include/opencv2/contrib/contrib.hpp"

#include <memory>

#include <string>
#include <iostream>
#include <cmath>
#include <time.h>

#include <AbstractFeatureDescriptor.hpp>
#include <Frame.hpp>

using namespace std;
using namespace cv;

class SIFTDescriptor : public AbstractFeatureDescriptor{

    public:
        Ptr<DescriptorExtractor> featureExtractor;
        SIFTDescriptor();
        void describeFeatures(std::shared_ptr<Frame> frame) override;
};

#endif


