#ifndef __bfvo_SIFT_detector_hpp__
#define __bfvo_SIFT_detector_hpp__

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

#include <AbstractFeatureDetector.hpp>
#include <Frame.hpp>

using namespace std;
using namespace cv;

class SIFTDetector : public AbstractFeatureDetector{

    public:
        Ptr<FeatureDetector> detector;
        SIFTDetector();
        void detectFeatures(std::shared_ptr<Frame> frame) override;
};

#endif


