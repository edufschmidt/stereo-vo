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

#include <Frame.hpp>
#include <SIFTDescriptor.hpp>

SIFTDescriptor::SIFTDescriptor(){
    this->featureExtractor = DescriptorExtractor::create("SIFT");
}

void SIFTDescriptor::describeFeatures(std::shared_ptr<Frame> frame){
  Mat tempDescriptors;
  featureExtractor->compute(*(frame->image), frame->keypointsOnImageCoordinates, tempDescriptors);
  *(frame->descriptors) = tempDescriptors.clone();
}

