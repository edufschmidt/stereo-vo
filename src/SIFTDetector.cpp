#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <memory>
#include <string>
#include <iostream>

#include <Frame.hpp>
#include <AbstractFeatureDetector.hpp>
#include <SIFTDetector.hpp>

SIFTDetector::SIFTDetector(){
    this->detector = FeatureDetector::create("SIFT");
}

void SIFTDetector::detectFeatures(std::shared_ptr<Frame> frame){
  detector->detect(*(frame->image), frame->keypointsOnImageCoordinates);
}






