#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <memory>
#include <string>
#include <iostream>

#include <Frame.hpp>
#include <AbstractFeatureDetector.hpp>
#include <CornerDetector.hpp>

CornerDetector::CornerDetector(int maxCorners, double qualityLevel, double minDistance, int blockSize, bool useHarrisDetector, double k){
    this->detector = GoodFeaturesToTrackDetector(maxCorners, qualityLevel, minDistance, blockSize, useHarrisDetector, k);
}

void CornerDetector::detectFeatures(std::shared_ptr<Frame> frame){

   detector.detect(*(frame->image), frame->keypointsOnImageCoordinates);
}





