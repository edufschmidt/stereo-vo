#ifndef __bfvo_feature_matching_utils_hpp__
#define __bfvo_feature_matching_utils_hpp__

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>

#include <Frame.hpp>

namespace FeatureMatchingUtils{

    std::vector<cv::DMatch> matchWithCrossCheck(cv::Mat *descriptors1, cv::Mat *descriptors2);

}

#endif

