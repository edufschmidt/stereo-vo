
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <memory>

#include <FeatureMatchingUtils.hpp>
#include <Frame.hpp>

using namespace std;
using namespace cv;

namespace FeatureMatchingUtils{

    std::vector<cv::DMatch> matchWithCrossCheck(cv::Mat *descriptors1, cv::Mat *descriptors2){
        // Available distance metrics: BruteForce (it uses L2 ), BruteForce-L1, BruteForce-Hamming, BruteForce-Hamming(2), FlannBased
        // (The L1-norm is the sum of the absolute values of the columns, which is consistent with the feature matching method used in the paper and based on SAD)
        std::vector<cv::DMatch> matches = std::vector<cv::DMatch>();
        Ptr<DescriptorMatcher> descriptorMatcher = DescriptorMatcher::create("BruteForce");
        vector<DMatch> matches12, matches21;
        descriptorMatcher->match( *descriptors1, *descriptors2, matches12);
        descriptorMatcher->match( *descriptors2, *descriptors1, matches21);

        for( size_t i = 0; i < matches12.size(); i++ ){
            DMatch forward = matches12[i];
            DMatch backward = matches21[forward.trainIdx];

            if( backward.trainIdx == forward.queryIdx ){
               matches.push_back( forward );
            }
        }
        return matches;
        }
}

