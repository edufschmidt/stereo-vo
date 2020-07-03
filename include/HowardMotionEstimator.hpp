#ifndef __bfvo_howard_motion_estimator_hpp__
#define __bfvo_howard_motion_estimator_hpp__

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <opencv2/legacy/legacy.hpp>
#include "/usr/local/include/opencv2/contrib/contrib.hpp"

#include <Frame.hpp>
#include <DepthSource.hpp>
#include <AbstractFeatureDetector.hpp>
#include <AbstractFeatureDescriptor.hpp>
#include <AbstractMotionEstimator.hpp>


class HowardMotionEstimator : public AbstractMotionEstimator{
	private:
        cv::Mat *leftRectifiedProjectionMatrix;
        DepthSource *depthSource;
        AbstractFeatureDetector *detector;
        AbstractFeatureDescriptor *descriptor;
        Eigen::Isometry3d lastMotionEstimate;
        std::shared_ptr<Frame> leftFrameA, rightFrameA, leftFrameB, rightFrameB;

        void detectValidFeatures(std::shared_ptr<Frame> leftFrame, DepthSource *depthSource, AbstractFeatureDetector *detector);
        cv::Mat buildConsistencyMatrix(std::vector<cv::DMatch> matches,	double distanceDelta);
        std::vector<DMatch> selectConsistentMatches(std::vector<DMatch> matches, double distanceDelta);
        Eigen::Isometry3d* makeInitialMotionEstimate(std::vector<DMatch> consistentMatches);
        double *refineMotionEstimate(std::vector<DMatch> consistentMatches, double *initialParameters);

    public:
        HowardMotionEstimator(DepthSource *depthSource, cv::Mat *leftRectProjectionMat, AbstractFeatureDetector *detector, AbstractFeatureDescriptor *descriptor);
        void loadNewRectifiedStereoPair(std::shared_ptr<Frame> left, std::shared_ptr<Frame> right);
		void estimateIncrementalMotion() override;
		Eigen::Isometry3d getIncrementalMotionEstimate() override;
};

#endif

