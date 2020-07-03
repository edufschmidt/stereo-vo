#ifndef __bfvo_frame_hpp__
#define __bfvo_frame_hpp__

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <memory>
#include <string>
#include <iostream>


class Frame {
	public:
        std::shared_ptr<cv::Mat> image;
		std::vector<cv::KeyPoint> keypointsOnImageCoordinates;
		std::vector<cv::Vec3f>    keypointsOnWorldCoordinates;
		std::shared_ptr<cv::Mat>  descriptors;

		Frame();
		void loadImage(cv::Mat *image);
};

#endif

