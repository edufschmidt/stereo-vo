#ifndef __bfvo_rectification_maps_hpp__
#define __bfvo_rectification_maps_hpp__

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <memory>
#include <CameraCalibrationParameters.hpp>
#include <Frame.hpp>

class RectificationMaps {
	public:
		std::shared_ptr<cv::Mat> mapX, mapY;
		std::shared_ptr<cv::Mat> rectifiedCameraMatrix;

   		RectificationMaps(CameraCalibrationParameters *cameraParams);
		RectificationMaps(CameraCalibrationParameters *cameraParams, cv::Mat *R);
		void rectify(std::shared_ptr<Frame> frame);
};

#endif
