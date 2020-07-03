#include <RectificationMaps.hpp>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <memory>
#include <string>
#include <iostream>

using namespace std;
using namespace cv;



RectificationMaps::RectificationMaps(CameraCalibrationParameters *cameraParams){

    this->mapX = std::make_shared<cv::Mat>();
    this->mapY = std::make_shared<cv::Mat>();
    this->rectifiedCameraMatrix = std::make_shared<cv::Mat>();

   	Size frameSize(cameraParams->width, cameraParams->height);

    cv::Mat R = cv::Mat();

	initUndistortRectifyMap(*(cameraParams->cameraMatrix),
                                *(cameraParams->distortionCoeffs),
                                        R, *(this->rectifiedCameraMatrix),
                                            frameSize,
                                                CV_32FC1,
                                                    *(this->mapX),
                                                        *(this->mapY));
}


RectificationMaps::RectificationMaps(CameraCalibrationParameters *cameraParams, cv::Mat *R){

    this->mapX = std::make_shared<cv::Mat>();
    this->mapY = std::make_shared<cv::Mat>();
    this->rectifiedCameraMatrix = std::make_shared<cv::Mat>();

   	Size frameSize(cameraParams->width, cameraParams->height);

	initUndistortRectifyMap(*(cameraParams->cameraMatrix),
                                *(cameraParams->distortionCoeffs),
                                        *R, *(this->rectifiedCameraMatrix),
                                            frameSize,
                                                CV_32FC1,
                                                    *(this->mapX),
                                                        *(this->mapY));
}

void RectificationMaps::rectify(std::shared_ptr<Frame> frameToRectify){

    int nrows = frameToRectify->image->rows;
	int ncols = frameToRectify->image->cols;

	cv::remap(*(frameToRectify->image),
					*(frameToRectify->image),
						*(this->mapX),
							*(this->mapY),
								 CV_INTER_CUBIC,
									 BORDER_CONSTANT,
										 0);
    return;
}
