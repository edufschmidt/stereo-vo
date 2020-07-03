#include <CameraCalibrationParameters.hpp>
#include <StereoCalibrationParameters.hpp>
#include <StereoRig.hpp>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <memory>
#include <string>
#include <iostream>

using namespace std;
using namespace cv;

StereoRig::StereoRig(CameraCalibrationParameters *leftParams, CameraCalibrationParameters *rightParams, StereoCalibrationParameters *stereoParams){

    this->R1 = new cv::Mat();
    this->P1 = new cv::Mat();
    this->R2 = new cv::Mat();
    this->P2 = new cv::Mat();
    this->Q = new cv::Mat();
    this->stereoDisparity = std::make_shared<cv::Mat>();

    Size frameSize(leftParams->width, leftParams->height);

    stereoRectify(*(leftParams->cameraMatrix), *(leftParams->distortionCoeffs),
                            *(rightParams->cameraMatrix), *(rightParams->distortionCoeffs),
                                            frameSize,
                                               *(stereoParams->R), *(stereoParams->T),
                                                     *(this->R1), *(this->R2),
                                                            *(this->P1), *(this->P2),
                                                                    *(this->Q),
                                                                        CALIB_ZERO_DISPARITY,
                                                                                0,
                                                                                frameSize,
                                                                                    0, 0);
}

void StereoRig::loadRectifiedStereoPair(std::shared_ptr<Frame> leftRectified, std::shared_ptr<Frame> rightRectified){
    this->leftRectifiedFrame  = leftRectified;
    this->rightRectifiedFrame = rightRectified;
}


void StereoRig::computeDisparitySGBM(){

    int nrows = this->leftRectifiedFrame->image->rows;
	int ncols = this->leftRectifiedFrame->image->cols;

	StereoSGBM sgbm;
	sgbm.SADWindowSize = 5;
	sgbm.numberOfDisparities = 192;
	sgbm.preFilterCap = 4;
	sgbm.minDisparity = 0;
	sgbm.uniquenessRatio = 1;
	sgbm.speckleWindowSize = 150;
	sgbm.speckleRange = 2;
	sgbm.disp12MaxDiff = 10;
	sgbm.fullDP = false;
	sgbm.P1 = 600;
	sgbm.P2 = 2400;

	cv::Mat tempDisparity;
	sgbm(*(this->leftRectifiedFrame->image), *(this->rightRectifiedFrame->image), tempDisparity);
	tempDisparity.convertTo(*(this->stereoDisparity), CV_32F, 1./16);

    return;
}

void StereoRig::computeDisparityBM(){

    int nrows = this->leftRectifiedFrame->image->rows;
	int ncols = this->leftRectifiedFrame->image->cols;

	StereoBM sbm(StereoBM::BASIC_PRESET,16,11);

	sbm.state->SADWindowSize = 11;
	sbm.state->numberOfDisparities = 32;
	sbm.state->preFilterSize = 5;
	sbm.state->preFilterCap = 61;
	sbm.state->minDisparity = 0;
	sbm.state->textureThreshold = 200;
	sbm.state->uniquenessRatio = 1;
	sbm.state->speckleWindowSize = 0;
	sbm.state->speckleRange = 0;
	sbm.state->disp12MaxDiff = 1;

    sbm(*(this->leftRectifiedFrame->image), *(this->rightRectifiedFrame->image), *(this->stereoDisparity), CV_32F);

    return;
}

cv::Vec3f StereoRig::getWorldCoordinates(double row, double col){
    cv::Mat Q_32F;
	this->Q->convertTo(Q_32F,CV_32F);
	cv::Mat_<float> vec(4,1);
 	cv::Vec3f point3f(-1,-1,-1);

 	vec(0) = col;
	vec(1) = row;
	vec(2) = this->stereoDisparity->at<float>(row,col);

	if(vec(2) == 0) return point3f;
	vec(3)=1;
	vec = Q_32F*vec;
	vec /= vec(3);

	if(vec(3)!=1 || abs(vec(0))>10 || abs(vec(1))>10) return point3f;

	point3f[0] = vec(0);
	point3f[1] = vec(1);
	point3f[2] = vec(2);

    return point3f;
}
