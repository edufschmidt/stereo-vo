#include <CameraCalibrationParameters.hpp>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <memory>
#include <string>
#include <iostream>

using namespace std;
using namespace cv;

CameraCalibrationParameters::CameraCalibrationParameters(int width, int height, double cx, double cy, double fx, double fy, double k1, double k2){

	this->height = height;
	this->width = width;

	this->cx = cx;
	this->cy = cy;

	this->fx = fx;
	this->fy = fy;

	this->k1 = k1;
	this->k2 = k2;

	double cameraMatrixData[] = {fx,   0,  cx,
								 0,   fy,  cy,
							     0,    0,   1};

	double distortionCoeffsData[] = {k1, k2, 0, 0};

    this->cameraMatrix     = std::make_shared<cv::Mat>(3,3,CV_64FC1, cameraMatrixData);
    this->distortionCoeffs = std::make_shared<cv::Mat>(4,1, CV_64FC1,distortionCoeffsData);
}


CameraCalibrationParameters::CameraCalibrationParameters(std::string calibrationFile){
    this->cameraMatrix = std::make_shared<cv::Mat>();
    this->distortionCoeffs = std::make_shared<cv::Mat>();
	this->loadFromFile(calibrationFile);
}

void CameraCalibrationParameters::writeToFile(std::string calibrationFile){

	FileStorage fs(calibrationFile, FileStorage::WRITE);

    fs << "height" << this->height;
    fs << "width" << this->width;

    fs << "cx" << this->cx;
    fs << "cy" << this->cy;

    fs << "fx" << this->fx;
    fs << "fy" << this->fy;

    fs << "k1" << this->k1;
    fs << "k2" << this->k2;

    fs.release();
}


int CameraCalibrationParameters::loadFromFile(std::string calibrationFile){

    FileStorage fs;
    fs.open(calibrationFile, FileStorage::READ);

    if (!fs.isOpened()){
        cout << "Failed to open " << calibrationFile << endl;
        return -1;
    }

    this->height = (int) fs["height"];
    this->width  = (int) fs["width"];

    this->cx  = (double) fs["cx"];
    this->cy  = (double) fs["cy"];

    this->fx  = (double) fs["fx"];
    this->fy  = (double) fs["fy"];

    this->k1  = (double) fs["k1"];
    this->k2  = (double) fs["k2"];

	double cameraMatrixData[] = {fx,   0,  cx,
								 0,   fy,  cy,
							     0,    0,   1};

	double distortionCoeffsData[] = {k1, k2, 0, 0};

    cv::Mat tempCameraMatrix     = cv::Mat(3,3,CV_64FC1, cameraMatrixData);
    cv::Mat tempDistortionCoeffs = cv::Mat(4,1, CV_64FC1,distortionCoeffsData);

    *(this->cameraMatrix) = tempCameraMatrix.clone();
    *(this->distortionCoeffs) = tempDistortionCoeffs.clone();

    return 0;
}
