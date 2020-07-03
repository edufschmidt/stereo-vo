#ifndef __bfvo_camera_parameters_hpp__
#define __bfvo_camera_parameters_hpp__

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <memory>

/*
 * Pinhole camera model parameters, obtained from a calibration procedure.
 * Radial distortion consider distortion model as rd = (1 + k1*ru^2 + k2*ru^4)
 */
class CameraCalibrationParameters {
	public:
		int height;
		int width;
		double cx;
		double cy;
		double fx;
		double fy;
		double k1;
		double k2;

		std::shared_ptr<cv::Mat> cameraMatrix;
		std::shared_ptr<cv::Mat> distortionCoeffs;

		CameraCalibrationParameters(int width, int height, double cx, double cy, double fx, double fy, double k1, double k2);
		CameraCalibrationParameters(std::string calibrationFile);

		void writeToFile(std::string calibrationFile);
		int loadFromFile(std::string calibrationFile);
};

#endif
