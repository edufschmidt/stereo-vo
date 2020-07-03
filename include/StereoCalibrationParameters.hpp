#ifndef __bfvo_stereo_parameters_hpp__
#define __bfvo_stereo_parameters_hpp__

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <memory>

class StereoCalibrationParameters {
	public:
		std::shared_ptr<cv::Mat> T;
		std::shared_ptr<cv::Mat> R;

		StereoCalibrationParameters(double *T_data, double* R_data);
		StereoCalibrationParameters(std::string calibrationFile);

		int loadFromFile(std::string calibrationFile);
		void writeToFile(std::string calibrationFile);
};
#endif
