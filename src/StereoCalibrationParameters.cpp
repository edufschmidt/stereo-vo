#include <StereoCalibrationParameters.hpp>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <memory>
#include <string>
#include <iostream>

using namespace std;
using namespace cv;


StereoCalibrationParameters::StereoCalibrationParameters(double *T_data, double* R_data){
	this->R = std::make_shared<cv::Mat>(3,3, CV_64FC1, R_data);
    this->T = std::make_shared<cv::Mat>(3,1,CV_64FC1, T_data);
}


StereoCalibrationParameters::StereoCalibrationParameters(std::string calibrationFile){
    this->R = std::make_shared<cv::Mat>();
    this->T = std::make_shared<cv::Mat>();
	this->loadFromFile(calibrationFile);
}

void StereoCalibrationParameters::writeToFile(std::string calibrationFile){

	FileStorage fs(calibrationFile, FileStorage::WRITE);

    fs << "TX" << this->T->at<double>(0,0);
    fs << "TY" << this->T->at<double>(1,0);
    fs << "TZ" << this->T->at<double>(2,0);

    fs << "R11" << this->R->at<double>(0,0);
    fs << "R21" << this->R->at<double>(1,0);
    fs << "R31" << this->R->at<double>(2,0);

    fs << "R12" << this->R->at<double>(0,1);
    fs << "R22" << this->R->at<double>(1,1);
    fs << "R32" << this->R->at<double>(2,1);

    fs << "R13" << this->R->at<double>(0,2);
    fs << "R23" << this->R->at<double>(1,2);
    fs << "R33" << this->R->at<double>(2,2);

    fs.release();
}


int StereoCalibrationParameters::loadFromFile(std::string calibrationFile){

    FileStorage fs;
    fs.open(calibrationFile, FileStorage::READ);

    if (!fs.isOpened()){
        cout << "Failed to open " << calibrationFile << endl;
        return -1;
    }

	double R_data[] = {fs["R11"], fs["R12"], fs["R13"], fs["R21"], fs["R22"], fs["R23"], fs["31"], fs["32"], fs["R33"]};
	double T_data[] = {fs["TX"],fs["TY"],fs["TZ"]};

    cv::Mat tempR = cv::Mat(3,3,CV_64FC1, R_data);
    cv::Mat tempT = cv::Mat(3,1, CV_64FC1,T_data);

    *(this->R) = tempR.clone();
    *(this->T) = tempT.clone();

    return 0;
}
