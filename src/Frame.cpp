#include <Frame.hpp>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <memory>
#include <string>
#include <iostream>

using namespace std;
using namespace cv;

Frame::Frame(){

    this->image = std::make_shared<cv::Mat>();
	this->keypointsOnImageCoordinates = std::vector<KeyPoint>();
	this->keypointsOnWorldCoordinates = std::vector<Vec3f>();
	this->descriptors = std::make_shared<cv::Mat>();
}
void Frame::loadImage(cv::Mat *image){
    *(this->image) = image->clone();
}
