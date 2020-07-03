#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <memory>

#include <CameraCalibrationParameters.hpp>
#include <StereoCalibrationParameters.hpp>
#include <RectificationMaps.hpp>
#include <StereoRig.hpp>
#include <Frame.hpp>

using namespace std;
using namespace cv;

int main( int argc, char** argv ) {

	CameraCalibrationParameters *leftCameraParams  = new CameraCalibrationParameters("../test/data/library/calibLeft.xml");
	CameraCalibrationParameters *rightCameraParams = new CameraCalibrationParameters("../test/data/library/calibRight.xml");
	StereoCalibrationParameters *stereoParams      = new StereoCalibrationParameters("../test/data/library/calibStereo.xml");

    StereoRig *stereoRig = new StereoRig(leftCameraParams, rightCameraParams, stereoParams);

    RectificationMaps *leftCameraRectificationMaps  = new RectificationMaps(leftCameraParams,  stereoRig->R1);
    RectificationMaps *rightCameraRectificationMaps = new RectificationMaps(rightCameraParams, stereoRig->R2);

    cv::Mat *imageFromLeftCamera = new cv::Mat();
    cv::Mat *imageFromRightCamera = new cv::Mat();

    *imageFromLeftCamera = imread("../test/data/library/left/left1.jpg",  CV_LOAD_IMAGE_GRAYSCALE);
    *imageFromRightCamera = imread("../test/data/library/right/right1.jpg", CV_LOAD_IMAGE_GRAYSCALE);

    std::shared_ptr<Frame> leftFrame = std::make_shared<Frame>();
    std::shared_ptr<Frame> rightFrame = std::make_shared<Frame>();
    leftFrame->loadImage(imageFromLeftCamera);
    rightFrame->loadImage(imageFromRightCamera);

    leftCameraRectificationMaps->rectify(leftFrame);
    rightCameraRectificationMaps->rectify(rightFrame);

    stereoRig->loadRectifiedStereoPair(leftFrame, rightFrame);

    stereoRig->computeDisparitySGBM();
/*
 *  Convert the disparity to an 8-bit image, and adjust the scale so it's easier to be visualized
 */
    double min;
    double max;
    cv::minMaxIdx(*(stereoRig->stereoDisparity), &min, &max);
    cv::Mat disp8;
    (*(stereoRig->stereoDisparity)).convertTo(disp8, CV_8UC1, 255 / (max-min), -min);

    namedWindow("disparity", 0);
	imshow("disparity", disp8);

	waitKey(0);
	return 0;
}

