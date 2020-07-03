#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <memory>

#include <CameraCalibrationParameters.hpp>
#include <StereoCalibrationParameters.hpp>
#include <RectificationMaps.hpp>
#include <StereoRig.hpp>

using namespace std;
using namespace cv;

int main( int argc, char** argv ) {

	CameraCalibrationParameters *leftCameraParams  = new CameraCalibrationParameters("../test/data/library/calibLeft.xml");
	CameraCalibrationParameters *rightCameraParams = new CameraCalibrationParameters("../test/data/library/calibRight.xml");
	StereoCalibrationParameters *stereoParams      = new StereoCalibrationParameters("../test/data/library/calibStereo.xml");

    StereoRig *stereoRig = new StereoRig(leftCameraParams, rightCameraParams, stereoParams);

    RectificationMaps *leftCameraRectificationMaps  = new RectificationMaps(leftCameraParams, stereoRig->R1);

    cv::Mat *imageFromLeftCamera = new cv::Mat();
    *imageFromLeftCamera = imread("../test/data/library/left/left1.jpg",  CV_LOAD_IMAGE_GRAYSCALE);

    std::shared_ptr<Frame> leftFrame = std::make_shared<Frame>();
    std::shared_ptr<Frame> leftRectifiedFrame = std::make_shared<Frame>();

    leftFrame->loadImage(imageFromLeftCamera);
    leftRectifiedFrame->loadImage(imageFromLeftCamera);

    leftCameraRectificationMaps->rectify(leftRectifiedFrame);

    namedWindow("left_image", 0);
	imshow("left_image", *(leftFrame->image));

    namedWindow("rectified_left_image", 0);
	imshow("rectified_left_image", *(leftRectifiedFrame->image));

	waitKey(0);
	return 0;
}
