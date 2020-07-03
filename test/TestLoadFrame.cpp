#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <memory>

#include <Frame.hpp>

using namespace std;
using namespace cv;

int main( int argc, char** argv ) {

	cv::Mat *imageFromLeftCamera = new cv::Mat();
    cv::Mat *imageFromRightCamera = new cv::Mat();

    *imageFromLeftCamera  = imread("../test/data/library/left/left1.jpg",  CV_LOAD_IMAGE_GRAYSCALE);
    *imageFromRightCamera = imread("../test/data/library/right/right1.jpg", CV_LOAD_IMAGE_GRAYSCALE);

    Frame *leftFrame = new Frame();
    Frame *rightFrame = new Frame();

    leftFrame->loadImage(imageFromLeftCamera);
    rightFrame->loadImage(imageFromRightCamera);


    namedWindow("left_image", 0);
	imshow("left_image", *leftFrame->image);

    namedWindow("right_image", 0);
	imshow("right_image", *rightFrame->image);

	waitKey(0);

	return 0;
}
