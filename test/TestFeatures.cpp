#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <opencv2/legacy/legacy.hpp>
#include "/usr/local/include/opencv2/contrib/contrib.hpp"

#include <memory>

#include <SIFTDetector.hpp>
#include <SIFTDescriptor.hpp>
#include <Frame.hpp>
#include <FilteringUtils.hpp>

using namespace std;
using namespace cv;

int main( int argc, char** argv ) {

   /*
    * Without this initialization, the SIFT related functions do not work
    */
	cv::initModule_nonfree();

	cv::Mat *image = new cv::Mat();
	*image = imread("../test/data/library/left/left1.jpg",  CV_LOAD_IMAGE_GRAYSCALE);

    std::shared_ptr<Frame> leftFrame = std::make_shared<Frame>();
    leftFrame->loadImage(image);

    int maxCorners = 1000;
	double qualityLevel = 0.0005;
	double minDistance = 2;
	int blockSize = 3;
	bool useHarrisDetector = false;
	double k = 0.04;
    SIFTDetector *detector = new SIFTDetector();

    cout << "Finding keypoints..." << endl;
    detector->detectFeatures(leftFrame);
    cout << leftFrame->keypointsOnImageCoordinates.size() << " SIFT keypoints found. " << endl;

    SIFTDescriptor *descriptor = new SIFTDescriptor();
    descriptor->describeFeatures(leftFrame);
    cout << "Size of the descriptor matrix: " << leftFrame->descriptors->size() << endl;

	waitKey(0);

	return 0;
}


