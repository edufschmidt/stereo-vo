#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <memory>

#include <Frame.hpp>
#include <FilteringUtils.hpp>

using namespace std;
using namespace cv;

int main( int argc, char** argv ) {

	cv::Mat *image = new cv::Mat();
	*image = imread("../test/data/library/left/left1.jpg",  CV_LOAD_IMAGE_GRAYSCALE);

    std::shared_ptr<Frame> leftFrame = std::make_shared<Frame>();
    leftFrame->loadImage(image);

   /*
    * Call the filtering function, passing the size of the kernel (which has to be odd)
    */

    FilteringUtils::applyBilateralSmoothing(leftFrame, 21);

    namedWindow("filtered_left_frame", 0);
	imshow("filtered_left_frame", *(leftFrame->image));

	waitKey(0);

	return 0;
}

