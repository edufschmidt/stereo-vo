#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <opencv2/legacy/legacy.hpp>
#include "/usr/local/include/opencv2/contrib/contrib.hpp"
#ifndef __bfvo_depth_source_hpp__
#define __bfvo_depth_source_hpp__

using namespace std;
using namespace cv;

class DepthSource {

	public:
		virtual cv::Vec3f getWorldCoordinates(double row, double col) = 0;
};

#endif
