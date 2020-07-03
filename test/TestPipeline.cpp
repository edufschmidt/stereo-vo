#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <CameraCalibrationParameters.hpp>
#include <StereoCalibrationParameters.hpp>
#include <RectificationMaps.hpp>

#include <AbstractFeatureDetector.hpp>
#include <AbstractFeatureDescriptor.hpp>

#include <AbstractMotionEstimator.hpp>
#include <HowardMotionEstimator.hpp>

#include <CornerDetector.hpp>
#include <SIFTDetector.hpp>
#include <SIFTDescriptor.hpp>
#include <StereoRig.hpp>
#include <Frame.hpp>

using namespace std;
using namespace cv;

std::string datasetPath = "./data/library/";
const long numberOfFrames = 340;

/*
 * Auxiliary function to load a new pair of images from the disk. This function can be modified to allow for
 * using a different source for the data.
 */
void getNewStereoPair(std::shared_ptr<Frame> leftFrame, std::shared_ptr<Frame> rightFrame, size_t index){

    std::shared_ptr<cv::Mat> imageFromLeftCam  = std::make_shared<cv::Mat>();
    std::shared_ptr<cv::Mat> imageFromRightCam = std::make_shared<cv::Mat>();

    *imageFromLeftCam  = imread("../test/data/library/left/left"   + std::to_string(index) + ".jpg", CV_LOAD_IMAGE_GRAYSCALE);
    *imageFromRightCam = imread("../test/data/library/right/right"  + std::to_string(index) + ".jpg", CV_LOAD_IMAGE_GRAYSCALE);

    leftFrame->image  = imageFromLeftCam;
    rightFrame->image = imageFromRightCam;
}

// Auxiliary function to preprocess the frames.
void preprocessStereoPair(std::shared_ptr<Frame> leftFrame, std::shared_ptr<Frame> rightFrame, RectificationMaps *leftRectMaps, RectificationMaps *rightRectMaps, StereoRig *rig){
    leftRectMaps->rectify(leftFrame);
    rightRectMaps->rectify(rightFrame);
}


void displayDisparity(std::shared_ptr<cv::Mat> disparity){
    double min;
    double max;
    cv::minMaxIdx(*disparity, &min, &max);
    cv::Mat disp8;
    // expand your range to 0..255. Similar to histEq();
    (*disparity).convertTo(disp8, CV_8UC1, 255 / (max-min), -min);
    cv::imshow("Out", disp8);
}



int main( int argc, char** argv ) {

   /*
    * Without this initialization, the SIFT related functions will not work, and you'll get a "Segmentation Fault" error.
    * (it´s an OpenCV requirement, so it's irrelevant when using non-SIFT detectors/descriptors or other SIFT implementations)
    */
	cv::initModule_nonfree();

   /*
    * Load calibration parameters from xml files
    */
	CameraCalibrationParameters *leftCameraParams  = new CameraCalibrationParameters("../test/data/library/calibLeft.xml");
	CameraCalibrationParameters *rightCameraParams = new CameraCalibrationParameters("../test/data/library/calibRight.xml");
	StereoCalibrationParameters *stereoParams      = new StereoCalibrationParameters("../test/data/library/calibStereo.xml");

   /*
    * Create StereoRig object, which will be the DepthSource for our motion estimator.
    * Note that, in order to use a different DepthSource, say a depth image from a RGB-D sensor, all one has to
    * do is to create a new class and inherit from DepthSource. In the case of this particular motion estimator,
    * it's assumed that the frame passed as leftFrame and the depth source are aligned
    * (i.e. depthSource->getWorldCoordinate(row,col) = world coord. of point projected to leftFrame(row,col))
    */
    StereoRig *stereoRig = new StereoRig(leftCameraParams, rightCameraParams, stereoParams);

   /*
    * Create the rectifications maps based on the parameters loaded above. In order to use this struct with
    * single (non-stereo) images, which don't need to be stereo rectified, simply set the second parameter to
    * a 3x3 identity matrix, or use the alternative constructor.
    */
    RectificationMaps *leftCameraRectificationMaps  = new RectificationMaps(leftCameraParams,  stereoRig->R1);
    RectificationMaps *rightCameraRectificationMaps = new RectificationMaps(rightCameraParams, stereoRig->R2);

   /*
    * Create the feature detector and descriptor to be used on feature matching by the
    * motion estimatior. In order to use a different detector/descriptor,
    * simply create a new class with the custom algorithm, such that it inherits from
    * AbstractFeatureDetector/AbstractFeatureDesciptor, overriding the abstract methods.
    * Once that's done, the new detectors/descriptors can be passed directly to the
    * motion estimator, as shown below.
    */
    CornerDetector *detector = new CornerDetector(1000, 0.0005, 2, 3, false, 0.04);
    //SIFTDetector *detector = new SIFTDetector();
    SIFTDescriptor *descriptor = new SIFTDescriptor();

    HowardMotionEstimator *motionEstimator = new HowardMotionEstimator(stereoRig, stereoRig->P1, detector, descriptor);

   /*
    * Create pointers for the frames to be loaded
    */
    std::shared_ptr<Frame> leftFrame;
    std::shared_ptr<Frame> rightFrame;

   /*
    * Create object to store the absolute pose with respect to the initial position and orientation
    */
    Eigen::Isometry3d absoluteMotion = Eigen::Isometry3d();
	absoluteMotion.setIdentity();

    double elapsed_time = 0;
    double tick0;

    for(size_t i=1; i<numberOfFrames; i++){

       /*
        * Create fresh new pointers for the frames
        * (this is an important step, since the shared_ptr objects store the reference counter to the frames,
        * which allows for automatic memory management)
        */
        leftFrame  = std::make_shared<Frame>();
        rightFrame = std::make_shared<Frame>();

       /*
        * Load a new pair of frames (change this function to use frames coming out from a different source)
        */
        tick0  = (double)cv::getTickCount();

        getNewStereoPair(leftFrame, rightFrame, i);


        elapsed_time =  1/((cv::getTickCount() - tick0) / cv::getTickFrequency());
        cout << "time spent loading frames: "<< 1/elapsed_time*1000 << " ms"<< endl;

       /*
        * Preprocess frames to remove distortion and rectify them using the stereo parameters loaded above
        */
        tick0 = (double)cv::getTickCount();

        preprocessStereoPair(leftFrame, rightFrame, leftCameraRectificationMaps, rightCameraRectificationMaps, stereoRig);

        elapsed_time =  1/((cv::getTickCount() - tick0) / cv::getTickFrequency());
        cout << "time spent on preprocessment: "<< 1/elapsed_time*1000 << " ms"<< endl;

       /*
        * Load the frames into the StereoRig object and compute a disparity image
        * This might look a bit ugly at first, but has to be done here to abstract the type of DepthSource used by the MotionEstimator.
        * (If there's a better way to do it, hiding this step from the programmer, I'm open to suggestions)
        */
        tick0  = (double)cv::getTickCount();

        stereoRig->loadRectifiedStereoPair(leftFrame, rightFrame);
        // stereoRig->computeDisparitySGBM();
        stereoRig->computeDisparityBM();

        elapsed_time =  1/((cv::getTickCount() - tick0) / cv::getTickFrequency());
        cout << "time spent on disparity computation: "<< 1/elapsed_time*1000 << " ms"<< endl;


        // Display the disparity image computed for the current stereo pair
        // displayDisparity(stereoRig->stereoDisparity);
        // waitKey(0);

       /*
        * Load the rectified frames into the motion estimator and invoke the motion estimation method
        */
        motionEstimator->loadNewRectifiedStereoPair(leftFrame, rightFrame);
        motionEstimator->estimateIncrementalMotion();

       /*
        * Collect the result of the latest motion estimation, and use it to update the absolute pose.
        * In order to perform bundle adjustment, one could store the incrementalMotion objects in a vector,
        * and feed this vector into the BA procedure later.
        */
        Eigen::Isometry3d incrementalMotion = motionEstimator->getIncrementalMotionEstimate();
        absoluteMotion = absoluteMotion*incrementalMotion;

       /*
        * Get the absolute translation and display it on the console. For recording purposes, pipe the output to
        * a text file.
        */
		Eigen::Vector3d absoluteTranslation = absoluteMotion.translation();
        cout << absoluteTranslation[0] << " "<< absoluteTranslation[1] << " "<< absoluteTranslation[2] << endl;

       /*
        * Note that, thanks to the C++11's shared_ptrs, we don't have to explicitly free memory, at the cost of some
        * overhead when passing these cool pointers back and forth as arguments.
        */
    }
}
