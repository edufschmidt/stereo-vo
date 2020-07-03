#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <memory>

#include <CameraCalibrationParameters.hpp>

using namespace std;
using namespace cv;

string pathToCalibrationFile = "/home/eduardo/Desktop/new_vo/bin/calibTestFile.xml";

int main( int argc, char** argv ) {

	CameraCalibrationParameters *calibParams = new CameraCalibrationParameters(320,240,160,120,262,263,0.1,0.2);
	calibParams->writeToFile(pathToCalibrationFile);

    cout << "Calibration parameters successfully saved to " << pathToCalibrationFile << endl;
    cout << "Loading parameters from file... " << endl;

	CameraCalibrationParameters *loadedCalibParams = new CameraCalibrationParameters(pathToCalibrationFile);

	cout << "width: "  << loadedCalibParams->width << "..........";  if(loadedCalibParams->width == 320) cout << "Passed!" << endl;
	cout << "height: " << loadedCalibParams->height << ".........."; if(loadedCalibParams->height == 240) cout << "Passed!" << endl;
	cout << "cx: "  << loadedCalibParams->cx << "..........";        if(loadedCalibParams->cx == 160) cout << "Passed!" << endl;
	cout << "cy: "  << loadedCalibParams->cy << "..........";        if(loadedCalibParams->cy == 120) cout << "Passed!" << endl;
	cout << "fx: "  << loadedCalibParams->fx << "..........";        if(loadedCalibParams->fx == 262) cout << "Passed!" << endl;
	cout << "fy: "  << loadedCalibParams->fy << "..........";        if(loadedCalibParams->fy == 263) cout << "Passed!" << endl;
	cout << "k1: "  << loadedCalibParams->k1 << "..........";        if(loadedCalibParams->k1 == 0.1) cout << "Passed!" << endl;
	cout << "k2: "  << loadedCalibParams->k2 << "..........";        if(loadedCalibParams->k2 == 0.2) cout << "Passed!" << endl;

    cout <<"Camera matrix = " << endl << *(loadedCalibParams->cameraMatrix) << endl;


	return 0;
}
