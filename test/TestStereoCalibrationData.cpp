#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <memory>

#include <StereoCalibrationParameters.hpp>

using namespace std;
using namespace cv;

string pathToCalibrationFile = "../bin/calibStereoTestFile.xml";

int main( int argc, char** argv ) {

	double T_data[] = {0.1,0.2,0.3};
	double R_data[] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};

	StereoCalibrationParameters *stereoParams = new StereoCalibrationParameters(T_data, R_data);
	stereoParams->writeToFile(pathToCalibrationFile);
	stereoParams->writeToFile(pathToCalibrationFile);
    cout << "Calibration parameters successfully saved to " << pathToCalibrationFile << endl;

	StereoCalibrationParameters *loadedStereoParams = new StereoCalibrationParameters(pathToCalibrationFile);
    cout << "loaded R = " << endl << *loadedStereoParams->R << endl;
    cout << "loaded T = " << endl << *loadedStereoParams->T << endl;

	return 0;
}
