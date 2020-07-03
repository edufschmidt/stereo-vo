
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <memory>

#include <FilteringUtils.hpp>
#include <CameraCalibrationParameters.hpp>
#include <Frame.hpp>

using namespace std;

namespace FilteringUtils{

    void applyBilateralSmoothing(std::shared_ptr<Frame> frame, int kernelLength){
        cv::Mat temp;
        bilateralFilter(*(frame->image), temp, kernelLength, kernelLength*2, kernelLength/2);
        *(frame->image) = temp.clone();
        return;
    }
}

