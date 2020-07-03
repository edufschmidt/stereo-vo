#ifndef __bfvo_stereo_rig_hpp__
#define __bfvo_stereo_rig_hpp__

#include <DepthSource.hpp>
#include <Frame.hpp>

class StereoRig : public DepthSource {

        public:
            cv::Mat *R1, *R2, *P1, *P2, *Q;
            std::shared_ptr<Frame> leftRectifiedFrame, rightRectifiedFrame;
            std::shared_ptr<cv::Mat> stereoDisparity;
            StereoRig(CameraCalibrationParameters *leftParams, CameraCalibrationParameters *rightParams, StereoCalibrationParameters *stereoParams);
            void loadRectifiedStereoPair(std::shared_ptr<Frame> leftRectified, std::shared_ptr<Frame> rightRectified);
            void computeDisparityBM();
            void computeDisparitySGBM();
            cv::Vec3f getWorldCoordinates(double row, double col) override;
};

#endif
