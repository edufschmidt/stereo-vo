#ifndef __bfvo_abstract_motion_estimator_hpp__
#define __bfvo_abstract_motion_estimator_hpp__

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class AbstractMotionEstimator {
	public:
		virtual void estimateIncrementalMotion() = 0;
		virtual Eigen::Isometry3d getIncrementalMotionEstimate() = 0;
};

#endif
