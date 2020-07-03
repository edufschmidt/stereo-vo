## stereo-vo

**Disclaimer**: *This implementation dates back from 2014 and has not been updated since. The project was recently moved to Github from a private repo simply to avoid losing it. A Dockerfile will be provided to facilitate eventual builds while I don't update, or reimplement the whole project with more recent versions of C++, OpenCV, Eigen, etc.*

Stereo visual odometry implementation based on Andrew Howard's paper [Real-time stereo visual odometry for autonomous ground vehicles](https://www-robotics.jpl.nasa.gov/publications/Andrew_Howard/howard_iros08_visodom.pdf). 

Originally intended for utilization by the [BlindFind project](http://brown.technologypublisher.com/technology/18485).

### Overview

Howard's paper make use of dense stereo, and does not rely on any statistical method such as RANSAC to remove outliers, as most of other reported algorithms did at the time of implementation. Instead, it relies on an inlier detection step (described by Hirschmuller in the paper "Fast, Unconstrained Camera Motion Estimation from Stereo without Tracking and Robust Statistics"), which is based on selecting all pairs of matches (between two consecutive frames), amongnst all possible pairs, that are mutually consistent according to a rigidity constraint, and then picking the largest set of mutually consistent matches. Also, no assumptions are made about the camera motion.

Motion is estimated by minimizing the bidirectional reprojection error (thus the error in image coordinates) between two consecutive frames using first an initial estimation obtained with Umeyama's method (due to its availability whithin the Eigen library) and then refining this estimate using the Levenberg-Marquardt algorithm.

The current version also allows for extension of the functionalities, for example, one can easily implement/adapt other detectors and descriptors, or even implement their own algorithm starting from the abstract classes defined by the project, possibly using some of OpenCV's functions nicely wrapped into easy-to-use classes and namespaces.

### Build Instructions

First make sure you have the following libraries installed;
* OpenCV
* GNU Scientific Library (GSL): Can be installed with `apt install libgsl0-dev`)
* Eigen (can be installed with `apt install libeigen3-dev`)

Then run:

```
cd ./build
cmake ..
make
```

Run any of the available test applications in the `/bin/` directory.


## TODO
- [ ] Documentation
- [ ] Test coverage
- [ ] Refactoring
- [ ] Add validity step, and return a HowardMotionEstimatorOutput object as the output of the motion estimation
  method. This new object should contain:
	- The estimated incremental motion;
    - The number of matches in the most recent A and B frames;
	- The number of inliers;
	- A flag indicating that the estimated motion is valid, considering some evaluations (see paper);

- [ ] Add actual unit tests to the Test*.cpp files;

- [ ] Increase efficiency by getting rid of allocation overhead, unecessary copying, etc;

- [ ] Modify the original algorithm to track KLT features;

- [ ] Apply Kalman filtering to improve quality of motion estimates by adding some prior knowledge from a motion model

- [ ] Combine VO estimates with IMU data;

- [ ] Add other feature detector and descriptor algorithms

- [ ] Add a new stereo disparity algorithm with better performance than the ones currently available in the class StereoRig;

- [ ] Implement bundle adjustment for the estimated trajectory;

- [ ] Take advantage of the GPU for performing feature detection/description and disparity computation;

- [ ] Add VO algorithm that compute both the left and right disparities, tracks features in both frames, and tries to minimize the reprojection error cosidering both sets of features.

- [ ] Use GPU for block matching and descriptor computation/matching.

- [ ] Assess performance on different datasets

### Performance evaluation 

Setup: 2013 i7 processor, 8GB RAM, no GPU acceleration, running on a single core

- Breakdown:
Loading frames: 0.886738 ms
Preprocessing: 2.24132 ms
Disparity computation: 50.5504 ms
Feature detection: 2.82794 ms
Feature description: 17.4632 ms
Feature matching: 33.0699 ms
Inlier computation: 6.01452 ms
Motion estimation: 1 ms to 31.6553 ms
- Frame rate: around 7 fps

