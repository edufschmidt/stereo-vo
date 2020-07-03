SET(PROJECT_NAME blindfind_viso)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8 FATAL_ERROR)
SET(CMAKE_VERBOSE_MAKEFILE ON)
SET(BUILD_TEST TRUE)

# Build type and flags
SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_CXX_FLAGS "-Wall -Werror -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O2 -mmmx -msse -msse -msse2 -msse3 -mssse3 -fomit-frame-pointer")

# Set definitions
ADD_DEFINITIONS(-std=c++0x -Wall -msse2 -msse3)

# Add plain cmake packages
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(fast REQUIRED)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${fast_INCLUDE_DIRS}
)

# Set link libraries
LIST(APPEND LINK_LIBS
  ${Eigen_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
  gsl
  gslcblas
  ${fast_LIBRARIES}
)

# Set sourcefiles
LIST(APPEND SOURCEFILES
  src/CameraCalibrationParameters.cpp
  src/StereoCalibrationParameters.cpp
  src/RectificationMaps.cpp
  src/StereoRig.cpp
  src/FilteringUtils.cpp
  src/CornerDetector.cpp
  src/SIFTDescriptor.cpp
  src/SIFTDetector.cpp
  src/Frame.cpp
  src/HowardMotionEstimator.cpp
  src/FeatureMatchingUtils.cpp
  src/BidirectionalReprojectionErrorOptUtils.cpp
)

# Create svo library
ADD_LIBRARY(bfviso SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(bfviso ${LINK_LIBS})

# ---------- Configure examples ---------------

ADD_EXECUTABLE(TestFeatures test/TestFeatures.cpp)
TARGET_LINK_LIBRARIES(TestFeatures bfviso)

ADD_EXECUTABLE(TestFiltering test/TestFiltering.cpp)
TARGET_LINK_LIBRARIES(TestFiltering bfviso)

ADD_EXECUTABLE(TestDisparity test/TestDisparity.cpp)
TARGET_LINK_LIBRARIES(TestDisparity bfviso)

ADD_EXECUTABLE(TestPipeline test/TestPipeline.cpp)
TARGET_LINK_LIBRARIES(TestPipeline bfviso)

ADD_EXECUTABLE(TestRectification test/TestRectification.cpp)
TARGET_LINK_LIBRARIES(TestRectification bfviso)

ADD_EXECUTABLE(TestCalibrationData test/TestCalibrationData.cpp)
TARGET_LINK_LIBRARIES(TestCalibrationData bfviso)

ADD_EXECUTABLE(TestStereoCalibrationData test/TestStereoCalibrationData.cpp)
TARGET_LINK_LIBRARIES(TestStereoCalibrationData bfviso)

ADD_EXECUTABLE(TestLoadFrame test/TestLoadFrame.cpp)
TARGET_LINK_LIBRARIES(TestLoadFrame bfviso)
