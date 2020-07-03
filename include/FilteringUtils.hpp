#ifndef __bfvo_filtering_utils_hpp__
#define __bfvo_filtering_utils_hpp__

#include <Frame.hpp>

namespace FilteringUtils{

    void applyBilateralSmoothing(std::shared_ptr<Frame> frame, int kernelLength);

}

#endif
