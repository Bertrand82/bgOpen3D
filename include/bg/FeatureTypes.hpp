#pragma once

#include <opencv2/core.hpp>

namespace loc {

using KeypointsF32 = cv::Mat;    // Nx4, CV_32F
using DescriptorsU8 = cv::Mat;   // Nx128, CV_8U

}  // namespace loc
