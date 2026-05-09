#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "include_ColmapModel.hpp"

namespace loc {

struct CameraIntrinsics {
  std::string model;
  int width = 0;
  int height = 0;
  cv::Mat K;      // 3x3, CV_64F
  cv::Mat dist;   // Nx1, CV_64F
};

CameraIntrinsics BuildIntrinsicsFromColmap(const ColmapCamera& camera);

cv::Mat UndistortImage(const cv::Mat& image, const CameraIntrinsics& intrinsics);

}  // namespace loc

std::vector<std::string> getImagesFromDir(const std::filesystem::path& dir_path);
