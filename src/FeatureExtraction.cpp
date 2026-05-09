#include "include_FeatureExtraction.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <opencv2/features2d.hpp>

namespace loc {

ColmapStyleSiftExtractor::ColmapStyleSiftExtractor(ColmapSiftOptions opt) : opt_(opt) {}

ExtractedFeaturesColmapStyle ColmapStyleSiftExtractor::Extract(const cv::Mat& image_gray) const {
  if (image_gray.empty()) {
    throw std::runtime_error("Image grayscale vide pour extraction de features.");
  }
  if (image_gray.type() != CV_8UC1) {
    throw std::runtime_error("L'extracteur attend une image CV_8UC1.");
  }

  cv::Ptr<cv::SIFT> sift = cv::SIFT::create(opt_.max_num_features);
  std::vector<cv::KeyPoint> keypoints_cv;
  cv::Mat descriptors_f32;
  sift->detectAndCompute(image_gray, cv::noArray(), keypoints_cv, descriptors_f32);

  ExtractedFeaturesColmapStyle out;
  out.keypoints = cv::Mat::zeros(static_cast<int>(keypoints_cv.size()), 4, CV_32F);

  for (int i = 0; i < static_cast<int>(keypoints_cv.size()); ++i) {
    const cv::KeyPoint& kp = keypoints_cv[static_cast<std::size_t>(i)];
    out.keypoints.at<float>(i, 0) = kp.pt.x;
    out.keypoints.at<float>(i, 1) = kp.pt.y;
    out.keypoints.at<float>(i, 2) = kp.size;
    const float angle_deg = kp.angle >= 0.0F ? kp.angle : 0.0F;
    out.keypoints.at<float>(i, 3) = angle_deg * static_cast<float>(CV_PI / 180.0);
  }

  if (descriptors_f32.empty()) {
    out.descriptors = cv::Mat(0, 128, CV_8U);
    return out;
  }
  if (descriptors_f32.type() != CV_32F || descriptors_f32.cols != 128) {
    throw std::runtime_error("Descripteurs SIFT inattendus (attendu CV_32F Nx128).");
  }

  out.descriptors = cv::Mat::zeros(descriptors_f32.rows, descriptors_f32.cols, CV_8U);
  for (int r = 0; r < descriptors_f32.rows; ++r) {
    for (int c = 0; c < descriptors_f32.cols; ++c) {
      const float v = descriptors_f32.at<float>(r, c);
      const int q = static_cast<int>(std::lround(std::clamp(v, 0.0F, 255.0F)));
      out.descriptors.at<unsigned char>(r, c) = static_cast<unsigned char>(q);
    }
  }

  return out;
}

}  // namespace loc
