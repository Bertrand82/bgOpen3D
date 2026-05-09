#pragma once

#include <string>
#include <opencv2/core.hpp>

#include "FeatureTypes.hpp"

namespace loc {

struct ExtractedFeaturesColmapStyle {
  KeypointsF32 keypoints;      // Nx4 float
  DescriptorsU8 descriptors;   // Nx128 uint8
};

struct ColmapSiftOptions {
  // Placeholders : l’impl devra caler les valeurs sur COLMAP (peak_threshold, etc.)
  int max_num_features = 8192;
  bool estimate_affine_shape = false;
  bool domain_size_pooling = false;
};

class IFeatureExtractor {
public:
  virtual ~IFeatureExtractor() = default;

  // image_gray: CV_8U mono
  virtual ExtractedFeaturesColmapStyle Extract(const cv::Mat& image_gray) const = 0;
};

// Impl extraction SIFT compatible “format COLMAP” en sortie (U8).
// (L’impl peut utiliser OpenCV SIFT puis quantifier/convertir en U8 si nécessaire.)
class ColmapStyleSiftExtractor final : public IFeatureExtractor {
public:
  explicit ColmapStyleSiftExtractor(ColmapSiftOptions opt = {});
  ExtractedFeaturesColmapStyle Extract(const cv::Mat& image_gray) const override;

private:
  ColmapSiftOptions opt_;
};

} // namespace loc