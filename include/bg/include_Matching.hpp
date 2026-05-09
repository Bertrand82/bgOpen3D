#pragma once

#include <cstdint>
#include <vector>
#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "ColmapModel.hpp"
#include "FeatureTypes.hpp"

namespace loc {

struct Match2D2D {
  int idx_current = -1;
  int idx_candidate = -1;
  int distance = 0; // Hamming si BFMatcher(NORM_HAMMING) sur U8, sinon L2 si CV_32F.
};

struct Correspondence2D3D {
  Eigen::Vector2d pt2d_px;
  Eigen::Vector3d pt3d_w;

  int source_image_id = -1;
  int idx_current = -1;
  int idx_candidate = -1;
  int64_t point3D_id = -1;
};

struct MatcherOptions {
  float ratio_test = 0.8f;
  bool cross_check = false;
};

class DescriptorMatcher {
public:
  explicit DescriptorMatcher(MatcherOptions opt = {});

  std::vector<Match2D2D> Match(const DescriptorsU8& desc_current,
                               const DescriptorsU8& desc_candidate) const;

private:
  MatcherOptions opt_;
};

class CorrespondenceBuilder {
public:
  // keypoints_current: Nx4 float; on utilise (x,y) pour la 2D
  std::vector<Correspondence2D3D> Build2D3D(
      const KeypointsF32& keypoints_current,
      const std::vector<Match2D2D>& matches,
      const ColmapImage& candidate_image,
      const ColmapReconstruction& model,
      int candidate_image_id) const;
};

} // namespace loc