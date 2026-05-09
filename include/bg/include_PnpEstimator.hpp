#pragma once

#include <vector>
#include <opencv2/core.hpp>

#include "CameraIntrinsics.hpp"
#include "Matching.hpp"
#include "PoseTypes.hpp"

namespace loc {

struct PnpRansacOptions {
  int iterationsCount = 5000;
  float reprojectionError = 6.0f;
  double confidence = 0.999;
  int minInliers = 20;

  int pnpMethod = 0; // ex: cv::SOLVEPNP_ITERATIVE
};

class PnpEstimator {
public:
  explicit PnpEstimator(PnpRansacOptions opt = {});

  PoseEstimateResult EstimatePoseW2C(
      const std::vector<Correspondence2D3D>& corr,
      const CameraIntrinsics& intr) const;

private:
  PnpRansacOptions opt_;
};

PoseDerived DerivePoseOutputs(const PoseW2C& pose_w2c);

} // namespace loc