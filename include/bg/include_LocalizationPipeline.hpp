#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "CandidateSelection.hpp"
#include "CameraIntrinsics.hpp"
#include "ColmapDatabase.hpp"
#include "ColmapModel.hpp"
#include "FeatureExtraction.hpp"
#include "Matching.hpp"
#include "PnpEstimator.hpp"
#include "PoseTypes.hpp"

namespace loc {

struct LocalizationInputs {
  std::string sparse_dir;        // ex: ".../sparse/0"
  std::string database_path;     // ".../database.db"
  std::string image_current_path;// fichier image à localiser

  PoseInitYPR init_pose;
};

struct LocalizationOptions {
  CandidateSelectionOptions candidate;
  ColmapSiftOptions sift;
  MatcherOptions matcher;
  PnpRansacOptions pnp;

  bool force_grayscale = true;
};

struct LocalizationOutputs {
  CameraIntrinsics intrinsics;

  std::vector<int> candidate_image_ids;
  int total_matches_2d2d = 0;
  int total_correspondences_2d3d = 0;

  PoseEstimateResult estimate;
};

class LocalizationPipeline {
public:
  explicit LocalizationPipeline(LocalizationOptions opt = {});

  LocalizationOutputs Run(const LocalizationInputs& in);

  void SetFeatureExtractor(std::unique_ptr<IFeatureExtractor> extractor);
  void SetCandidateSelector(std::unique_ptr<ICandidateSelector> selector);

private:
  LocalizationOptions opt_;

  std::unique_ptr<IFeatureExtractor> extractor_;
  std::unique_ptr<ICandidateSelector> selector_;

  ColmapModelLoader model_loader_;
};

} // namespace loc