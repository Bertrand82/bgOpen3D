#include "include_LocalizationPipeline.hpp"

#include <stdexcept>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace loc {

LocalizationPipeline::LocalizationPipeline(LocalizationOptions opt)
    : opt_(opt),
      extractor_(std::make_unique<ColmapStyleSiftExtractor>(opt_.sift)),
      selector_(std::make_unique<HardcodedCandidateSelector>(opt_.candidate)),
      model_loader_(ColmapModelLoader::Options{true}) {}

void LocalizationPipeline::SetFeatureExtractor(std::unique_ptr<IFeatureExtractor> extractor) {
  if (!extractor) {
    throw std::invalid_argument("Feature extractor nul.");
  }
  extractor_ = std::move(extractor);
}

void LocalizationPipeline::SetCandidateSelector(std::unique_ptr<ICandidateSelector> selector) {
  if (!selector) {
    throw std::invalid_argument("Candidate selector nul.");
  }
  selector_ = std::move(selector);
}

LocalizationOutputs LocalizationPipeline::Run(const LocalizationInputs& in) {
  if (in.sparse_dir.empty()) {
    throw std::invalid_argument("sparse_dir vide.");
  }
  if (in.database_path.empty()) {
    throw std::invalid_argument("database_path vide.");
  }
  if (in.image_current_path.empty()) {
    throw std::invalid_argument("image_current_path vide.");
  }

  if (!extractor_) {
    throw std::runtime_error("Feature extractor non initialise.");
  }
  if (!selector_) {
    throw std::runtime_error("Candidate selector non initialise.");
  }

  LocalizationOutputs out;

  const ColmapReconstruction model = model_loader_.LoadFromSparseDir(in.sparse_dir);
  if (model.cameras.empty()) {
    throw std::runtime_error("Aucune camera dans la reconstruction.");
  }

  out.candidate_image_ids = selector_->SelectCandidates(model, in.init_pose.C0_world);
  if (out.candidate_image_ids.empty()) {
    throw std::runtime_error("Aucune image candidate selectionnee.");
  }

  const int reference_image_id = out.candidate_image_ids.front();
  const auto it_ref_image = model.images.find(reference_image_id);
  if (it_ref_image == model.images.end()) {
    throw std::runtime_error("Image candidate de reference introuvable dans la reconstruction.");
  }

  const int reference_camera_id = it_ref_image->second.camera_id;
  const auto it_ref_camera = model.cameras.find(reference_camera_id);
  if (it_ref_camera == model.cameras.end()) {
    throw std::runtime_error("Camera de reference introuvable dans la reconstruction.");
  }
  out.intrinsics = BuildIntrinsicsFromColmap(it_ref_camera->second);

  cv::Mat image = cv::imread(in.image_current_path, cv::IMREAD_UNCHANGED);
  if (image.empty()) {
    throw std::runtime_error("Impossible de lire image_current_path.");
  }

  cv::Mat image_gray;
  if (opt_.force_grayscale) {
    if (image.channels() == 1) {
      image_gray = image;
    } else {
      cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
    }
  } else {
    if (image.type() != CV_8UC1) {
      throw std::runtime_error("image_current doit etre CV_8UC1 quand force_grayscale=false.");
    }
    image_gray = image;
  }

  const ExtractedFeaturesColmapStyle current_features = extractor_->Extract(image_gray);

  ColmapDatabase database(in.database_path);
  DescriptorMatcher matcher(opt_.matcher);
  CorrespondenceBuilder correspondence_builder;

  std::vector<Correspondence2D3D> all_corr;

  for (int candidate_image_id : out.candidate_image_ids) {
    const auto it_img = model.images.find(candidate_image_id);
    if (it_img == model.images.end()) {
      continue;
    }

    const cv::Mat candidate_desc = database.LoadDescriptors(candidate_image_id);
    const std::vector<Match2D2D> matches = matcher.Match(current_features.descriptors, candidate_desc);
    out.total_matches_2d2d += static_cast<int>(matches.size());

    const cv::Mat candidate_kpts = database.LoadKeypoints(candidate_image_id);
    if (candidate_kpts.rows != static_cast<int>(it_img->second.points2D.size())) {
      continue;
    }

    const std::vector<Correspondence2D3D> corr = correspondence_builder.Build2D3D(
        current_features.keypoints, matches, it_img->second, model, candidate_image_id);

    all_corr.insert(all_corr.end(), corr.begin(), corr.end());
  }

  out.total_correspondences_2d3d = static_cast<int>(all_corr.size());

  PnpEstimator pnp(opt_.pnp);
  out.estimate = pnp.EstimatePoseW2C(all_corr, out.intrinsics);
  return out;
}

}  // namespace loc
