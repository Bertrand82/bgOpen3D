#include "include_CameraIntrinsics.hpp"
#include "include_ColmapDatabase.hpp"
#include "include_ColmapModel.hpp"
#include "include_FeatureExtraction.hpp"
#include "include_Matching.hpp"
#include "include_PnpEstimator.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

std::filesystem::path FindAssetsDir() {
  const std::vector<std::filesystem::path> candidates = {
      "data",
      "../data",
      "../../data",
      "../../../data",
  };

  for (const auto& candidate : candidates) {
    if (std::filesystem::exists(candidate / "database.db") &&
        std::filesystem::exists(candidate / "sparse/0") &&
        std::filesystem::exists(candidate / "images")) {
      return candidate;
    }
  }

  throw std::runtime_error("Impossible de trouver data/.");
}

std::filesystem::path FindCurrentImagePath(const std::filesystem::path& assets_dir) {
  const std::vector<std::filesystem::path> candidates = {
      assets_dir / "IMAGE_CURRENT.jpg",
      assets_dir / "IMAGE_CURRENT.JPG",
      assets_dir / "images/IMAGE_CURRENT.jpg",
      assets_dir / "images/IMAGE_CURRENT.JPG",
  };

  for (const auto& p : candidates) {
    if (std::filesystem::exists(p)) {
      return p;
    }
  }

  throw std::runtime_error("Impossible de trouver ./data/IMAGE_CURRENT.jpg (ou variante .JPG).");
}

bool IsFinitePose(const loc::PoseW2C& pose) {
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      if (!std::isfinite(pose.R(r, c))) {
        return false;
      }
    }
    if (!std::isfinite(pose.t(r))) {
      return false;
    }
  }
  return true;
}

std::vector<loc::Match2D2D> BuildMatchesL2Ratio(const cv::Mat& desc_query_u8,
                                                const cv::Mat& desc_candidate_u8,
                                                float ratio_test) {
  if (desc_query_u8.empty() || desc_candidate_u8.empty()) {
    return {};
  }
  if (desc_query_u8.type() != CV_8U || desc_candidate_u8.type() != CV_8U ||
      desc_query_u8.cols != 128 || desc_candidate_u8.cols != 128) {
    throw std::runtime_error("Descripteurs invalides pour BuildMatchesL2Ratio.");
  }

  cv::Mat desc_query_f32;
  cv::Mat desc_candidate_f32;
  desc_query_u8.convertTo(desc_query_f32, CV_32F);
  desc_candidate_u8.convertTo(desc_candidate_f32, CV_32F);

  cv::BFMatcher bf(cv::NORM_L2, false);
  std::vector<std::vector<cv::DMatch>> knn_matches;
  bf.knnMatch(desc_query_f32, desc_candidate_f32, knn_matches, 2);

  std::vector<loc::Match2D2D> out;
  out.reserve(knn_matches.size());
  for (const auto& pair : knn_matches) {
    if (pair.size() < 2U) {
      continue;
    }
    const cv::DMatch& m1 = pair[0];
    const cv::DMatch& m2 = pair[1];
    if (m2.distance <= 0.0F) {
      continue;
    }
    if (m1.distance < ratio_test * m2.distance) {
      out.push_back({m1.queryIdx, m1.trainIdx, static_cast<int>(m1.distance)});
    }
  }

  return out;
}

}  // namespace

int main() {
    using clock = std::chrono::steady_clock; // recommandé pour mesurer une durée
    auto start_time = clock::now();

  try {
    std::cout << "Test PnpEstimator image+db+sparse starting..." << std::endl;

    const std::filesystem::path assets_dir = FindAssetsDir();
    const std::filesystem::path db_path = assets_dir / "database.db";
    const std::filesystem::path sparse_dir = assets_dir / "sparse/0";
    const std::filesystem::path query_image_path = FindCurrentImagePath(assets_dir);

    cv::Mat query_bgr = cv::imread(query_image_path.string(), cv::IMREAD_COLOR);
    if (query_bgr.empty()) {
      throw std::runtime_error("Impossible de lire l'image courante: " + query_image_path.string());
    }
    std::cout << "BG query_image_path: " << query_image_path << std::endl;
    cv::Mat query_gray;
    cv::cvtColor(query_bgr, query_gray, cv::COLOR_BGR2GRAY);

    loc::ColmapStyleSiftExtractor extractor;
    const loc::ExtractedFeaturesColmapStyle query_features = extractor.Extract(query_gray);
    if (query_features.descriptors.empty()) {
      throw std::runtime_error("Descripteurs query vides.");
    }

    loc::ColmapModelLoader loader(loc::ColmapModelLoader::Options{true});
    const loc::ColmapReconstruction recon = loader.LoadFromSparseDir(sparse_dir.string());
    if (recon.images.empty()) {
      throw std::runtime_error("Aucune image dans la reconstruction.");
    }
    std::cout << "Reconstruction chargée: " << recon.images.size() << " images, "
              << recon.points3D.size() << " points 3D." << std::endl;
    loc::ColmapDatabase database(db_path.string());
    loc::CorrespondenceBuilder correspondence_builder;

    loc::PnpRansacOptions pnp_options;
    pnp_options.iterationsCount = 5000;
    pnp_options.reprojectionError = 16.0F;
    pnp_options.confidence = 0.999;
    pnp_options.minInliers = 6;

    loc::PnpEstimator estimator(pnp_options);

    int best_image_id = -1;
    std::vector<loc::Correspondence2D3D> best_corr;
    loc::PoseEstimateResult best_estimate;
    int best_inliers = -1;
    std::size_t best_corr_count_seen = 0U;

    for (const auto& kv : recon.images) {
      const int candidate_image_id = kv.first;
      const auto it_candidate = recon.images.find(candidate_image_id);
      if (it_candidate == recon.images.end()) {
        continue;
      }

      const cv::Mat candidate_desc = database.LoadDescriptors(candidate_image_id);
      if (candidate_desc.empty()) {
        continue;
      }

        const std::vector<loc::Match2D2D> matches =
          BuildMatchesL2Ratio(query_features.descriptors, candidate_desc, 0.8F);
      if (matches.empty()) {
        continue;
      }

      const std::vector<loc::Correspondence2D3D> corr = correspondence_builder.Build2D3D(
          query_features.keypoints, matches, it_candidate->second, recon, candidate_image_id);

      if (corr.size() > best_corr_count_seen) {
        best_corr_count_seen = corr.size();
      }
      if (corr.size() < 6U) {
        continue;
      }

      const auto it_camera = recon.cameras.find(it_candidate->second.camera_id);
      if (it_camera == recon.cameras.end()) {
        continue;
      }

      const loc::CameraIntrinsics intrinsics = loc::BuildIntrinsicsFromColmap(it_camera->second);
      const loc::PoseEstimateResult estimate = estimator.EstimatePoseW2C(corr, intrinsics);

      if (estimate.success && estimate.num_inliers > best_inliers) {
        best_inliers = estimate.num_inliers;
        best_estimate = estimate;
        best_corr = corr;
        best_image_id = candidate_image_id;
      }
    }

    if (!best_estimate.success) {
      throw std::runtime_error(
          "PnP a echoue sur toutes les candidates. max_corr_2d3d=" +
          std::to_string(best_corr_count_seen));
    }
    if (best_estimate.num_inliers < pnp_options.minInliers) {
      throw std::runtime_error("Nombre d'inliers insuffisant.");
    }
    if (best_estimate.inlier_mask.size() != best_corr.size()) {
      throw std::runtime_error("Taille inlier_mask incoherente.");
    }
    if (!IsFinitePose(best_estimate.pose_w2c)) {
      throw std::runtime_error("Pose estimee contient des NaN/Inf.");
    }
    auto end_time_extraction = clock::now();
    auto duree_extraction = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time_extraction - start_time).count();
    std::cout << "Durée totale (extraction+matching+PnP): " << duree_extraction/1000000 << " millisecondes." << std::endl;

    std::cout << "Test PnpEstimator image+db+sparse OK"
              << " | query_image=" << query_image_path.filename().string()
              << " | best_image_id=" << best_image_id
              << " | corr_2d3d=" << best_corr.size()
              << " | inliers=" << best_estimate.num_inliers << std::endl;
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Test PnpEstimator image+db+sparse KO: " << e.what() << std::endl;
    return 1;
  }
}
