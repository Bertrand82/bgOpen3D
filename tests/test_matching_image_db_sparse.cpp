#include "include_ColmapDatabase.hpp"
#include "include_ColmapModel.hpp"
#include "include_FeatureExtraction.hpp"
#include "include_Matching.hpp"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

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

}  // namespace

int main() {
  try {
    using clock = std::chrono::steady_clock; // recommandé pour mesurer une durée
    auto start_time = clock::now();
    std::cout << "Test Matching image+db+sparse starting..." << std::endl;
    const std::filesystem::path assets_dir = FindAssetsDir();
    const std::filesystem::path db_path = assets_dir / "database.db";
    const std::filesystem::path sparse_dir = assets_dir / "sparse/0";
    const std::filesystem::path query_image_path = FindCurrentImagePath(assets_dir);
    std::cout << "BG query_image_path: " << query_image_path << std::endl;
    cv::Mat query_bgr = cv::imread(query_image_path.string(), cv::IMREAD_COLOR);
    if (query_bgr.empty()) {
      throw std::runtime_error("Impossible de lire l'image courante: " + query_image_path.string());
    }

    cv::Mat query_gray;
    cv::cvtColor(query_bgr, query_gray, cv::COLOR_BGR2GRAY);

    loc::ColmapStyleSiftExtractor extractor;
    const loc::ExtractedFeaturesColmapStyle query_features = extractor.Extract(query_gray);

    if (query_features.keypoints.empty() || query_features.descriptors.empty()) {
      throw std::runtime_error("Features query vides.");
    }
    if (query_features.keypoints.type() != CV_32F || query_features.keypoints.cols != 4) {
      throw std::runtime_error("Format keypoints query invalide (attendu Nx4 CV_32F).");
    }
    if (query_features.descriptors.type() != CV_8U || query_features.descriptors.cols != 128) {
      throw std::runtime_error("Format descriptors query invalide (attendu Nx128 CV_8U).");
    }
    auto end_time_extraction = clock::now();
    auto duree_extraction = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time_extraction - start_time).count();
    std::cout << "Durée extraction features: " << duree_extraction/1000000 << " millisecondes." << std::endl;
    std::cout << "Query features extraits: " << query_features.keypoints.rows
              << " keypoints, " << query_features.descriptors.rows
              << " descriptors." << std::endl;  
    loc::ColmapModelLoader loader(loc::ColmapModelLoader::Options{true});
    const loc::ColmapReconstruction recon = loader.LoadFromSparseDir(sparse_dir.string());
    if (recon.images.empty()) {
      throw std::runtime_error("Aucune image dans la reconstruction.");
    }

    loc::ColmapDatabase database(db_path.string());
    loc::DescriptorMatcher matcher(loc::MatcherOptions{0.8F, true});

    int best_image_id = -1;
    std::size_t best_num_matches = 0U;
    std::vector<loc::Match2D2D> best_matches;

    int i = 0;
    for (const auto& kv : recon.images) {
      const int candidate_image_id = kv.first;
      const cv::Mat candidate_desc = database.LoadDescriptors(candidate_image_id);
      if (candidate_desc.empty()) {
        continue;
      }

      const std::vector<loc::Match2D2D> matches =
          matcher.Match(query_features.descriptors, candidate_desc);

      if (matches.size() > best_num_matches) {
        best_num_matches = matches.size();
        best_image_id = candidate_image_id;
        best_matches = matches;
      }
      std::cout << i++ << " - Image candidate_id=" << candidate_image_id
                << " | matches_2d2d=" << matches.size() << std::endl;
    }
    auto end_time_matching = clock::now();
    auto duree_matching = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time_matching - end_time_extraction).count();
    std::cout << "Durée matching: " << duree_matching/1000000 << " millisecondes." << std::endl;

    if (best_image_id < 0 || best_matches.empty()) {
      throw std::runtime_error("Aucun match 2D-2D avec les images de la base.");
    }

    const auto it_best_img = recon.images.find(best_image_id);
    if (it_best_img == recon.images.end()) {
      throw std::runtime_error("Meilleure image candidate introuvable dans le modele.");
    }

    // Sanity check: les keypoints DB doivent correspondre aux points2D du modele.
    const cv::Mat best_candidate_kpts = database.LoadKeypoints(best_image_id);
    if (best_candidate_kpts.rows != static_cast<int>(it_best_img->second.points2D.size())) {
      throw std::runtime_error("Incoherence DB/model: nb keypoints != nb points2D pour la meilleure image.");
    }

    loc::CorrespondenceBuilder correspondence_builder;
    const std::vector<loc::Correspondence2D3D> corr = correspondence_builder.Build2D3D(
        query_features.keypoints, best_matches, it_best_img->second, recon, best_image_id);

    if (corr.empty()) {
      throw std::runtime_error("Aucune correspondance 2D-3D construite.");
    }

    std::cout << "Test Matching image+db+sparse OK"
              << " | query_image=" << query_image_path.filename().string()
              << " | best_image_id=" << best_image_id
              << " | matches_2d2d=" << best_matches.size()
              << " | corr_2d3d=" << corr.size() << std::endl;
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Test Matching image+db+sparse KO: " << e.what() << std::endl;
    return 1;
  }
}
