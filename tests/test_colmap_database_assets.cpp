#include "include_ColmapDatabase.hpp"
#include "include_ColmapModel.hpp"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

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
        std::filesystem::exists(candidate / "sparse/0")) {
      return candidate;
    }
  }

  throw std::runtime_error("Impossible de trouver data/.");
}

}  // namespace

int main() {
  try {
    std::cout << "Test ColmapDatabase assets starting..." << std::endl;
    const std::filesystem::path assets_dir = FindAssetsDir();
    const std::filesystem::path db_path = assets_dir / "database.db";
    const std::filesystem::path sparse_dir = assets_dir / "sparse/0";

    loc::ColmapDatabase database(db_path.string());

    // image_id=1 existe dans data/database.db
    const int image_id = 1;
    const cv::Mat keypoints = database.LoadKeypoints(image_id);
    const cv::Mat descriptors = database.LoadDescriptors(image_id);
    std::cout << "Keypoints: " << keypoints.rows << "x" << keypoints.cols
              << ", type=" << keypoints.type() << std::endl;
    std::cout << "Descriptors: " << descriptors.rows << "x" << descriptors.cols << ", type=" << descriptors.type() << std::endl;  
    if (keypoints.type() != CV_32F) {
      throw std::runtime_error("Type keypoints attendu=CV_32F.");
    }
    if (descriptors.type() != CV_8U) {
      throw std::runtime_error("Type descriptors attendu=CV_8U.");
    }
    if (keypoints.cols != 4) {
      throw std::runtime_error("Nombre de colonnes keypoints attendu=4.");
    }
    if (descriptors.cols != 128) {
      throw std::runtime_error("Nombre de colonnes descriptors attendu=128.");
    }
    if (keypoints.rows <= 0 || descriptors.rows <= 0) {
      throw std::runtime_error("Keypoints/descriptors vides pour image_id=1.");
    }
    if (keypoints.rows != descriptors.rows) {
      throw std::runtime_error("Incoherence: rows(keypoints) != rows(descriptors).");
    }

    loc::ColmapModelLoader loader(loc::ColmapModelLoader::Options{true});
    const loc::ColmapReconstruction recon = loader.LoadFromSparseDir(sparse_dir.string());

    const auto it_image = recon.images.find(image_id);
    if (it_image == recon.images.end()) {
      throw std::runtime_error("Image id 1 absente de la reconstruction.");
    }

    const int num_points2d = static_cast<int>(it_image->second.points2D.size());
    if (num_points2d != keypoints.rows) {
      throw std::runtime_error("Incoherence reconstruction/DB: points2D != keypoints rows.");
    }

    std::cout << "Test ColmapDatabase assets OK: image_id=" << image_id
              << ", keypoints=" << keypoints.rows
              << ", descriptors=" << descriptors.rows << std::endl;
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Test ColmapDatabase assets KO: " << e.what() << std::endl;
    return 1;
  }
}
