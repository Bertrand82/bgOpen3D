#include "include_Open3DUtils.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

std::filesystem::path FindSparseDir() {
  const std::vector<std::filesystem::path> candidates = {
      "data/sparse/0",
      "../data/sparse/0",
      "../../data/sparse/0",
      "../../../data/sparse/0",
  };

  for (const auto& candidate : candidates) {
    if (std::filesystem::exists(candidate / "cameras.bin") ||
        std::filesystem::exists(candidate / "cameras.txt")) {
      return candidate;
    }
  }

  throw std::runtime_error(
      "Impossible de trouver data/sparse/0 depuis le repertoire courant.");
}

std::filesystem::path FindTmpDir() {
  const std::filesystem::path tmp = std::filesystem::temp_directory_path();
  return tmp;
}

}  // namespace

int main() {
  try {
    std::cout << "Test Open3DUtils starting..." << std::endl;

    // --- 1. Load COLMAP reconstruction ---
    const std::filesystem::path sparse_dir = FindSparseDir();
    loc::ColmapModelLoader loader(loc::ColmapModelLoader::Options{true});
    const loc::ColmapReconstruction recon =
        loader.LoadFromSparseDir(sparse_dir.string());

    if (recon.points3D.empty()) {
      throw std::runtime_error("Aucun point 3D dans la reconstruction.");
    }
    std::cout << "  Points3D charges: " << recon.points3D.size() << std::endl;

    // --- 2. Convert to Open3D PointCloud ---
    const auto cloud = loc::ColmapToOpen3DPointCloud(recon);
    if (!cloud) {
      throw std::runtime_error("ColmapToOpen3DPointCloud a retourne nullptr.");
    }
    if (cloud->points_.size() != recon.points3D.size()) {
      throw std::runtime_error(
          "Taille du nuage de points incorrecte: attendu=" +
          std::to_string(recon.points3D.size()) +
          ", obtenu=" + std::to_string(cloud->points_.size()));
    }
    std::cout << "  PointCloud Open3D cree avec " << cloud->points_.size()
              << " points." << std::endl;

    // --- 3. Verify at least one point matches ---
    // Pick the first point3D and verify it is present in the cloud
    const auto& first_entry = *recon.points3D.begin();
    const Eigen::Vector3d& expected_xyz = first_entry.second.xyz;
    bool found = false;
    for (const auto& pt : cloud->points_) {
      if ((pt - expected_xyz).norm() < 1e-9) {
        found = true;
        break;
      }
    }
    if (!found) {
      throw std::runtime_error(
          "Le premier point ColmapPoint3D n'a pas ete trouve dans le nuage Open3D.");
    }

    // --- 4. Save and reload the point cloud ---
    const std::filesystem::path ply_path =
        FindTmpDir() / "test_open3d_pointcloud.ply";
    loc::SavePointCloud(*cloud, ply_path.string());
    std::cout << "  Nuage de points sauvegarde vers: " << ply_path << std::endl;

    const auto cloud_loaded = loc::LoadPointCloud(ply_path.string());
    if (!cloud_loaded) {
      throw std::runtime_error("LoadPointCloud a retourne nullptr.");
    }
    if (cloud_loaded->points_.size() != cloud->points_.size()) {
      throw std::runtime_error(
          "Taille du nuage recharge incorrecte: attendu=" +
          std::to_string(cloud->points_.size()) +
          ", obtenu=" + std::to_string(cloud_loaded->points_.size()));
    }
    std::cout << "  Nuage de points recharge avec " << cloud_loaded->points_.size()
              << " points." << std::endl;

    // Cleanup
    std::filesystem::remove(ply_path);

    std::cout << "[PASS] test_open3d_pointcloud" << std::endl;
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "[FAIL] test_open3d_pointcloud: " << e.what() << std::endl;
    return 1;
  }
}
