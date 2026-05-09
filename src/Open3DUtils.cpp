#include "include_Open3DUtils.hpp"

#include <stdexcept>

#include <open3d/io/PointCloudIO.h>

namespace loc {

std::shared_ptr<open3d::geometry::PointCloud>
ColmapToOpen3DPointCloud(const ColmapReconstruction& recon) {
  auto cloud = std::make_shared<open3d::geometry::PointCloud>();
  cloud->points_.reserve(recon.points3D.size());
  for (const auto& kv : recon.points3D) {
    cloud->points_.push_back(kv.second.xyz);
  }
  return cloud;
}

void SavePointCloud(const open3d::geometry::PointCloud& cloud,
                    const std::string& path) {
  if (!open3d::io::WritePointCloud(path, cloud)) {
    throw std::runtime_error(
        "Impossible de sauvegarder le nuage de points vers: " + path);
  }
}

std::shared_ptr<open3d::geometry::PointCloud>
LoadPointCloud(const std::string& path) {
  auto cloud = std::make_shared<open3d::geometry::PointCloud>();
  if (!open3d::io::ReadPointCloud(path, *cloud)) {
    throw std::runtime_error(
        "Impossible de charger le nuage de points depuis: " + path);
  }
  return cloud;
}

}  // namespace loc
