#pragma once

#include <memory>
#include <string>

#include <open3d/geometry/PointCloud.h>

#include "ColmapModel.hpp"

namespace loc {

// Converts a ColmapReconstruction's 3D points to an Open3D PointCloud.
// Each ColmapPoint3D entry becomes one point in the resulting cloud.
std::shared_ptr<open3d::geometry::PointCloud>
ColmapToOpen3DPointCloud(const ColmapReconstruction& recon);

// Saves an Open3D PointCloud to a file (PLY, PCD, XYZ, etc.).
// The format is inferred from the file extension.
// Throws std::runtime_error on failure.
void SavePointCloud(const open3d::geometry::PointCloud& cloud,
                    const std::string& path);

// Loads an Open3D PointCloud from a file (PLY, PCD, XYZ, etc.).
// Throws std::runtime_error on failure.
std::shared_ptr<open3d::geometry::PointCloud>
LoadPointCloud(const std::string& path);

}  // namespace loc
