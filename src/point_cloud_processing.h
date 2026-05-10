#pragma once

#include <open3d/Open3D.h>

#include <string>
#include <vector>

bool LoadAndMergePointCloudFiles(const std::vector<std::string>& input_files,
                                 open3d::geometry::PointCloud& merged_cloud);