#include "point_cloud_merge.h"
#include "point_cloud_processing.h"

#include <open3d/Open3D.h>

#include <iostream>
#include <string>
#include <vector>

bool LoadAndMergePointCloudFiles(const std::vector<std::string>& input_files,
                                 open3d::geometry::PointCloud& merged_cloud) {
    std::size_t loaded_files = 0;
    std::size_t skipped_files = 0;

    for (const std::string& input_file : input_files) {
        open3d::geometry::PointCloud cloud;

        std::cout << "Loading: " << input_file << std::endl;

        if (!open3d::io::ReadPointCloud(input_file, cloud)) {
            std::cerr << "Warning: skipping invalid point cloud file: " << input_file
                      << std::endl;
            ++skipped_files;
            continue;
        }

        if (cloud.IsEmpty()) {
            std::cout << "Warning: point cloud is empty: " << input_file << std::endl;
        }

        merged_cloud += cloud;
        ++loaded_files;

        std::cout << "  Points loaded: " << cloud.points_.size() << std::endl;
    }

    if (loaded_files == 0) {
        std::cerr << "Error: no valid point cloud could be loaded." << std::endl;
        return false;
    }

    if (skipped_files > 0) {
        std::cout << "Warning: " << skipped_files
                  << " invalid input file(s) were skipped." << std::endl;
    }

    std::cout << "Merged point count: " << merged_cloud.points_.size() << std::endl;
    return true;
}

int MergePointCloudFiles(const std::string& output_file,
                         const std::vector<std::string>& input_files) {
    open3d::geometry::PointCloud merged_cloud;

    if (!LoadAndMergePointCloudFiles(input_files, merged_cloud)) {
        return 1;
    }

    if (!open3d::io::WritePointCloud(output_file, merged_cloud)) {
        std::cerr << "Error: failed to write merged point cloud: " << output_file << std::endl;
        return 1;
    }

    std::cout << "Merged point cloud written to: " << output_file << std::endl;

    return 0;
}