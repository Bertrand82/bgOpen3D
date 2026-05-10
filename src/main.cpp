#include <open3d/Open3D.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0]
                  << " output.ply input1.ply input2.ply [input3.ply ...]" << std::endl;
        return 1;
    }

    const std::string output_file = argv[1];

    open3d::geometry::PointCloud merged_cloud;
    bool has_colors = false;
    bool has_normals = false;

    for (int i = 2; i < argc; ++i) {
        const std::string input_file = argv[i];
        open3d::geometry::PointCloud cloud;

        std::cout << "Loading: " << input_file << std::endl;

        if (!open3d::io::ReadPointCloud(input_file, cloud)) {
            std::cerr << "Error: failed to read point cloud: " << input_file << std::endl;
            return 1;
        }

        if (cloud.IsEmpty()) {
            std::cout << "Warning: point cloud is empty: " << input_file << std::endl;
        }

        if (cloud.HasColors()) {
            has_colors = true;
        }
        if (cloud.HasNormals()) {
            has_normals = true;
        }

        merged_cloud += cloud;

        std::cout << "  Points loaded: " << cloud.points_.size() << std::endl;
    }

    std::cout << "Merged point count: " << merged_cloud.points_.size() << std::endl;

    if (!open3d::io::WritePointCloud(output_file, merged_cloud)) {
        std::cerr << "Error: failed to write merged point cloud: " << output_file << std::endl;
        return 1;
    }

    std::cout << "Merged point cloud written to: " << output_file << std::endl;

    return 0;
}