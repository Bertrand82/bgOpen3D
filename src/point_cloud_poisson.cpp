#include "point_cloud_merge.h"
#include "point_cloud_processing.h"

#include <open3d/Open3D.h>

#include <iostream>
#include <tuple>
#include <vector>

namespace {

int ResolveAutoPoissonDepth(const std::size_t point_count) {
    // Conservative defaults to avoid OOM on large clouds.
    if (point_count >= 6000000) {
        return 6;
    }
    if (point_count >= 2500000) {
        return 7;
    }
    if (point_count >= 800000) {
        return 8;
    }
    return 9;
}

}  // namespace

int PoissonFromPointCloudFiles(const std::string& output_file,
                               const std::vector<std::string>& input_files,
                               int depth,
                               float width,
                               float scale,
                               bool linear_fit,
                               int n_threads,
                               double voxel_size,
                               bool auto_depth) {
    open3d::geometry::PointCloud merged_cloud;

    if (!LoadAndMergePointCloudFiles(input_files, merged_cloud)) {
        return 1;
    }

    if (voxel_size > 0.0) {
        std::cout << "Downsampling point cloud (voxel_size=" << voxel_size << ")..." << std::endl;
        auto downsampled = merged_cloud.VoxelDownSample(voxel_size);
        if (downsampled && !downsampled->IsEmpty()) {
            merged_cloud = *downsampled;
        } else {
            std::cerr << "Error: voxel downsampling produced an empty point cloud." << std::endl;
            return 1;
        }
    }

    if (auto_depth) {
        const int selected_depth = ResolveAutoPoissonDepth(merged_cloud.points_.size());
        std::cout << "Auto depth enabled: point_count=" << merged_cloud.points_.size()
                  << " -> depth=" << selected_depth << std::endl;
        depth = selected_depth;
    }

    if (!merged_cloud.HasNormals()) {
        std::cout << "Estimating normals for Poisson reconstruction..." << std::endl;
        merged_cloud.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));
        merged_cloud.OrientNormalsConsistentTangentPlane(30);
    }

    merged_cloud.NormalizeNormals();

        std::cout << "Running Poisson reconstruction..."
              << " depth=" << depth
              << " width=" << width
              << " scale=" << scale
              << " linear_fit=" << (linear_fit ? "true" : "false")
              << " threads=" << n_threads
              << std::endl;
    auto [mesh, densities] =
            open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(
                merged_cloud, depth, width, scale, linear_fit, n_threads);

    (void)densities;

    if (!mesh || mesh->IsEmpty()) {
        std::cerr << "Error: poisson reconstruction produced an empty mesh." << std::endl;
        return 1;
    }

    mesh->ComputeVertexNormals();

    if (!open3d::io::WriteTriangleMesh(output_file, *mesh)) {
        std::cerr << "Error: failed to write poisson mesh: " << output_file << std::endl;
        return 1;
    }

    std::cout << "Poisson mesh written to: " << output_file << std::endl;
    return 0;
}