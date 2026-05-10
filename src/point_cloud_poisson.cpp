#include "point_cloud_merge.h"
#include "point_cloud_processing.h"

#include <open3d/Open3D.h>

#include <iostream>
#include <tuple>
#include <vector>

int PoissonFromPointCloudFiles(const std::string& output_file,
                               const std::vector<std::string>& input_files) {
    open3d::geometry::PointCloud merged_cloud;

    if (!LoadAndMergePointCloudFiles(input_files, merged_cloud)) {
        return 1;
    }

    if (!merged_cloud.HasNormals()) {
        std::cout << "Estimating normals for Poisson reconstruction..." << std::endl;
        merged_cloud.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));
        merged_cloud.OrientNormalsConsistentTangentPlane(30);
    }

    merged_cloud.NormalizeNormals();

    std::cout << "Running Poisson reconstruction..." << std::endl;
    auto [mesh, densities] =
            open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(
                    merged_cloud, 8, 0.0f, 1.1f, false, -1);

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