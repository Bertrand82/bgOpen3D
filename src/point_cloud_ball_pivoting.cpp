#include "point_cloud_merge.h"
#include "point_cloud_processing.h"

#include <open3d/Open3D.h>

#include <iostream>
#include <limits>
#include <vector>

int BallPivotingFromPointCloudFiles(const std::string& output_file,
                                    const std::vector<std::string>& input_files,
                                    double radius) {
    open3d::geometry::PointCloud merged_cloud;

    if (radius <= 0.0 || radius >= std::numeric_limits<double>::max()) {
        std::cerr << "Error: Ball Pivoting radius must be a positive finite value."
                  << std::endl;
        return 1;
    }

    if (!LoadAndMergePointCloudFiles(input_files, merged_cloud)) {
        return 1;
    }

    if (!merged_cloud.HasNormals()) {
        std::cout << "Estimating normals for Ball Pivoting..." << std::endl;
        merged_cloud.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));
        merged_cloud.OrientNormalsConsistentTangentPlane(30);
    }

    merged_cloud.NormalizeNormals();

    const std::vector<double> radii = {radius, radius * 2.0, radius * 4.0, radius * 8.0};

    std::cout << "Running Ball Pivoting reconstruction..." << std::endl;
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh =
            open3d::geometry::TriangleMesh::CreateFromPointCloudBallPivoting(
                    merged_cloud, radii);

    if (!mesh || mesh->IsEmpty()) {
        std::cerr << "Error: Ball Pivoting reconstruction produced an empty mesh."
                  << std::endl;
        return 1;
    }

    mesh->ComputeVertexNormals();

    if (!open3d::io::WriteTriangleMesh(output_file, *mesh)) {
        std::cerr << "Error: failed to write Ball Pivoting mesh: " << output_file
                  << std::endl;
        return 1;
    }

    std::cout << "Ball Pivoting mesh written to: " << output_file << std::endl;
    return 0;
}