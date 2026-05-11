#include "point_cloud_merge.h"

#include <open3d/Open3D.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>

namespace {

struct ExactPointKey {
    std::uint64_t x_bits;
    std::uint64_t y_bits;
    std::uint64_t z_bits;

    bool operator==(const ExactPointKey& other) const {
        return x_bits == other.x_bits && y_bits == other.y_bits && z_bits == other.z_bits;
    }
};

struct ExactPointKeyHash {
    std::size_t operator()(const ExactPointKey& k) const {
        std::size_t h = 1469598103934665603ULL;
        h ^= k.x_bits;
        h *= 1099511628211ULL;
        h ^= k.y_bits;
        h *= 1099511628211ULL;
        h ^= k.z_bits;
        h *= 1099511628211ULL;
        return h;
    }
};

struct QuantizedPointKey {
    std::int64_t x;
    std::int64_t y;
    std::int64_t z;

    bool operator==(const QuantizedPointKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct QuantizedPointKeyHash {
    std::size_t operator()(const QuantizedPointKey& k) const {
        std::size_t h = 1469598103934665603ULL;
        h ^= static_cast<std::uint64_t>(k.x);
        h *= 1099511628211ULL;
        h ^= static_cast<std::uint64_t>(k.y);
        h *= 1099511628211ULL;
        h ^= static_cast<std::uint64_t>(k.z);
        h *= 1099511628211ULL;
        return h;
    }
};

std::uint64_t BitsFromDouble(double value) {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(double));
    return bits;
}

bool IsFinite3(const Eigen::Vector3d& v) {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

void RemoveNonFinitePointsInPlace(open3d::geometry::PointCloud& cloud) {
    const bool has_colors = cloud.HasColors();
    const bool has_normals = cloud.HasNormals();

    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> colors;
    std::vector<Eigen::Vector3d> normals;

    points.reserve(cloud.points_.size());
    if (has_colors) {
        colors.reserve(cloud.colors_.size());
    }
    if (has_normals) {
        normals.reserve(cloud.normals_.size());
    }

    for (std::size_t i = 0; i < cloud.points_.size(); ++i) {
        const Eigen::Vector3d& p = cloud.points_[i];
        if (!IsFinite3(p)) {
            continue;
        }

        points.push_back(p);
        if (has_colors) {
            colors.push_back(cloud.colors_[i]);
        }
        if (has_normals) {
            normals.push_back(cloud.normals_[i]);
        }
    }

    cloud.points_ = std::move(points);
    if (has_colors) {
        cloud.colors_ = std::move(colors);
    }
    if (has_normals) {
        cloud.normals_ = std::move(normals);
    }
}

std::size_t DeduplicatePointsInPlace(open3d::geometry::PointCloud& cloud, double epsilon) {
    const bool has_colors = cloud.HasColors();
    const bool has_normals = cloud.HasNormals();

    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> colors;
    std::vector<Eigen::Vector3d> normals;

    points.reserve(cloud.points_.size());
    if (has_colors) {
        colors.reserve(cloud.colors_.size());
    }
    if (has_normals) {
        normals.reserve(cloud.normals_.size());
    }

    if (epsilon <= 0.0) {
        std::unordered_set<ExactPointKey, ExactPointKeyHash> seen;
        seen.reserve(cloud.points_.size());

        for (std::size_t i = 0; i < cloud.points_.size(); ++i) {
            const Eigen::Vector3d& p = cloud.points_[i];
            ExactPointKey key{BitsFromDouble(p.x()), BitsFromDouble(p.y()), BitsFromDouble(p.z())};
            if (seen.find(key) != seen.end()) {
                continue;
            }
            seen.insert(key);
            points.push_back(p);
            if (has_colors) {
                colors.push_back(cloud.colors_[i]);
            }
            if (has_normals) {
                normals.push_back(cloud.normals_[i]);
            }
        }
    } else {
        std::unordered_set<QuantizedPointKey, QuantizedPointKeyHash> seen;
        seen.reserve(cloud.points_.size());

        for (std::size_t i = 0; i < cloud.points_.size(); ++i) {
            const Eigen::Vector3d& p = cloud.points_[i];
            QuantizedPointKey key{static_cast<std::int64_t>(std::llround(p.x() / epsilon)),
                                  static_cast<std::int64_t>(std::llround(p.y() / epsilon)),
                                  static_cast<std::int64_t>(std::llround(p.z() / epsilon))};
            if (seen.find(key) != seen.end()) {
                continue;
            }
            seen.insert(key);
            points.push_back(p);
            if (has_colors) {
                colors.push_back(cloud.colors_[i]);
            }
            if (has_normals) {
                normals.push_back(cloud.normals_[i]);
            }
        }
    }

    const std::size_t old_size = cloud.points_.size();
    cloud.points_ = std::move(points);
    if (has_colors) {
        cloud.colors_ = std::move(colors);
    }
    if (has_normals) {
        cloud.normals_ = std::move(normals);
    }

    return old_size - cloud.points_.size();
}

}  // namespace

int PostMergePointCloudInPlace(open3d::geometry::PointCloud& cloud,
                               double voxel_size,
                               double dedup_eps,
                               bool skip_dedup) {
    if (voxel_size < 0.0) {
        std::cerr << "Error: --voxel must be >= 0." << std::endl;
        return 1;
    }

    if (dedup_eps < 0.0) {
        std::cerr << "Error: --dedup-eps must be >= 0." << std::endl;
        return 1;
    }

    if (cloud.IsEmpty()) {
        std::cerr << "Error: input point cloud is empty." << std::endl;
        return 1;
    }

    const std::size_t initial_count = cloud.points_.size();

    RemoveNonFinitePointsInPlace(cloud);
    const std::size_t finite_count = cloud.points_.size();
    std::cout << "Removed non-finite points: " << (initial_count - finite_count) << std::endl;

    if (!skip_dedup) {
        const std::size_t removed_duplicates = DeduplicatePointsInPlace(cloud, dedup_eps);
        std::cout << "Removed duplicate points: " << removed_duplicates << std::endl;
    }

    if (voxel_size > 0.0) {
        std::cout << "Applying voxel downsample (voxel_size=" << voxel_size << ")..." << std::endl;
        auto downsampled = cloud.VoxelDownSample(voxel_size);
        if (!downsampled || downsampled->IsEmpty()) {
            std::cerr << "Error: voxel downsample produced an empty cloud." << std::endl;
            return 1;
        }
        cloud = *downsampled;
    }

    std::cout << "Final point count: " << cloud.points_.size() << std::endl;

    return 0;
}

int PostMergePointCloudFile(const std::string& output_file,
                            const std::string& input_file,
                            double voxel_size,
                            double dedup_eps,
                            bool skip_dedup) {
    open3d::geometry::PointCloud cloud;
    std::cout << "Loading point cloud: " << input_file << std::endl;
    if (!open3d::io::ReadPointCloud(input_file, cloud)) {
        std::cerr << "Error: failed to read point cloud: " << input_file << std::endl;
        return 1;
    }

    if (PostMergePointCloudInPlace(cloud, voxel_size, dedup_eps, skip_dedup) != 0) {
        return 1;
    }

    if (!open3d::io::WritePointCloud(output_file, cloud)) {
        std::cerr << "Error: failed to write point cloud: " << output_file << std::endl;
        return 1;
    }

    std::cout << "Post-merge point cloud written to: " << output_file << std::endl;
    return 0;
}
