#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace loc {

struct ColmapCamera {
  int camera_id = -1;
  std::string model;
  int width = 0;
  int height = 0;
  std::vector<double> params;
};

struct ColmapPoint2D {
  Eigen::Vector2d xy = Eigen::Vector2d::Zero();
  int64_t point3D_id = -1;
};

struct ColmapImage {
  int image_id = -1;
  int camera_id = -1;
  std::string name;

  Eigen::Quaterniond qvec = Eigen::Quaterniond::Identity();
  Eigen::Vector3d tvec = Eigen::Vector3d::Zero();

  std::vector<ColmapPoint2D> points2D;
};

struct ColmapPoint3D {
  int64_t point3D_id = -1;
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
};

struct ColmapReconstruction {
  std::unordered_map<int, ColmapCamera> cameras;
  std::unordered_map<int, ColmapImage> images;
  std::unordered_map<int64_t, ColmapPoint3D> points3D;
};

class ColmapModelLoader {
public:
  struct Options { bool prefer_binary = true; };
  ColmapModelLoader();
  explicit ColmapModelLoader(Options opt);

  ColmapReconstruction LoadFromSparseDir(const std::string& sparse_dir) const;

private:
  Options opt_;
};

} // namespace loc