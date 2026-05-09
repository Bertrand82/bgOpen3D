#pragma once

#include <vector>
#include <cstdint>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace loc {

// Pose monde -> caméra: Xc = R * Xw + t
struct PoseW2C {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
};

struct PoseInitYPR {
  Eigen::Vector3d C0_world = Eigen::Vector3d::Zero();
  double yaw_deg   = 0.0;
  double pitch_deg = 0.0;
  double roll_deg  = 0.0;
};

struct PoseDerived {
  Eigen::Vector3d C_world = Eigen::Vector3d::Zero();         // C = -R^T t
  Eigen::Quaterniond q_w2c = Eigen::Quaterniond::Identity(); // quaternion monde->cam
  double yaw_deg   = 0.0;
  double pitch_deg = 0.0;
  double roll_deg  = 0.0;
};

struct PoseEstimateResult {
  PoseW2C pose_w2c;
  PoseDerived derived;
  std::vector<uint8_t> inlier_mask;
  int num_inliers = 0;
  bool success = false;
};

} // namespace loc