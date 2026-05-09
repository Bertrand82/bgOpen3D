#include "include_PnpEstimator.hpp"

#include <cmath>
#include <stdexcept>

#include <opencv2/calib3d.hpp>

namespace loc {

namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

}  // namespace

PnpEstimator::PnpEstimator(PnpRansacOptions opt) : opt_(opt) {}

PoseEstimateResult PnpEstimator::EstimatePoseW2C(const std::vector<Correspondence2D3D>& corr,
                                                 const CameraIntrinsics& intr) const {
  PoseEstimateResult out;

  if (corr.size() < 4U) {
    return out;
  }
  if (intr.K.empty()) {
    throw std::runtime_error("Intrinsics invalides: K vide.");
  }

  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> image_points;
  object_points.reserve(corr.size());
  image_points.reserve(corr.size());

  for (const Correspondence2D3D& c : corr) {
    object_points.emplace_back(static_cast<float>(c.pt3d_w.x()),
                               static_cast<float>(c.pt3d_w.y()),
                               static_cast<float>(c.pt3d_w.z()));
    image_points.emplace_back(static_cast<float>(c.pt2d_px.x()),
                              static_cast<float>(c.pt2d_px.y()));
  }

  cv::Mat rvec;
  cv::Mat tvec;
  std::vector<int> inliers_idx;

  const int pnp_method = opt_.pnpMethod == 0 ? cv::SOLVEPNP_ITERATIVE : opt_.pnpMethod;

  const bool ok = cv::solvePnPRansac(object_points,
                                     image_points,
                                     intr.K,
                                     intr.dist,
                                     rvec,
                                     tvec,
                                     false,
                                     opt_.iterationsCount,
                                     opt_.reprojectionError,
                                     opt_.confidence,
                                     inliers_idx,
                                     pnp_method);
  if (!ok) {
    return out;
  }

  cv::Mat R_cv;
  cv::Rodrigues(rvec, R_cv);

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out.pose_w2c.R(r, c) = R_cv.at<double>(r, c);
    }
    out.pose_w2c.t(r) = tvec.at<double>(r, 0);
  }

  out.inlier_mask.assign(corr.size(), static_cast<uint8_t>(0));
  for (int idx : inliers_idx) {
    if (idx >= 0 && idx < static_cast<int>(out.inlier_mask.size())) {
      out.inlier_mask[static_cast<std::size_t>(idx)] = static_cast<uint8_t>(1);
    }
  }
  out.num_inliers = static_cast<int>(inliers_idx.size());
  out.success = out.num_inliers >= opt_.minInliers;

  if (out.success) {
    out.derived = DerivePoseOutputs(out.pose_w2c);
  }

  return out;
}

PoseDerived DerivePoseOutputs(const PoseW2C& pose_w2c) {
  PoseDerived out;

  out.C_world = -pose_w2c.R.transpose() * pose_w2c.t;
  out.q_w2c = Eigen::Quaterniond(pose_w2c.R).normalized();

  const double sy = std::sqrt(pose_w2c.R(0, 0) * pose_w2c.R(0, 0) +
                              pose_w2c.R(1, 0) * pose_w2c.R(1, 0));
  const bool singular = sy < 1e-9;

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  if (!singular) {
    roll = std::atan2(pose_w2c.R(2, 1), pose_w2c.R(2, 2));
    pitch = std::atan2(-pose_w2c.R(2, 0), sy);
    yaw = std::atan2(pose_w2c.R(1, 0), pose_w2c.R(0, 0));
  } else {
    roll = std::atan2(-pose_w2c.R(1, 2), pose_w2c.R(1, 1));
    pitch = std::atan2(-pose_w2c.R(2, 0), sy);
    yaw = 0.0;
  }

  out.roll_deg = roll * kRadToDeg;
  out.pitch_deg = pitch * kRadToDeg;
  out.yaw_deg = yaw * kRadToDeg;

  return out;
}

}  // namespace loc
