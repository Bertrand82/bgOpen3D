#include "include_Matching.hpp"

#include <stdexcept>

#include <opencv2/features2d.hpp>

namespace loc {

DescriptorMatcher::DescriptorMatcher(MatcherOptions opt) : opt_(opt) {}

std::vector<Match2D2D> DescriptorMatcher::Match(const DescriptorsU8& desc_current,
                                                const DescriptorsU8& desc_candidate) const {
  std::vector<Match2D2D> out;
  if (desc_current.empty() || desc_candidate.empty()) {
    return out;
  }

  if (desc_current.type() != CV_8U || desc_candidate.type() != CV_8U) {
    throw std::runtime_error("DescriptorMatcher attend des descripteurs CV_8U.");
  }
  if (desc_current.cols != 128 || desc_candidate.cols != 128) {
    throw std::runtime_error("DescriptorMatcher attend des descripteurs Nx128.");
  }

  cv::BFMatcher bf(cv::NORM_HAMMING, opt_.cross_check);

  if (opt_.cross_check) {
    std::vector<cv::DMatch> matches;
    bf.match(desc_current, desc_candidate, matches);
    out.reserve(matches.size());
    for (const cv::DMatch& m : matches) {
      if (m.queryIdx < 0 || m.trainIdx < 0) {
        continue;
      }
      out.push_back({m.queryIdx, m.trainIdx, static_cast<int>(m.distance)});
    }
    return out;
  }

  std::vector<std::vector<cv::DMatch>> knn_matches;
  bf.knnMatch(desc_current, desc_candidate, knn_matches, 2);

  out.reserve(knn_matches.size());
  for (const auto& pair : knn_matches) {
    if (pair.size() < 2U) {
      continue;
    }
    const cv::DMatch& m1 = pair[0];
    const cv::DMatch& m2 = pair[1];
    if (m2.distance <= 0.0F) {
      continue;
    }
    if (m1.distance < opt_.ratio_test * m2.distance) {
      out.push_back({m1.queryIdx, m1.trainIdx, static_cast<int>(m1.distance)});
    }
  }

  return out;
}

std::vector<Correspondence2D3D> CorrespondenceBuilder::Build2D3D(
    const KeypointsF32& keypoints_current,
    const std::vector<Match2D2D>& matches,
    const ColmapImage& candidate_image,
    const ColmapReconstruction& model,
    int candidate_image_id) const {
  std::vector<Correspondence2D3D> out;
  if (keypoints_current.empty() || matches.empty()) {
    return out;
  }
  if (keypoints_current.type() != CV_32F || keypoints_current.cols < 2) {
    throw std::runtime_error("keypoints_current doit etre en CV_32F avec au moins 2 colonnes.");
  }

  out.reserve(matches.size());
  for (const Match2D2D& m : matches) {
    if (m.idx_current < 0 || m.idx_current >= keypoints_current.rows) {
      continue;
    }
    if (m.idx_candidate < 0 ||
        m.idx_candidate >= static_cast<int>(candidate_image.points2D.size())) {
      continue;
    }

    const ColmapPoint2D& p2d_candidate =
        candidate_image.points2D[static_cast<std::size_t>(m.idx_candidate)];
    if (p2d_candidate.point3D_id < 0) {
      continue;
    }

    const auto it_p3d = model.points3D.find(p2d_candidate.point3D_id);
    if (it_p3d == model.points3D.end()) {
      continue;
    }

    Correspondence2D3D c;
    c.pt2d_px.x() = static_cast<double>(keypoints_current.at<float>(m.idx_current, 0));
    c.pt2d_px.y() = static_cast<double>(keypoints_current.at<float>(m.idx_current, 1));
    c.pt3d_w = it_p3d->second.xyz;
    c.source_image_id = candidate_image_id;
    c.idx_current = m.idx_current;
    c.idx_candidate = m.idx_candidate;
    c.point3D_id = p2d_candidate.point3D_id;
    out.push_back(c);
  }

  return out;
}

}  // namespace loc
