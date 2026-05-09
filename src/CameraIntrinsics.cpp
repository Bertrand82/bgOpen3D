#include "include_CameraIntrinsics.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <stdexcept>
#include <vector>

#include <opencv2/calib3d.hpp>

namespace loc {

namespace {

cv::Mat MakeK(double fx, double fy, double cx, double cy) {
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = fx;
  K.at<double>(1, 1) = fy;
  K.at<double>(0, 2) = cx;
  K.at<double>(1, 2) = cy;
  return K;
}

cv::Mat MakeDist(const std::vector<double>& coeffs) {
  cv::Mat dist(static_cast<int>(coeffs.size()), 1, CV_64F);
  for (int i = 0; i < static_cast<int>(coeffs.size()); ++i) {
    dist.at<double>(i, 0) = coeffs[static_cast<std::size_t>(i)];
  }
  return dist;
}

}  // namespace

CameraIntrinsics BuildIntrinsicsFromColmap(const ColmapCamera& camera) {
  CameraIntrinsics out;
  out.model = camera.model;
  out.width = camera.width;
  out.height = camera.height;

  const std::vector<double>& p = camera.params;

  if (camera.model == "SIMPLE_PINHOLE") {
    if (p.size() != 3U) {
      throw std::runtime_error("SIMPLE_PINHOLE attend 3 parametres");
    }
    out.K = MakeK(p[0], p[0], p[1], p[2]);
    out.dist = MakeDist({});
    return out;
  }

  if (camera.model == "PINHOLE") {
    if (p.size() != 4U) {
      throw std::runtime_error("PINHOLE attend 4 parametres");
    }
    out.K = MakeK(p[0], p[1], p[2], p[3]);
    out.dist = MakeDist({});
    return out;
  }

  if (camera.model == "SIMPLE_RADIAL") {
    if (p.size() != 4U) {
      throw std::runtime_error("SIMPLE_RADIAL attend 4 parametres");
    }
    out.K = MakeK(p[0], p[0], p[1], p[2]);
    out.dist = MakeDist({p[3], 0.0, 0.0, 0.0});
    return out;
  }

  if (camera.model == "RADIAL") {
    if (p.size() != 5U) {
      throw std::runtime_error("RADIAL attend 5 parametres");
    }
    out.K = MakeK(p[0], p[0], p[1], p[2]);
    out.dist = MakeDist({p[3], p[4], 0.0, 0.0});
    return out;
  }

  if (camera.model == "OPENCV") {
    if (p.size() != 8U) {
      throw std::runtime_error("OPENCV attend 8 parametres");
    }
    out.K = MakeK(p[0], p[1], p[2], p[3]);
    out.dist = MakeDist({p[4], p[5], p[6], p[7]});
    return out;
  }

  if (camera.model == "OPENCV_FISHEYE") {
    if (p.size() != 8U) {
      throw std::runtime_error("OPENCV_FISHEYE attend 8 parametres");
    }
    out.K = MakeK(p[0], p[1], p[2], p[3]);
    out.dist = MakeDist({p[4], p[5], p[6], p[7]});
    return out;
  }

  throw std::runtime_error("Modele camera COLMAP non supporte: " + camera.model);
}

cv::Mat UndistortImage(const cv::Mat& image, const CameraIntrinsics& intrinsics) {
  if (image.empty()) {
    throw std::runtime_error("Image d'entree vide");
  }
  if (intrinsics.K.empty()) {
    throw std::runtime_error("Matrice K vide");
  }

  cv::Mat undistorted;
  if (intrinsics.model == "OPENCV_FISHEYE") {
    cv::fisheye::undistortImage(image, undistorted, intrinsics.K, intrinsics.dist);
  } else {
    cv::undistort(image, undistorted, intrinsics.K, intrinsics.dist);
  }
  return undistorted;
}

}  // namespace loc

std::vector<std::string> getImagesFromDir(const std::filesystem::path& dir_path) {
  if (!std::filesystem::exists(dir_path)) {
    throw std::runtime_error("Dossier images introuvable: " + dir_path.string());
  }
  if (!std::filesystem::is_directory(dir_path)) {
    throw std::runtime_error("Le chemin n'est pas un dossier: " + dir_path.string());
  }

  std::vector<std::string> image_files;
  for (const auto& entry : std::filesystem::directory_iterator(dir_path)) {
    if (!entry.is_regular_file()) {
      continue;
    }

    std::string extension = entry.path().extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char ch) {
                     return static_cast<char>(std::tolower(ch));
                   });

    if (extension == ".jpg" || extension == ".jpeg" || extension == ".png" ||
        extension == ".bmp" || extension == ".tif" || extension == ".tiff") {
      image_files.push_back(entry.path().filename().string());
    }
  }

  std::sort(image_files.begin(), image_files.end());
  if (image_files.empty()) {
    throw std::runtime_error("Aucune image trouvee dans: " + dir_path.string());
  }

  return image_files;
}
