#include "include_ColmapModel.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace {

struct CameraModelInfo {
  const char* name;
  int num_params;
};

const std::unordered_map<int, CameraModelInfo>& CameraModelsById() {
  static const std::unordered_map<int, CameraModelInfo> kModels = {
    {0, {"SIMPLE_PINHOLE", 3}},
    {1, {"PINHOLE", 4}},
    {2, {"SIMPLE_RADIAL", 4}},
    {3, {"RADIAL", 5}},
    {4, {"OPENCV", 8}},
    {5, {"OPENCV_FISHEYE", 8}},
    {6, {"FULL_OPENCV", 12}},
    {7, {"FOV", 5}},
    {8, {"SIMPLE_RADIAL_FISHEYE", 4}},
    {9, {"RADIAL_FISHEYE", 5}},
    {10, {"THIN_PRISM_FISHEYE", 12}},
  };
  return kModels;
}

const std::unordered_map<std::string, int>& CameraModelNumParamsByName() {
  static const std::unordered_map<std::string, int> kModels = {
    {"SIMPLE_PINHOLE", 3},
    {"PINHOLE", 4},
    {"SIMPLE_RADIAL", 4},
    {"RADIAL", 5},
    {"OPENCV", 8},
    {"OPENCV_FISHEYE", 8},
    {"FULL_OPENCV", 12},
    {"FOV", 5},
    {"SIMPLE_RADIAL_FISHEYE", 4},
    {"RADIAL_FISHEYE", 5},
    {"THIN_PRISM_FISHEYE", 12},
  };
  return kModels;
}

template <typename T>
T ReadLE(std::ifstream& ifs, const std::string& what) {
  T value{};
  ifs.read(reinterpret_cast<char*>(&value), sizeof(T));
  if (!ifs) {
    throw std::runtime_error("Erreur de lecture binaire de " + what + ".");
  }
  return value;
}

void LoadCamerasFromText(const std::filesystem::path& cameras_txt,
                        loc::ColmapReconstruction& recon) {
  std::ifstream ifs(cameras_txt);
  if (!ifs.is_open()) {
    throw std::runtime_error("Impossible d'ouvrir " + cameras_txt.string());
  }

  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::istringstream iss(line);
    loc::ColmapCamera cam;
    std::string model_name;
    std::uint64_t width_u64 = 0;
    std::uint64_t height_u64 = 0;
    if (!(iss >> cam.camera_id >> model_name >> width_u64 >> height_u64)) {
      throw std::runtime_error("Ligne cameras.txt invalide: " + line);
    }

    cam.model = model_name;
    cam.width = static_cast<int>(width_u64);
    cam.height = static_cast<int>(height_u64);

    double p = 0.0;
    while (iss >> p) {
      cam.params.push_back(p);
    }

    const auto it_expected = CameraModelNumParamsByName().find(cam.model);
    if (it_expected != CameraModelNumParamsByName().end() &&
        static_cast<int>(cam.params.size()) != it_expected->second) {
      throw std::runtime_error(
          "Nombre de parametres invalide pour le modele " + cam.model +
          " (attendu=" + std::to_string(it_expected->second) +
          ", lu=" + std::to_string(cam.params.size()) + ").");
    }

    recon.cameras[cam.camera_id] = std::move(cam);
  }
}

void LoadCamerasFromBinary(const std::filesystem::path& cameras_bin,
                          loc::ColmapReconstruction& recon) {
  std::ifstream ifs(cameras_bin, std::ios::binary);
  if (!ifs.is_open()) {
    throw std::runtime_error("Impossible d'ouvrir " + cameras_bin.string());
  }

  const std::uint64_t num_cameras = ReadLE<std::uint64_t>(ifs, "num_cameras");
  for (std::uint64_t i = 0; i < num_cameras; ++i) {
    const std::uint32_t camera_id = ReadLE<std::uint32_t>(ifs, "camera_id");
    const std::int32_t model_id = ReadLE<std::int32_t>(ifs, "model_id");
    const std::uint64_t width = ReadLE<std::uint64_t>(ifs, "camera.width");
    const std::uint64_t height = ReadLE<std::uint64_t>(ifs, "camera.height");

    const auto it_model = CameraModelsById().find(static_cast<int>(model_id));
    if (it_model == CameraModelsById().end()) {
      throw std::runtime_error("Model id camera COLMAP inconnu: " + std::to_string(model_id));
    }

    loc::ColmapCamera cam;
    cam.camera_id = static_cast<int>(camera_id);
    cam.model = it_model->second.name;
    cam.width = static_cast<int>(width);
    cam.height = static_cast<int>(height);
    cam.params.resize(static_cast<std::size_t>(it_model->second.num_params));

    for (double& param : cam.params) {
      param = ReadLE<double>(ifs, "camera.param");
    }

    recon.cameras[cam.camera_id] = std::move(cam);
  }
}

void LoadPoints3DFromText(const std::filesystem::path& points3d_txt,
                          loc::ColmapReconstruction& recon) {
  std::ifstream ifs(points3d_txt);
  if (!ifs.is_open()) {
    throw std::runtime_error("Impossible d'ouvrir " + points3d_txt.string());
  }

  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::istringstream iss(line);
    loc::ColmapPoint3D p3d;
    std::uint32_t r = 0;
    std::uint32_t g = 0;
    std::uint32_t b = 0;
    double error = 0.0;
    if (!(iss >> p3d.point3D_id >> p3d.xyz.x() >> p3d.xyz.y() >> p3d.xyz.z() >> r >> g >> b >> error)) {
      throw std::runtime_error("Ligne points3D.txt invalide: " + line);
    }

    recon.points3D[p3d.point3D_id] = std::move(p3d);
  }
}

void LoadPoints3DFromBinary(const std::filesystem::path& points3d_bin,
                            loc::ColmapReconstruction& recon) {
  std::ifstream ifs(points3d_bin, std::ios::binary);
  if (!ifs.is_open()) {
    throw std::runtime_error("Impossible d'ouvrir " + points3d_bin.string());
  }

  const std::uint64_t num_points = ReadLE<std::uint64_t>(ifs, "num_points3D");
  for (std::uint64_t i = 0; i < num_points; ++i) {
    loc::ColmapPoint3D p3d;
    p3d.point3D_id = static_cast<std::int64_t>(ReadLE<std::uint64_t>(ifs, "point3D_id"));
    p3d.xyz.x() = ReadLE<double>(ifs, "point3D.x");
    p3d.xyz.y() = ReadLE<double>(ifs, "point3D.y");
    p3d.xyz.z() = ReadLE<double>(ifs, "point3D.z");

    (void)ReadLE<std::uint8_t>(ifs, "point3D.r");
    (void)ReadLE<std::uint8_t>(ifs, "point3D.g");
    (void)ReadLE<std::uint8_t>(ifs, "point3D.b");
    (void)ReadLE<double>(ifs, "point3D.error");

    const std::uint64_t track_len = ReadLE<std::uint64_t>(ifs, "point3D.track_len");
    for (std::uint64_t k = 0; k < track_len; ++k) {
      (void)ReadLE<std::uint32_t>(ifs, "point3D.track.image_id");
      (void)ReadLE<std::uint32_t>(ifs, "point3D.track.point2d_idx");
    }

    recon.points3D[p3d.point3D_id] = std::move(p3d);
  }
}

void LoadImagesFromText(const std::filesystem::path& images_txt,
                        loc::ColmapReconstruction& recon) {
  std::ifstream ifs(images_txt);
  if (!ifs.is_open()) {
    throw std::runtime_error("Impossible d'ouvrir " + images_txt.string());
  }

  std::string first_line;
  while (std::getline(ifs, first_line)) {
    if (first_line.empty() || first_line[0] == '#') {
      continue;
    }

    std::istringstream iss(first_line);
    loc::ColmapImage img;
    double qw = 1.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;

    if (!(iss >> img.image_id >> qw >> qx >> qy >> qz >> tx >> ty >> tz >> img.camera_id >> img.name)) {
      throw std::runtime_error("Ligne images.txt invalide: " + first_line);
    }

    img.qvec = Eigen::Quaterniond(qw, qx, qy, qz);
    img.tvec = Eigen::Vector3d(tx, ty, tz);

    std::string second_line;
    if (!std::getline(ifs, second_line)) {
      throw std::runtime_error("Ligne points2D manquante pour image_id=" + std::to_string(img.image_id));
    }

    std::istringstream iss_points(second_line);
    double x = 0.0;
    double y = 0.0;
    std::int64_t p3d_id = -1;
    while (iss_points >> x >> y >> p3d_id) {
      loc::ColmapPoint2D p2d;
      p2d.xy = Eigen::Vector2d(x, y);
      p2d.point3D_id = p3d_id;
      img.points2D.push_back(p2d);
    }

    recon.images[img.image_id] = std::move(img);
  }
}

void LoadImagesFromBinary(const std::filesystem::path& images_bin,
                          loc::ColmapReconstruction& recon) {
  std::ifstream ifs(images_bin, std::ios::binary);
  if (!ifs.is_open()) {
    throw std::runtime_error("Impossible d'ouvrir " + images_bin.string());
  }

  const std::uint64_t num_images = ReadLE<std::uint64_t>(ifs, "num_images");
  for (std::uint64_t i = 0; i < num_images; ++i) {
    loc::ColmapImage img;
    img.image_id = static_cast<int>(ReadLE<std::uint32_t>(ifs, "image_id"));

    const double qw = ReadLE<double>(ifs, "image.qw");
    const double qx = ReadLE<double>(ifs, "image.qx");
    const double qy = ReadLE<double>(ifs, "image.qy");
    const double qz = ReadLE<double>(ifs, "image.qz");
    img.qvec = Eigen::Quaterniond(qw, qx, qy, qz);

    img.tvec.x() = ReadLE<double>(ifs, "image.tx");
    img.tvec.y() = ReadLE<double>(ifs, "image.ty");
    img.tvec.z() = ReadLE<double>(ifs, "image.tz");

    img.camera_id = static_cast<int>(ReadLE<std::uint32_t>(ifs, "image.camera_id"));

    std::string name;
    char c = '\0';
    while (ifs.get(c)) {
      if (c == '\0') {
        break;
      }
      name.push_back(c);
    }
    if (!ifs) {
      throw std::runtime_error("Erreur de lecture du nom d'image en binaire.");
    }
    img.name = name;

    const std::uint64_t num_points2d = ReadLE<std::uint64_t>(ifs, "image.num_points2D");
    img.points2D.reserve(static_cast<std::size_t>(num_points2d));
    for (std::uint64_t k = 0; k < num_points2d; ++k) {
      loc::ColmapPoint2D p2d;
      p2d.xy.x() = ReadLE<double>(ifs, "image.point2D.x");
      p2d.xy.y() = ReadLE<double>(ifs, "image.point2D.y");
      p2d.point3D_id = ReadLE<std::int64_t>(ifs, "image.point2D.point3D_id");
      img.points2D.push_back(p2d);
    }

    recon.images[img.image_id] = std::move(img);
  }
}

}  // namespace

namespace loc {

ColmapModelLoader::ColmapModelLoader() = default;

ColmapModelLoader::ColmapModelLoader(Options opt) : opt_(opt) {}

ColmapReconstruction ColmapModelLoader::LoadFromSparseDir(const std::string& sparse_dir) const {
  const std::filesystem::path sparse_path(sparse_dir);
  const std::filesystem::path cameras_bin = sparse_path / "cameras.bin";
  const std::filesystem::path cameras_txt = sparse_path / "cameras.txt";
  const std::filesystem::path images_bin = sparse_path / "images.bin";
  const std::filesystem::path images_txt = sparse_path / "images.txt";
  const std::filesystem::path points3d_bin = sparse_path / "points3D.bin";
  const std::filesystem::path points3d_txt = sparse_path / "points3D.txt";

  ColmapReconstruction recon;

  const bool has_cameras_bin = std::filesystem::exists(cameras_bin);
  const bool has_cameras_txt = std::filesystem::exists(cameras_txt);
  const bool has_images_bin = std::filesystem::exists(images_bin);
  const bool has_images_txt = std::filesystem::exists(images_txt);
  const bool has_points3d_bin = std::filesystem::exists(points3d_bin);
  const bool has_points3d_txt = std::filesystem::exists(points3d_txt);

  if (!has_cameras_bin && !has_cameras_txt) {
    throw std::runtime_error(
      "Aucun fichier cameras.bin/cameras.txt trouve dans " + sparse_path.string());
  }
  if (!has_images_bin && !has_images_txt) {
    throw std::runtime_error(
      "Aucun fichier images.bin/images.txt trouve dans " + sparse_path.string());
  }
  if (!has_points3d_bin && !has_points3d_txt) {
    throw std::runtime_error(
      "Aucun fichier points3D.bin/points3D.txt trouve dans " + sparse_path.string());
  }

  if (opt_.prefer_binary) {
    if (has_cameras_bin) {
      LoadCamerasFromBinary(cameras_bin, recon);
    } else {
      LoadCamerasFromText(cameras_txt, recon);
    }

    if (has_images_bin) {
      LoadImagesFromBinary(images_bin, recon);
    } else {
      LoadImagesFromText(images_txt, recon);
    }

    if (has_points3d_bin) {
      LoadPoints3DFromBinary(points3d_bin, recon);
    } else {
      LoadPoints3DFromText(points3d_txt, recon);
    }
  } else {
    if (has_cameras_txt) {
      LoadCamerasFromText(cameras_txt, recon);
    } else {
      LoadCamerasFromBinary(cameras_bin, recon);
    }

    if (has_images_txt) {
      LoadImagesFromText(images_txt, recon);
    } else {
      LoadImagesFromBinary(images_bin, recon);
    }

    if (has_points3d_txt) {
      LoadPoints3DFromText(points3d_txt, recon);
    } else {
      LoadPoints3DFromBinary(points3d_bin, recon);
    }
  }

  return recon;
}

}  // namespace loc