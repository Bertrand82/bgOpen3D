#include "include_ColmapModel.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

bool AlmostEqual(double a, double b, double eps = 100) {
  return std::abs(a - b) <= (b/eps);
}

std::filesystem::path FindSparseDir() {
  const std::vector<std::filesystem::path> candidates = {
      "data/sparse/0",
      "../data/sparse/0",
      "../../data/sparse/0",
      "../../../data/sparse/0",
  };

  for (const auto& candidate : candidates) {
    if (std::filesystem::exists(candidate / "cameras.bin") &&
        std::filesystem::exists(candidate / "cameras.txt")) {
      return candidate;
    }
  }

  throw std::runtime_error(
      "Impossible de trouver data/sparse/0 depuis le repertoire courant.");
}

void CheckCamera(const loc::ColmapCamera& cam) {
  std::cout << "CheckCamera Camera ID: " << cam.camera_id << "model "<< cam.model << "\n";
  if (cam.camera_id > 2) {
    throw std::runtime_error("camera_id attendu=1 was " + std::to_string(cam.camera_id));
  }
  if (cam.model != "SIMPLE_RADIAL") {
    throw std::runtime_error("model attendu=SIMPLE_RADIAL was " + cam.model);
  }
  if (cam.width != 4032 || cam.height != 3024) {
    throw std::runtime_error("width/height inattendus");
  }
  if (cam.params.size() != 4U) {
    throw std::runtime_error("nombre de parametres attendu=4 was " + std::to_string(cam.params.size()));
  }

  if (AlmostEqual(cam.params[0],0.0,1000)) {
    throw std::runtime_error("param[0] expected  was "+std::to_string(cam.params[0]));
  }
  if (AlmostEqual(cam.params[1], 0.0000,1000)) {
    throw std::runtime_error("param[1] expected was "+std::to_string(cam.params[1]));
  }
  if (AlmostEqual(cam.params[2], 0.0,1000)) {
    throw std::runtime_error("param[2] expected was "+std::to_string(cam.params[2]));
  }
  if (AlmostEqual(cam.params[3], 0.0,1000)) {
    throw std::runtime_error("param[3] expected was "+std::to_string(cam.params[3]));
  }
}

void CheckImagesAndPoints3D(const loc::ColmapReconstruction& recon) {
  std::cout << "CheckImagesAndPoints3D nb images: " << recon.images.size() << " nb points3D: " << recon.points3D.size() << "\n"; 
  if (recon.images.empty()) {
    throw std::runtime_error("Aucune image chargee");
  }
  if (recon.points3D.empty()) {
    throw std::runtime_error("Aucun point3D charge");
  }

  const auto it_img = recon.images.find(5);
  if (it_img == recon.images.end()) {
    throw std::runtime_error("Image 5 introuvable");
  }
  if (it_img->second.camera_id > 2) {
    throw std::runtime_error("camera_id inattendu pour image 5 attendu=1 was " + std::to_string(it_img->second.camera_id) );
  }
  if (it_img->second.points2D.empty()) {
    throw std::runtime_error("points2D vides pour image 5");
  }

  const auto it_p3d = recon.points3D.find(1);
  if (it_p3d == recon.points3D.end()) {
    throw std::runtime_error("Point3D 1 introuvable");
  }
}

}  // namespace

int main() {
  try {
    std::cout << "Test ColmapModelLoader (camera, images, points3D) starting..." << std::endl;
    const std::filesystem::path sparse_dir = FindSparseDir();

    loc::ColmapModelLoader loader_bin(loc::ColmapModelLoader::Options{true});
    const loc::ColmapReconstruction recon_bin =
        loader_bin.LoadFromSparseDir(sparse_dir.string());


    const auto it_bin = recon_bin.cameras.find(1);
    if (it_bin == recon_bin.cameras.end()) {
      throw std::runtime_error("Camera 1 introuvable en lecture binaire");
    }
    CheckCamera(it_bin->second);
    CheckImagesAndPoints3D(recon_bin);

    loc::ColmapModelLoader loader_txt(loc::ColmapModelLoader::Options{false});
    const loc::ColmapReconstruction recon_txt =
        loader_txt.LoadFromSparseDir(sparse_dir.string());


    const auto it_txt = recon_txt.cameras.find(1);
    if (it_txt == recon_txt.cameras.end()) {
      throw std::runtime_error("Camera 1 introuvable en lecture texte");
    }
    CheckCamera(it_txt->second);
    CheckImagesAndPoints3D(recon_txt);

    std::cout << "Test ColmapModelLoader (camera) OK" << std::endl;
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Test ColmapModelLoader (camera) KO: " << e.what() << std::endl;
    return 1;
  }
}
