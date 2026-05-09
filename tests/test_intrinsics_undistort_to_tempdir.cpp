#include "include_CameraIntrinsics.hpp"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

namespace {

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

  throw std::runtime_error("Impossible de trouver tests/assets/sparse/0.");
}


}  // namespace

int main() {
  try {
    std::cout << "Test intrinsics/undistort starting..." << std::endl;
    const std::filesystem::path sparse_dir = FindSparseDir();

    loc::ColmapModelLoader loader(loc::ColmapModelLoader::Options{true});
    const loc::ColmapReconstruction recon = loader.LoadFromSparseDir(sparse_dir.string());

    const auto it_cam = recon.cameras.find(1);
    if (it_cam == recon.cameras.end()) {
      throw std::runtime_error("Camera 1 introuvable");
    }

    const loc::CameraIntrinsics intrinsics = loc::BuildIntrinsicsFromColmap(it_cam->second);
    if (intrinsics.K.empty()) {
      throw std::runtime_error("K est vide");
    }

    const std::filesystem::path out_dir =
        std::filesystem::temp_directory_path() / "georeloc_intrinsics_test_outputs";
    std::filesystem::create_directories(out_dir);
    std::cout << "BG Dossier de sortie: " << out_dir << std::endl;
    
  
    const std::vector<std::string> image_files = getImagesFromDir(sparse_dir.parent_path().parent_path() / "images"); 
    
    for (long unsigned i = 0; i < 3; ++i) {
      const std::filesystem::path image_path = sparse_dir.parent_path().parent_path() / "images" / image_files[i];
      const cv::Mat raw = cv::imread(image_path.string());
      if (raw.empty()) {
        throw std::runtime_error("Impossible de lire l'image: " + image_path.string());
      }

      const std::filesystem::path out_path_original = out_dir / ("originel_" + image_files[i]);
      cv::imwrite(out_path_original.string(), raw);
      
      const cv::Mat transformed = loc::UndistortImage(raw, intrinsics);
     const std::filesystem::path out_path = out_dir / ("transformed_" + image_files[i]);
      if (!cv::imwrite(out_path.string(), transformed)) {
        throw std::runtime_error("Echec ecriture: " + out_path.string());
      }
      
    
      std::cout << "BG Dossier de sortie: " << i<<"  "<< out_path << "\n" ;
      if (!std::filesystem::exists(out_path)) {
        throw std::runtime_error("Fichier absent apres ecriture: " + out_path.string());
      }
    }

    std::cout << "Test intrinsics/undistort OK. Dossier: " << out_dir << std::endl;
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Test intrinsics/undistort KO: " << e.what() << std::endl;
    return 1;
  }
}
