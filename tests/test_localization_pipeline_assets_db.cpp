#include "include_LocalizationPipeline.hpp"
#include "include_CoordinateConverter.hpp"

#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

/**
 * resources :
 * /geometry/pose_prior.h
 */

namespace {

std::filesystem::path FindAssetsDir() {
  const std::vector<std::filesystem::path> candidates = {
      "data",
      "../data",
      "../../data",
      "../../../data",
  };

  for (const auto& candidate : candidates) {
    if (std::filesystem::exists(candidate / "database.db") &&
        std::filesystem::exists(candidate / "sparse/0") &&
        std::filesystem::exists(candidate / "images")) {
      return candidate;
    }
  }

  throw std::runtime_error("Impossible de trouver data/.");
}

class FixedImageIdSelector final : public loc::ICandidateSelector {
public:
  explicit FixedImageIdSelector(int image_id) : image_id_(image_id) {}

  std::vector<int> SelectCandidates(const loc::ColmapReconstruction&,
                                    const Eigen::Vector3d&) const override {
    return {image_id_};
  }

private:
  int image_id_ = -1;
};

class DatabaseBackedExtractor final : public loc::IFeatureExtractor {
public:
  DatabaseBackedExtractor(const std::string& db_path, int image_id)
      : db_(db_path), image_id_(image_id) {}

  loc::ExtractedFeaturesColmapStyle Extract(const cv::Mat&) const override {
    loc::ExtractedFeaturesColmapStyle out;
    out.keypoints = db_.LoadKeypoints(image_id_);
    out.descriptors = db_.LoadDescriptors(image_id_);
    return out;
  }

private:
  mutable loc::ColmapDatabase db_;
  int image_id_ = -1;
};

}  // namespace

int main() {
  try {
    const std::filesystem::path assets_dir = FindAssetsDir();
    const std::filesystem::path db_path = assets_dir / "database.db";
    const std::filesystem::path sparse_dir = assets_dir / "sparse/0";

    const int image_id = 1;

    loc::ColmapModelLoader loader(loc::ColmapModelLoader::Options{true});
    const loc::ColmapReconstruction recon = loader.LoadFromSparseDir(sparse_dir.string());
    const auto it_img = recon.images.find(image_id);
    if (it_img == recon.images.end()) {
      throw std::runtime_error("image_id=1 introuvable dans la reconstruction.");
    }

    const std::filesystem::path query_image = assets_dir / "images" / it_img->second.name;
    if (!std::filesystem::exists(query_image)) {
      throw std::runtime_error("Image query introuvable: " + query_image.string());
    }
    std::cout << "BG query_image: " << query_image << std::endl;

    loc::LocalizationOptions options;
    options.matcher.cross_check = true;
    options.pnp.minInliers = 15;
    options.pnp.reprojectionError = 8.0F;
    options.candidate.max_candidates = 1;

    loc::LocalizationPipeline pipeline(options);
    pipeline.SetCandidateSelector(std::make_unique<FixedImageIdSelector>(image_id));
    pipeline.SetFeatureExtractor(std::make_unique<DatabaseBackedExtractor>(db_path.string(), image_id));

    loc::LocalizationInputs in;
    in.sparse_dir = sparse_dir.string();
    in.database_path = db_path.string();
    in.image_current_path = query_image.string();

    const loc::LocalizationOutputs out = pipeline.Run(in);

    if (out.candidate_image_ids.size() != 1U || out.candidate_image_ids[0] != image_id) {
      throw std::runtime_error("Le candidat attendu n'a pas ete utilise.");
    }
    if (out.total_matches_2d2d <= 0) {
      throw std::runtime_error("Aucun match 2D-2D dans le pipeline.");
    }
    if (out.total_correspondences_2d3d <= 0) {
      throw std::runtime_error("Aucune correspondance 2D-3D dans le pipeline.");
    }
    if (!out.estimate.success) {
      throw std::runtime_error("PnP a echoue alors que les features viennent de la DB de reference.");
    }
    if (out.estimate.num_inliers < options.pnp.minInliers) {
      throw std::runtime_error("Nombre d'inliers insuffisant.");
    }

    std::cout << "Test LocalizationPipeline assets DB OK: matches=" << out.total_matches_2d2d
              << ", corr=" << out.total_correspondences_2d3d
              << ", inliers=" << out.estimate.num_inliers << std::endl;
    std::cout << "Pose estimee (w2c) R:\n" <<  out.estimate.pose_w2c.R << std::endl;  
    std::cout << "Pose estimee (w2c) t:\n" <<  out.estimate.pose_w2c.t  << std::endl;  
    std::cout << "Camera position (monde): "
              << "x_camera=" << out.estimate.derived.C_world.x()
              << ", y_camera=" << out.estimate.derived.C_world.y()
              << ", z_camera=" << out.estimate.derived.C_world.z() << std::endl;

    const loc::CoordinateConverter converter;
    const loc::GeodeticCoordinate geodetic = converter.EcefToGeodetic(
      out.estimate.derived.C_world.x(),
      out.estimate.derived.C_world.y(),
      out.estimate.derived.C_world.z());

    std::cout << "Position geodesique (camera): "
          << "latitude=" << geodetic.latitude_deg
          << ", longitude=" << geodetic.longitude_deg
          << ", z=" << geodetic.altitude_m << std::endl;
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Test LocalizationPipeline assets DB KO: " << e.what() << std::endl;
    return 1;
  }
}
