// Test: ColmapStyleSiftExtractor sur ./data/IMAGE_CURRENT.JPG
// Vérifie que l'extraction retourne des keypoints/descripteurs valides.

#include "include_FeatureExtraction.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char* argv[]) {
    // Chemin vers l'image : arg optionnel ou chemin par défaut relatif au répertoire d'exécution
    std::string image_path = "./data/IMAGE_CURRENT.JPG";
    if (argc > 1) {
        image_path = argv[1];
    }

    cv::Mat image_bgr = cv::imread(image_path, cv::IMREAD_COLOR);
    if (image_bgr.empty()) {
        std::cerr << "[FAIL] Impossible de charger l'image : " << image_path << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "[INFO] Image chargée : " << image_path
              << "  taille=" << image_bgr.cols << "x" << image_bgr.rows << std::endl;

    cv::Mat image_gray;
    cv::cvtColor(image_bgr, image_gray, cv::COLOR_BGR2GRAY);

    loc::ColmapStyleSiftExtractor extractor;
    loc::ExtractedFeaturesColmapStyle result = extractor.Extract(image_gray);

    const int n = result.keypoints.rows;
    std::cout << "[INFO] Nombre de keypoints extraits : " << n << std::endl;

    // --- Vérifications ---

    if (n == 0) {
        std::cerr << "[FAIL] Aucun keypoint extrait." << std::endl;
        return EXIT_FAILURE;
    }

    if (result.keypoints.cols != 4 || result.keypoints.type() != CV_32F) {
        std::cerr << "[FAIL] keypoints: attendu Nx4 CV_32F, obtenu "
                  << result.keypoints.cols << " cols, type=" << result.keypoints.type()
                  << std::endl;
        return EXIT_FAILURE;
    }

    if (result.descriptors.rows != n || result.descriptors.cols != 128 ||
        result.descriptors.type() != CV_8U) {
        std::cerr << "[FAIL] descriptors: attendu " << n
                  << "x128 CV_8U, obtenu " << result.descriptors.rows
                  << "x" << result.descriptors.cols
                  << " type=" << result.descriptors.type() << std::endl;
        return EXIT_FAILURE;
    }

    // Afficher les 5 premiers keypoints (x, y, scale, angle_rad)
    std::cout << "[INFO] Premiers keypoints (x, y, scale, angle_rad) :" << std::endl;
    const int display_count = std::min(n, 5);
    for (int i = 0; i < display_count; ++i) {
        std::cout << "  [" << i << "] "
                  << result.keypoints.at<float>(i, 0) << ", "
                  << result.keypoints.at<float>(i, 1) << ", "
                  << result.keypoints.at<float>(i, 2) << ", "
                  << result.keypoints.at<float>(i, 3) << std::endl;
    }

    std::cout << "[PASS] test_feature_extraction_image" << std::endl;
    return EXIT_SUCCESS;
}
