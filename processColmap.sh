#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"



COLMAP_CMD="/home/bertrand/workspaceCpp/colmap/build/src/colmap/exe/colmap"


echo "COLMAP binaire utilisé: $COLMAP_CMD"
"$COLMAP_CMD" --version
BG_WORK="${BG_WORK:-/data/BG}"
ls "$BG_WORK"
echo "COLMAP EXTRACTOR"
"$COLMAP_CMD" feature_extractor --help
echo "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"

# Lister le contenu d'un dossier (équivalent de "dir" sous Windows)
ls -la "$BG_WORK"
echo "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"

# Extraction des features
"$COLMAP_CMD" feature_extractor \
  --database_path "$BG_WORK/database.db" \
  --image_path "$BG_WORK/images" \
  --ImageReader.single_camera 1 \
  --ImageReader.camera_model SIMPLE_RADIAL \
  --FeatureExtraction.use_gpu 0 \
  --FeatureExtraction.num_threads 1 \
  --log_level 2

echo "MATCH"
"$COLMAP_CMD" matches_importer --help

# Import des matches (fichier match.txt)
"$COLMAP_CMD" matches_importer \
  --database_path "$BG_WORK/database.db" \
  --FeatureMatching.use_gpu 0 \
  --match_list_path "$BG_WORK/match.txt"
echo "xxxxxxxxxx Rajout des position gps estimés xxxxxxxxxxxxxxxxxxx"
echo "MAPPER"

# Créer le dossier sparse (équivalent de "mkd")
mkdir -p "$BG_WORK/sparse"

# Reconstruction sparse
"$COLMAP_CMD" mapper \
  --database_path "$BG_WORK/database.db" \
  --image_path "$BG_WORK/images" \
  --output_path "$BG_WORK/sparse"

echo "CONVERTER"

# Export PLY
"$COLMAP_CMD" model_converter \
  --input_path "$BG_WORK/sparse/0" \
  --output_path "$BG_WORK/sparse/0/points3D.ply" \
  --output_type PLY

# Export TXT (cameras.txt / images.txt / points3D.txt)
"$COLMAP_CMD" model_converter \
  --input_path "$BG_WORK/sparse/0" \
  --output_path "$BG_WORK/sparse/0" \
  --output_type TXT
