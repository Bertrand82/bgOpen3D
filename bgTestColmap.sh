#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
echo "------------------------------------------------------------"
echo "Script directory: ${SCRIPT_DIR}"
echo "Build directory: ${BUILD_DIR}"
echo "------------------------------------------------------------"
# cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}"
# cmake --build "${BUILD_DIR}" -j
ctest --test-dir "${BUILD_DIR}" --output-on-failure

./build/test_colmap_model_loader_cameras_points3D_images
./build/test_intrinsics_undistort_to_tempdir
