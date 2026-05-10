#pragma once

#include <string>
#include <vector>

int MergePointCloudFiles(const std::string& output_file,
                         const std::vector<std::string>& input_files);

int PoissonFromPointCloudFiles(const std::string& output_file,
                               const std::vector<std::string>& input_files,
                               int depth = 8,
                               float width = 0.0f,
                               float scale = 1.1f,
                               bool linear_fit = false,
                               int n_threads = -1,
                               double voxel_size = 0.0,
                               bool auto_depth = false);

int BallPivotingFromPointCloudFiles(const std::string& output_file,
                                    const std::vector<std::string>& input_files,
                                    double radius = 0.005);