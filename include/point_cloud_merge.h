#pragma once

#include <string>
#include <vector>

int MergePointCloudFiles(const std::string& output_file,
                         const std::vector<std::string>& input_files);

int PoissonFromPointCloudFiles(const std::string& output_file,
                               const std::vector<std::string>& input_files);

int BallPivotingFromPointCloudFiles(const std::string& output_file,
                                    const std::vector<std::string>& input_files,
                                    double radius = 0.005);