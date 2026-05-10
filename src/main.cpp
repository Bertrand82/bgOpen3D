#include "point_cloud_merge.h"

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void PrintUsage(const char* program_name) {
    std::cerr << "Usage: " << program_name
              << " --merge output.ply input1.ply input2.ply [input3.ply ...]\n"
              << "       " << program_name
              << " --poisson output.ply input1.ply input2.ply [input3.ply ...]\n"
              << "       " << program_name
              << " --buildMeshBallPivoting [--radius value] output.ply input1.ply input2.ply [input3.ply ...]"
              << std::endl;
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 5) {
        PrintUsage(argv[0]);
        return 1;
    }

    const std::string mode = argv[1];
    const std::string output_file = argv[2];
    std::vector<std::string> input_files;
    input_files.reserve(static_cast<std::size_t>(argc - 3));

    for (int i = 3; i < argc; ++i) {
        input_files.emplace_back(argv[i]);
    }

    if (mode == "--merge") {
        return MergePointCloudFiles(output_file, input_files);
    }

    if (mode == "--poisson") {
        return PoissonFromPointCloudFiles(output_file, input_files);
    }

    if (mode == "--buildMeshBallPivoting") {
        double radius = 0.005;
        int first_input_index = 3;

        if (argc >= 7 && std::string(argv[2]) == "--radius") {
            try {
                radius = std::stod(argv[3]);
            } catch (const std::exception&) {
                std::cerr << "Error: invalid radius value: " << argv[3] << std::endl;
                return 1;
            }

            if (argc < 7) {
                PrintUsage(argv[0]);
                return 1;
            }

            first_input_index = 5;
        }

        const std::string ball_output_file = argv[first_input_index - 1];
        std::vector<std::string> ball_input_files;
        ball_input_files.reserve(static_cast<std::size_t>(argc - first_input_index));

        for (int i = first_input_index; i < argc; ++i) {
            ball_input_files.emplace_back(argv[i]);
        }

        if (ball_input_files.size() < 2) {
            PrintUsage(argv[0]);
            return 1;
        }

        return BallPivotingFromPointCloudFiles(ball_output_file, ball_input_files, radius);
    }

    PrintUsage(argv[0]);
    return 1;
}