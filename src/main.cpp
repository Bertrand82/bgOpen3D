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
    if (argc < 2) {
        PrintUsage(argv[0]);
        return 1;
    }

    const std::string mode = argv[1];

    if (mode == "--merge") {
        if (argc < 5) {
            PrintUsage(argv[0]);
            return 1;
        }

        const std::string output_file = argv[2];
        std::vector<std::string> input_files;
        input_files.reserve(static_cast<std::size_t>(argc - 3));

        for (int i = 3; i < argc; ++i) {
            input_files.emplace_back(argv[i]);
        }

        return MergePointCloudFiles(output_file, input_files);
    }

    if (mode == "--poisson") {
        if (argc < 4) {
            PrintUsage(argv[0]);
            return 1;
        }

        const std::string output_file = argv[2];
        std::vector<std::string> input_files;
        input_files.reserve(static_cast<std::size_t>(argc - 3));

        for (int i = 3; i < argc; ++i) {
            input_files.emplace_back(argv[i]);
        }

        return PoissonFromPointCloudFiles(output_file, input_files);
    }

    if (mode == "--buildMeshBallPivoting") {
        double radius = 0.005;
        int output_index = 2;

        if (argc >= 3 && std::string(argv[2]) == "--radius") {
            if (argc < 6) {
                PrintUsage(argv[0]);
                return 1;
            }

            try {
                radius = std::stod(argv[3]);
            } catch (const std::exception&) {
                std::cerr << "Error: invalid radius value: " << argv[3] << std::endl;
                return 1;
            }

            if (radius <= 0.0) {
                std::cerr << "Error: radius must be > 0." << std::endl;
                return 1;
            }

            output_index = 4;
        }

        if (argc < output_index + 2) {
            PrintUsage(argv[0]);
            return 1;
        }

        const std::string ball_output_file = argv[output_index];
        std::vector<std::string> ball_input_files;
        ball_input_files.reserve(static_cast<std::size_t>(argc - (output_index + 1)));

        for (int i = output_index + 1; i < argc; ++i) {
            ball_input_files.emplace_back(argv[i]);
        }

        if (ball_input_files.empty()) {
                PrintUsage(argv[0]);
                return 1;
        }

        return BallPivotingFromPointCloudFiles(ball_output_file, ball_input_files, radius);
    }

    PrintUsage(argv[0]);
    return 1;
}