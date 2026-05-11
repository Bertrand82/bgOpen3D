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
              << " --poisson [--depth value] [--width value] [--scale value] [--threads value]"
              << " [--voxel value] [--auto-depth] [--linear-fit] output.ply input1.ply input2.ply [input3.ply ...]\n"
              << "       " << program_name
              << " --buildMeshBallPivoting [--radius value] output.ply input1.ply input2.ply [input3.ply ...]\n"
              << "       " << program_name
              << " --post-merge [--voxel size] [--dedup-eps eps] [--skip-dedup] input.ply output.ply"
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
        int depth = 8;
        float width = 0.0f;
        float scale = 1.1f;
        bool linear_fit = false;
        int n_threads = -1;
        double voxel_size = 0.0;
        bool auto_depth = false;

        int index = 2;
        while (index < argc) {
            const std::string arg = argv[index];
            if (arg == "--linear-fit") {
                linear_fit = true;
                ++index;
                continue;
            }

            if (arg == "--auto-depth") {
                auto_depth = true;
                ++index;
                continue;
            }

            if (arg == "--depth" || arg == "--width" || arg == "--scale" ||
                arg == "--threads" || arg == "--voxel") {
                if (index + 1 >= argc) {
                    std::cerr << "Error: missing value for " << arg << std::endl;
                    return 1;
                }

                try {
                    if (arg == "--depth") {
                        depth = std::stoi(argv[index + 1]);
                    } else if (arg == "--width") {
                        width = std::stof(argv[index + 1]);
                    } else if (arg == "--scale") {
                        scale = std::stof(argv[index + 1]);
                    } else if (arg == "--threads") {
                        n_threads = std::stoi(argv[index + 1]);
                    } else if (arg == "--voxel") {
                        voxel_size = std::stod(argv[index + 1]);
                    }
                } catch (const std::exception&) {
                    std::cerr << "Error: invalid value for " << arg << ": " << argv[index + 1] << std::endl;
                    return 1;
                }

                index += 2;
                continue;
            }

            if (arg.rfind("--", 0) == 0) {
                std::cerr << "Error: unknown option for --poisson: " << arg << std::endl;
                return 1;
            }

            break;
        }

        if (!auto_depth && depth <= 0) {
            std::cerr << "Error: --depth must be > 0." << std::endl;
            return 1;
        }

        if (width < 0.0f) {
            std::cerr << "Error: --width must be >= 0." << std::endl;
            return 1;
        }

        if (scale <= 0.0f) {
            std::cerr << "Error: --scale must be > 0." << std::endl;
            return 1;
        }

        if (voxel_size < 0.0) {
            std::cerr << "Error: --voxel must be >= 0." << std::endl;
            return 1;
        }

        if (argc - index < 2) {
            PrintUsage(argv[0]);
            return 1;
        }

        const std::string output_file = argv[index];
        std::vector<std::string> input_files;
        input_files.reserve(static_cast<std::size_t>(argc - (index + 1)));

        for (int i = index + 1; i < argc; ++i) {
            input_files.emplace_back(argv[i]);
        }

        return PoissonFromPointCloudFiles(output_file,
                                          input_files,
                                          depth,
                                          width,
                                          scale,
                                          linear_fit,
                                          n_threads,
                                          voxel_size,
                                          auto_depth);
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

    if (mode == "--post-merge") {
        double voxel_size = 0.0;
        double dedup_eps = 0.0;
        bool skip_dedup = false;

        int index = 2;
        while (index < argc) {
            const std::string arg = argv[index];

            if (arg == "--skip-dedup") {
                skip_dedup = true;
                ++index;
                continue;
            }

            if (arg == "--voxel" || arg == "--dedup-eps") {
                if (index + 1 >= argc) {
                    std::cerr << "Error: missing value for " << arg << std::endl;
                    return 1;
                }

                try {
                    if (arg == "--voxel") {
                        voxel_size = std::stod(argv[index + 1]);
                    } else {
                        dedup_eps = std::stod(argv[index + 1]);
                    }
                } catch (const std::exception&) {
                    std::cerr << "Error: invalid value for " << arg << ": " << argv[index + 1] << std::endl;
                    return 1;
                }

                index += 2;
                continue;
            }

            if (arg.rfind("--", 0) == 0) {
                std::cerr << "Error: unknown option for --post-merge: " << arg << std::endl;
                return 1;
            }

            break;
        }

        if (argc - index != 2) {
            PrintUsage(argv[0]);
            return 1;
        }

        const std::string input_file = argv[index];
        const std::string output_file = argv[index + 1];
        return PostMergePointCloudFile(output_file,
                                       input_file,
                                       voxel_size,
                                       dedup_eps,
                                       skip_dedup);
    }

    PrintUsage(argv[0]);
    return 1;
}