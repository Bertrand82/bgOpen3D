#include "point_cloud_merge.h"

#include <iostream>
#include <string>

namespace {

void PrintUsage(const char* program_name) {
    std::cerr
            << "Usage: " << program_name
            << " [--voxel size] [--dedup-eps eps] [--skip-dedup] input.ply output.ply\n"
            << "  --voxel size     Optional voxel downsample size (> 0).\n"
            << "  --dedup-eps eps  Optional dedup tolerance (>= 0). 0 means exact dedup.\n"
            << "  --skip-dedup     Skip duplicate suppression.\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 3) {
        PrintUsage(argv[0]);
        return 1;
    }

    double voxel_size = 0.0;
    double dedup_eps = 0.0;
    bool skip_dedup = false;

    int idx = 1;
    while (idx < argc) {
        const std::string arg = argv[idx];

        if (arg == "--voxel") {
            if (idx + 1 >= argc) {
                std::cerr << "Error: missing value for --voxel" << std::endl;
                return 1;
            }
            try {
                voxel_size = std::stod(argv[idx + 1]);
            } catch (const std::exception&) {
                std::cerr << "Error: invalid value for --voxel: " << argv[idx + 1] << std::endl;
                return 1;
            }
            idx += 2;
            continue;
        }

        if (arg == "--dedup-eps") {
            if (idx + 1 >= argc) {
                std::cerr << "Error: missing value for --dedup-eps" << std::endl;
                return 1;
            }
            try {
                dedup_eps = std::stod(argv[idx + 1]);
            } catch (const std::exception&) {
                std::cerr << "Error: invalid value for --dedup-eps: " << argv[idx + 1] << std::endl;
                return 1;
            }
            idx += 2;
            continue;
        }

        if (arg == "--skip-dedup") {
            skip_dedup = true;
            ++idx;
            continue;
        }

        if (arg.rfind("--", 0) == 0) {
            std::cerr << "Error: unknown option: " << arg << std::endl;
            return 1;
        }

        break;
    }

    if (argc - idx != 2) {
        PrintUsage(argv[0]);
        return 1;
    }

    const std::string input_file = argv[idx];
    const std::string output_file = argv[idx + 1];

    return PostMergePointCloudFile(output_file, input_file, voxel_size, dedup_eps, skip_dedup);
}
