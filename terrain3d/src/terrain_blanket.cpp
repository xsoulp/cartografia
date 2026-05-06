#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <pcl/PolygonMesh.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fs = std::filesystem;

struct Config {
    fs::path input_path = "terrain3d/data/center_contours.xyz";
    fs::path output_cloud_path = "terrain3d/output/center_blanket_cloud.pcd";
    fs::path output_mesh_path = "terrain3d/output/center_blanket_mesh.ply";
    double grid_step = 1.0;
    double target_half = 500.0;
    int neighbors = 12;
    double power = 2.0;
};

struct SamplePoint {
    float x;
    float y;
    float z;
};

struct Bounds {
    double min_x;
    double min_y;
    double max_x;
    double max_y;
};

struct InputData {
    std::vector<SamplePoint> samples;
    Bounds target_bounds;
    bool has_target_bounds = false;
};

Config parse_args(int argc, char** argv) {
    Config config;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto read_value = [&](const std::string& flag) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for " + flag);
            }
            return argv[++i];
        };

        if (arg == "--input") {
            config.input_path = read_value(arg);
        } else if (arg == "--cloud") {
            config.output_cloud_path = read_value(arg);
        } else if (arg == "--mesh") {
            config.output_mesh_path = read_value(arg);
        } else if (arg == "--grid-step") {
            config.grid_step = std::stod(read_value(arg));
        } else if (arg == "--target-half") {
            config.target_half = std::stod(read_value(arg));
        } else if (arg == "--neighbors") {
            config.neighbors = std::stoi(read_value(arg));
        } else if (arg == "--power") {
            config.power = std::stod(read_value(arg));
        } else if (arg == "--help" || arg == "-h") {
            std::cout
                << "Usage: terrain_blanket [options]\n"
                << "  --input PATH       Input XYZ file (default: terrain3d/data/center_contours.xyz)\n"
                << "  --cloud PATH       Output PCD path (default: terrain3d/output/center_blanket_cloud.pcd)\n"
                << "  --mesh PATH        Output PLY path (default: terrain3d/output/center_blanket_mesh.ply)\n"
                << "  --grid-step VALUE  Grid spacing in meters (default: 1.0)\n"
                << "  --target-half N    Half-size of target patch in meters (default: 500)\n"
                << "  --neighbors N      Number of IDW neighbors (default: 12)\n"
                << "  --power VALUE      IDW power exponent (default: 2.0)\n";
            std::exit(0);
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }

    if (config.grid_step <= 0.0) {
        throw std::runtime_error("grid-step must be positive");
    }
    if (config.target_half <= 0.0) {
        throw std::runtime_error("target-half must be positive");
    }
    if (config.neighbors <= 0) {
        throw std::runtime_error("neighbors must be positive");
    }

    return config;
}

InputData load_samples(const fs::path& input_path) {
    std::ifstream input(input_path);
    if (!input) {
        throw std::runtime_error("Unable to open input file: " + input_path.string());
    }

    InputData data;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty()) {
            continue;
        }

        if (line[0] == '#') {
            if (line.rfind("# target_bbox_local ", 0) == 0) {
                std::istringstream header(line.substr(20));
                header >> data.target_bounds.min_x >> data.target_bounds.min_y
                       >> data.target_bounds.max_x >> data.target_bounds.max_y;
                if (!header) {
                    throw std::runtime_error("Failed to parse target bounds from " + input_path.string());
                }
                data.has_target_bounds = true;
            }
            continue;
        }

        std::istringstream row(line);
        SamplePoint point{};
        row >> point.x >> point.y >> point.z;
        if (!row) {
            continue;
        }
        data.samples.push_back(point);
    }

    if (data.samples.empty()) {
        throw std::runtime_error("No sample points were loaded from " + input_path.string());
    }
    return data;
}

double interpolate_idw(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& support_cloud,
    pcl::KdTreeFLANN<pcl::PointXYZ>& kd_tree,
    float x,
    float y,
    int neighbors,
    double power
) {
    const pcl::PointXYZ query(x, y, 0.0f);
    std::vector<int> indices(static_cast<std::size_t>(neighbors));
    std::vector<float> sq_distances(static_cast<std::size_t>(neighbors));
    const int found = kd_tree.nearestKSearch(query, neighbors, indices, sq_distances);
    if (found <= 0) {
        throw std::runtime_error("No support points found for interpolation");
    }

    double weighted_sum = 0.0;
    double weight_total = 0.0;
    for (int i = 0; i < found; ++i) {
        const float sq_distance = sq_distances[static_cast<std::size_t>(i)];
        if (sq_distance <= 1e-6f) {
            return support_cloud->points[static_cast<std::size_t>(indices[static_cast<std::size_t>(i)])].z;
        }

        const double distance = std::sqrt(static_cast<double>(sq_distance));
        const double weight = 1.0 / std::pow(distance, power);
        const double z = support_cloud->points[static_cast<std::size_t>(indices[static_cast<std::size_t>(i)])].z;
        weighted_sum += weight * z;
        weight_total += weight;
    }

    if (weight_total == 0.0) {
        throw std::runtime_error("Interpolation weights summed to zero");
    }
    return weighted_sum / weight_total;
}

int main(int argc, char** argv) {
    try {
        const Config config = parse_args(argc, argv);
        const InputData input_data = load_samples(config.input_path);
        const std::vector<SamplePoint>& samples = input_data.samples;

        auto support_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        support_cloud->reserve(samples.size());
        for (const SamplePoint& sample : samples) {
            support_cloud->push_back(pcl::PointXYZ(sample.x, sample.y, sample.z));
        }

        pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
        kd_tree.setInputCloud(support_cloud);

        Bounds target_bounds = {
            -config.target_half,
            -config.target_half,
            config.target_half,
            config.target_half,
        };
        if (input_data.has_target_bounds) {
            target_bounds = input_data.target_bounds;
        }

        const int width = static_cast<int>(
            std::round((target_bounds.max_x - target_bounds.min_x) / config.grid_step)
        ) + 1;
        const int height = static_cast<int>(
            std::round((target_bounds.max_y - target_bounds.min_y) / config.grid_step)
        ) + 1;
        auto blanket_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        blanket_cloud->width = static_cast<std::uint32_t>(width);
        blanket_cloud->height = static_cast<std::uint32_t>(height);
        blanket_cloud->is_dense = true;
        blanket_cloud->points.resize(static_cast<std::size_t>(width * height));

        float min_z = std::numeric_limits<float>::max();
        float max_z = std::numeric_limits<float>::lowest();
        for (int row = 0; row < height; ++row) {
            const float y = static_cast<float>(target_bounds.min_y + row * config.grid_step);
            for (int col = 0; col < width; ++col) {
                const float x = static_cast<float>(target_bounds.min_x + col * config.grid_step);
                const float z = static_cast<float>(
                    interpolate_idw(support_cloud, kd_tree, x, y, config.neighbors, config.power)
                );
                blanket_cloud->at(static_cast<std::size_t>(col), static_cast<std::size_t>(row)) =
                    pcl::PointXYZ(x, y, z);
                min_z = std::min(min_z, z);
                max_z = std::max(max_z, z);
            }
        }

        fs::create_directories(config.output_cloud_path.parent_path());
        fs::create_directories(config.output_mesh_path.parent_path());

        if (pcl::io::savePCDFileBinary(config.output_cloud_path.string(), *blanket_cloud) != 0) {
            throw std::runtime_error("Failed to write cloud: " + config.output_cloud_path.string());
        }

        pcl::PolygonMesh mesh;
        pcl::toPCLPointCloud2(*blanket_cloud, mesh.cloud);
        mesh.polygons.reserve(static_cast<std::size_t>((width - 1) * (height - 1) * 2));
        for (int row = 0; row < height - 1; ++row) {
            for (int col = 0; col < width - 1; ++col) {
                const int top_left = row * width + col;
                const int top_right = top_left + 1;
                const int bottom_left = top_left + width;
                const int bottom_right = bottom_left + 1;

                pcl::Vertices tri_a;
                tri_a.vertices = {top_left, bottom_left, top_right};
                mesh.polygons.push_back(tri_a);

                pcl::Vertices tri_b;
                tri_b.vertices = {top_right, bottom_left, bottom_right};
                mesh.polygons.push_back(tri_b);
            }
        }

        if (pcl::io::savePLYFileBinary(config.output_mesh_path.string(), mesh) != 0) {
            throw std::runtime_error("Failed to write mesh: " + config.output_mesh_path.string());
        }

        std::cout << std::fixed << std::setprecision(2)
                  << "Loaded " << samples.size() << " contour support samples\n"
                  << "Generated blanket grid " << width << "x" << height
                  << " over local bounds [" << target_bounds.min_x << ", " << target_bounds.min_y
                  << "] to [" << target_bounds.max_x << ", " << target_bounds.max_y << "]"
                  << " with z-range [" << min_z << ", " << max_z << "]\n"
                  << "Cloud written to: " << config.output_cloud_path << "\n"
                  << "Mesh written to: " << config.output_mesh_path << "\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "terrain_blanket failed: " << error.what() << '\n';
        return 1;
    }
}
