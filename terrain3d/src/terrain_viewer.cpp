#include <chrono>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <pcl/PolygonMesh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkObject.h>

namespace fs = std::filesystem;

struct Config {
    fs::path cloud_path = "terrain3d/output/center_blanket_cloud.pcd";
    fs::path mesh_path = "terrain3d/output/center_blanket_mesh.ply";
    bool show_cloud = true;
    bool show_mesh = true;
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

        if (arg == "--cloud") {
            config.cloud_path = read_value(arg);
        } else if (arg == "--mesh") {
            config.mesh_path = read_value(arg);
        } else if (arg == "--mesh-only") {
            config.show_cloud = false;
            config.show_mesh = true;
        } else if (arg == "--cloud-only") {
            config.show_cloud = true;
            config.show_mesh = false;
        } else if (arg == "--help" || arg == "-h") {
            std::cout
                << "Usage: terrain_viewer [options]\n"
                << "  --cloud PATH      Input PCD path (default: terrain3d/output/center_blanket_cloud.pcd)\n"
                << "  --mesh PATH       Input PLY path (default: terrain3d/output/center_blanket_mesh.ply)\n"
                << "  --mesh-only       Show only the terrain mesh\n"
                << "  --cloud-only      Show only the terrain point cloud\n";
            std::exit(0);
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }
    return config;
}

int main(int argc, char** argv) {
    try {
        const Config config = parse_args(argc, argv);

        // PCL 1.14 still triggers a VTK deprecation warning during viewer setup.
        // Suppress VTK's global warning stream so the viewer starts cleanly.
        vtkObject::GlobalWarningDisplayOff();

        auto viewer = pcl::make_shared<pcl::visualization::PCLVisualizer>("Terrain Viewer");
        viewer->setBackgroundColor(0.08, 0.08, 0.1);
        viewer->addCoordinateSystem(10.0);
        viewer->initCameraParameters();

        bool added_anything = false;

        if (config.show_cloud) {
            if (!fs::exists(config.cloud_path)) {
                throw std::runtime_error("Cloud file not found: " + config.cloud_path.string());
            }

            auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            if (pcl::io::loadPCDFile(config.cloud_path.string(), *cloud) != 0) {
                throw std::runtime_error("Failed to load cloud: " + config.cloud_path.string());
            }

            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, 235, 210, 120);
            viewer->addPointCloud<pcl::PointXYZ>(cloud, color, "terrain_cloud");
            viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "terrain_cloud"
            );
            added_anything = true;
        }

        if (config.show_mesh) {
            if (!fs::exists(config.mesh_path)) {
                throw std::runtime_error("Mesh file not found: " + config.mesh_path.string());
            }

            pcl::PolygonMesh mesh;
            if (pcl::io::loadPLYFile(config.mesh_path.string(), mesh) != 0) {
                throw std::runtime_error("Failed to load mesh: " + config.mesh_path.string());
            }

            viewer->addPolygonMesh(mesh, "terrain_mesh");
            viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_COLOR, 0.75, 0.67, 0.42, "terrain_mesh"
            );
            viewer->setRepresentationToSurfaceForAllActors();
            added_anything = true;
        }

        if (!added_anything) {
            throw std::runtime_error("Nothing to display. Enable the cloud and/or mesh.");
        }

        viewer->resetCamera();
        std::cout
            << "Opening terrain viewer window.\n"
            << "Controls: drag to orbit, mouse wheel to zoom, right drag to pan.\n";

        while (!viewer->wasStopped()) {
            viewer->spinOnce(16);
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "terrain_viewer failed: " << error.what() << '\n';
        return 1;
    }
}
