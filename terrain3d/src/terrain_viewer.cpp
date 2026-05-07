#include <chrono>
#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <pcl/PolygonMesh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkObject.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>

namespace fs = std::filesystem;

using Edge = std::pair<int, int>;

struct Config {
    fs::path cloud_path = "terrain3d/output/center_blanket_cloud.pcd";
    fs::path mesh_path = "terrain3d/output/center_blanket_mesh.ply";
    fs::path constructions_path;
    fs::path actual_constructions_path = "terrain3d/output/construction_artifacts/constructions_actual_height.obj";
    fs::path assumed_constructions_path = "terrain3d/output/construction_artifacts/constructions_assumed_6m.obj";
    bool show_cloud = false;
    bool show_mesh = true;
    bool show_constructions = true;
    bool use_grouped_constructions = true;
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
        } else if (arg == "--show-cloud") {
            config.show_cloud = true;
        } else if (arg == "--mesh") {
            config.mesh_path = read_value(arg);
        } else if (arg == "--constructions") {
            config.constructions_path = read_value(arg);
            config.use_grouped_constructions = false;
        } else if (arg == "--actual-constructions") {
            config.actual_constructions_path = read_value(arg);
        } else if (arg == "--assumed-constructions") {
            config.assumed_constructions_path = read_value(arg);
        } else if (arg == "--no-constructions") {
            config.show_constructions = false;
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
                << "  --show-cloud      Show the terrain point cloud together with other layers\n"
                << "  --mesh PATH       Input PLY path (default: terrain3d/output/center_blanket_mesh.ply)\n"
                << "  --constructions PATH\n"
                << "                    Input OBJ path for one construction artifact layer\n"
                << "  --actual-constructions PATH\n"
                << "                    Input OBJ path for actual-height construction artifacts\n"
                << "  --assumed-constructions PATH\n"
                << "                    Input OBJ path for assumed-height construction artifacts\n"
                << "  --no-constructions\n"
                << "                    Do not show construction artifacts\n"
                << "  --mesh-only       Show only the terrain mesh\n"
                << "  --cloud-only      Show only the terrain point cloud\n";
            std::exit(0);
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }
    return config;
}

std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> load_obj_edges(const fs::path& obj_path) {
    std::ifstream input(obj_path);
    if (!input) {
        throw std::runtime_error("Unable to open OBJ for wireframe edges: " + obj_path.string());
    }

    std::vector<pcl::PointXYZ> vertices;
    std::set<Edge> unique_edges;
    std::string line;
    while (std::getline(input, line)) {
        std::istringstream row(line);
        std::string tag;
        row >> tag;
        if (tag == "v") {
            pcl::PointXYZ point;
            row >> point.x >> point.y >> point.z;
            if (row) {
                vertices.push_back(point);
            }
        } else if (tag == "f") {
            std::vector<int> face_indices;
            std::string token;
            while (row >> token) {
                const std::size_t slash = token.find('/');
                const std::string index_token = token.substr(0, slash);
                if (index_token.empty()) {
                    continue;
                }
                int vertex_index = std::stoi(index_token);
                if (vertex_index < 0) {
                    vertex_index = static_cast<int>(vertices.size()) + vertex_index + 1;
                }
                if (vertex_index > 0) {
                    face_indices.push_back(vertex_index - 1);
                }
            }

            for (std::size_t i = 0; i < face_indices.size(); ++i) {
                const int a = face_indices[i];
                const int b = face_indices[(i + 1) % face_indices.size()];
                if (
                    a >= 0 &&
                    b >= 0 &&
                    a < static_cast<int>(vertices.size()) &&
                    b < static_cast<int>(vertices.size()) &&
                    a != b
                ) {
                    unique_edges.insert(std::minmax(a, b));
                }
            }
        }
    }

    std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> edges;
    edges.reserve(unique_edges.size());
    for (const Edge& edge : unique_edges) {
        edges.push_back({vertices[static_cast<std::size_t>(edge.first)], vertices[static_cast<std::size_t>(edge.second)]});
    }
    return edges;
}

void add_construction_layer(
    const pcl::visualization::PCLVisualizer::Ptr& viewer,
    const fs::path& obj_path,
    const std::string& layer_id
) {
    if (!fs::exists(obj_path)) {
        throw std::runtime_error("Construction OBJ not found: " + obj_path.string());
    }

    std::ifstream input(obj_path);
    if (!input) {
        throw std::runtime_error("Failed to load construction OBJ: " + obj_path.string());
    }

    auto points = vtkSmartPointer<vtkPoints>::New();
    auto polygons = vtkSmartPointer<vtkCellArray>::New();
    auto colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetName("building_colors");
    colors->SetNumberOfComponents(3);

    auto color_for_object = [](const std::string& object_name) {
        const std::size_t hash = std::hash<std::string>{}(object_name);
        unsigned char color[3] = {
            static_cast<unsigned char>(70 + (hash & 0x7f)),
            static_cast<unsigned char>(70 + ((hash >> 8) & 0x7f)),
            static_cast<unsigned char>(70 + ((hash >> 16) & 0x7f)),
        };
        return std::array<unsigned char, 3>{color[0], color[1], color[2]};
    };

    std::string current_object = "construction";
    std::array<unsigned char, 3> current_color = color_for_object(current_object);
    std::size_t object_count = 0;
    std::size_t face_count = 0;

    std::string line;
    while (std::getline(input, line)) {
        std::istringstream row(line);
        std::string tag;
        row >> tag;
        if (tag == "o") {
            row >> current_object;
            current_color = color_for_object(current_object);
            ++object_count;
        } else if (tag == "v") {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            row >> x >> y >> z;
            if (row) {
                points->InsertNextPoint(x, y, z);
            }
        } else if (tag == "f") {
            std::vector<vtkIdType> face_indices;
            std::string token;
            while (row >> token) {
                const std::size_t slash = token.find('/');
                const std::string index_token = token.substr(0, slash);
                if (index_token.empty()) {
                    continue;
                }
                vtkIdType vertex_index = static_cast<vtkIdType>(std::stoll(index_token));
                if (vertex_index < 0) {
                    vertex_index = points->GetNumberOfPoints() + vertex_index + 1;
                }
                if (vertex_index > 0 && vertex_index <= points->GetNumberOfPoints()) {
                    face_indices.push_back(vertex_index - 1);
                }
            }

            if (face_indices.size() >= 3) {
                polygons->InsertNextCell(static_cast<vtkIdType>(face_indices.size()));
                for (const vtkIdType index : face_indices) {
                    polygons->InsertCellPoint(index);
                }
                colors->InsertNextTypedTuple(current_color.data());
                ++face_count;
            }
        }
    }

    auto polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(points);
    polydata->SetPolys(polygons);
    polydata->GetCellData()->SetScalars(colors);

    if (points->GetNumberOfPoints() == 0 || face_count == 0) {
        throw std::runtime_error("No construction geometry was loaded from: " + obj_path.string());
    }

    std::cout
        << "Loaded " << layer_id << " construction artifacts with "
        << object_count << " objects, "
        << points->GetNumberOfPoints()
        << " vertices and " << face_count << " faces.\n";

    if (!viewer->addModelFromPolyData(polydata, layer_id)) {
        throw std::runtime_error("PCL could not attach construction layer: " + layer_id);
    }
}

int main(int argc, char** argv) {
    try {
        const Config config = parse_args(argc, argv);

        // PCL 1.14 still triggers a VTK deprecation warning during viewer setup.
        // Suppress VTK's global warning stream so the viewer starts cleanly.
        vtkObject::GlobalWarningDisplayOff();

        auto viewer = pcl::make_shared<pcl::visualization::PCLVisualizer>("Terrain Viewer");
        viewer->setBackgroundColor(0.68, 0.84, 1.0);
        viewer->addCoordinateSystem(10.0);
        viewer->initCameraParameters();

        bool added_anything = false;
        pcl::PolygonMesh terrain_mesh;
        bool terrain_mesh_loaded = false;
        bool terrain_mesh_visible = false;

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

            if (pcl::io::loadPLYFile(config.mesh_path.string(), terrain_mesh) != 0) {
                throw std::runtime_error("Failed to load mesh: " + config.mesh_path.string());
            }
            terrain_mesh_loaded = true;

            std::cout
                << "Loaded mesh with " << terrain_mesh.cloud.width * terrain_mesh.cloud.height
                << " vertices and " << terrain_mesh.polygons.size() << " faces.\n";

            const bool added_mesh = viewer->addPolygonMesh(terrain_mesh, "terrain_mesh");
            if (added_mesh) {
                terrain_mesh_visible = true;
                viewer->setRepresentationToSurfaceForAllActors();
                added_anything = true;
            } else {
                std::cerr
                    << "PCL could not attach the mesh actor. "
                    << "Run without --mesh-only to view the terrain cloud fallback.\n";
            }
        }

        if (config.show_constructions) {
            if (config.use_grouped_constructions) {
                add_construction_layer(
                    viewer,
                    config.assumed_constructions_path,
                    "assumed_6m_constructions"
                );
                add_construction_layer(
                    viewer,
                    config.actual_constructions_path,
                    "actual_height_constructions"
                );
            } else {
                add_construction_layer(
                    viewer,
                    config.constructions_path,
                    "construction_artifacts"
                );
            }
            added_anything = true;
        }

        if (!added_anything) {
            throw std::runtime_error("Nothing to display. Enable the cloud and/or mesh.");
        }

        viewer->registerKeyboardCallback(
            [&](const pcl::visualization::KeyboardEvent& event) {
                if (!event.keyDown() || event.getKeySym() != "s" || !terrain_mesh_loaded) {
                    return;
                }

                if (terrain_mesh_visible) {
                    viewer->removePolygonMesh("terrain_mesh");
                    terrain_mesh_visible = false;
                    std::cout << "Blanket surface hidden. Press 's' to show it.\n";
                } else if (viewer->addPolygonMesh(terrain_mesh, "terrain_mesh")) {
                    terrain_mesh_visible = true;
                    viewer->setRepresentationToSurfaceForAllActors();
                    std::cout << "Blanket surface shown. Press 's' to hide it.\n";
                } else {
                    std::cerr << "Could not restore terrain mesh actor.\n";
                }
            }
        );

        viewer->resetCamera();
        viewer->setCameraPosition(
            0.0, -8500.0, 3200.0,
            0.0, 0.0, 60.0,
            0.0, 0.0, 1.0
        );
        std::cout
            << "Opening terrain viewer window.\n"
            << "Controls: drag to orbit, mouse wheel to zoom, right drag to pan.\n"
            << "Shortcut: press 's' to hide/show the blanket surface.\n";

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
