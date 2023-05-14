#include <rspd/planedetector.h>
#include "utils.h"
#include "patchwork/patchwork.hpp"

using namespace rspd;
// ----------------------------------------------------------------------------
void ReadBinOpen3d(const std::string& filename, open3d::geometry::PointCloud &cloud);
pcl::PointCloud<pcl::PointXYZI>::Ptr getCloud(std::string filename);

int main(int argc, char *argv[]) {

    // utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
    if (argc < 2) {
        utility::LogInfo("Open3D {}", OPEN3D_VERSION);
        utility::LogInfo("Usage:");
        utility::LogInfo("    > TestVisualizer [filename]");
        return 1;
    }

    auto cloud_ptr = std::make_shared<open3d::geometry::PointCloud>();
    //ReadBinOpen3d(argv[1], *cloud_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloudI = getCloud(argv[1]);
    pcl::PointCloud<pcl::PointXYZI> srcGround;
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrSrcNonground(new pcl::PointCloud<pcl::PointXYZI>);
    double tSrc = 0.0;
    PatchWork<pcl::PointXYZI> patchwork("KITTI");
    patchwork.estimate_ground(*ptrCloudI, srcGround, *ptrSrcNonground, tSrc);

    cloud_ptr->points_.resize(ptrSrcNonground->size());
    for (int i = 0; i < ptrSrcNonground->size(); i++) {
        auto &pt = cloud_ptr->points_[i];
        pt.x() = ptrSrcNonground->points[i].x;
        pt.y() = ptrSrcNonground->points[i].y;
        pt.z() = ptrSrcNonground->points[i].z;
    }

    std::set<Plane*> planes = rspd::planeSegmentation(cloud_ptr);

    // Visualization
    // create a vector of geometries to visualize, starting with input point cloud
    std::vector<std::shared_ptr<const geometry::Geometry>> geometries;
    geometries.reserve(1 + planes.size());
    geometries.push_back(cloud_ptr);

    // Colors (default MATLAB colors)
    std::vector<Eigen::Vector3d> colors;
    colors.push_back(Eigen::Vector3d(0.8500, 0.3250, 0.0980));
    colors.push_back(Eigen::Vector3d(0.9290, 0.6940, 0.1250));
    colors.push_back(Eigen::Vector3d(0.4940, 0.1840, 0.5560));
    colors.push_back(Eigen::Vector3d(0.4660, 0.6740, 0.1880));
    colors.push_back(Eigen::Vector3d(0.3010, 0.7450, 0.9330));
    colors.push_back(Eigen::Vector3d(0.6350, 0.0780, 0.1840));
    colors.push_back(Eigen::Vector3d(0.75, 0.75, 0.75));
    colors.push_back(Eigen::Vector3d(0, 0.4470, 0.7410));
    colors.push_back(Eigen::Vector3d(0.1250, 0.6940, 0.1250));
    colors.push_back(Eigen::Vector3d(0.75, 0.75, 0));
    colors.push_back(Eigen::Vector3d(0.75, 0, 0.75));
    colors.push_back(Eigen::Vector3d(0, 0.75, 0.75));
    colors.push_back(Eigen::Vector3d(0.25, 0.25, 0.25));

    // add any planes
    size_t i = 0;
    for (const auto& p : planes) {
        auto pviz = makePlane(p->center(), p->normal(), p->basisU(), p->basisV());
        pviz->PaintUniformColor(colors[i%colors.size()]);
        geometries.push_back(pviz);

        ++i;
    }

    visualization::DrawGeometries(geometries, "Points and Planes", 1600, 900);

    // visualization::VisualizerWithVertexSelection visualizer;
    // visualizer.CreateVisualizerWindow("Plane Selection: Select Point Close to Desired Plane", 1600, 900);
    // visualizer.AddGeometry(cloud_ptr);
    // visualizer.Run();
    // visualizer.DestroyVisualizerWindow();
    // const auto pts = visualizer.GetPickedPoints();

    // for (const auto& pt : pts) {
    //     double d = std::numeric_limits<double>::max();
    //     Plane* closest_plane;
    //     for (const auto& p : planes) {
    //         if (std::abs(p->getSignedDistanceFromSurface(pt.coord.cast<float>())) < d) {
    //             d = std::abs(p->getSignedDistanceFromSurface(pt.coord.cast<float>()));
    //             closest_plane = p;
    //         }
    //     }

    //     if (closest_plane == nullptr) {
    //         std::cout << "Could not find closest plane to selected point!" << std::endl;
    //     } else {
    //         std::cout << closest_plane->normal().transpose() << " ";
    //         std::cout << closest_plane->distanceFromOrigin() << "\t";
    //         std::cout << closest_plane->center().transpose() << "\t";
    //         std::cout << closest_plane->basisU().transpose() << "\t";
    //         std::cout << closest_plane->basisV().transpose() << std::endl;
    //     }
    // }

    // utility::LogInfo("End of the test.\n");

    return 0;
}

void ReadBinOpen3d(const std::string& filename, open3d::geometry::PointCloud &cloud) {
    FILE *file = fopen(filename.c_str(), "rb");
    if (!file) {
        std::cerr << "error: failed to load point cloud " << filename << std::endl;
        return;
    }
    // check whether the suffix of the file is bin
    std::string suffix = filename.substr(filename.find_last_of('.') + 1);
    if (suffix != "bin") {
        std::cerr << "error: failed to load point cloud " << filename << std::endl;
        return;
    }

    std::vector<float> buffer(1000000);
    size_t num_points =
            fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
    fclose(file);
    cloud.points_.resize(num_points);
    for (int i = 0; i < num_points; i++) {
        auto &pt = cloud.points_[i];
        pt.x() = buffer[i * 4];
        pt.y() = buffer[i * 4 + 1];
        pt.z() = buffer[i * 4 + 2];
        std::cout << pt.x() << " " << pt.y() << " " << pt.z() << " " << buffer[i * 4] << " " << buffer[i * 4 + 1]
                  << " " << buffer[i * 4 + 2] << std::endl;
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr getCloud(std::string filename) {
    FILE *file = fopen(filename.c_str(), "rb");
    if (!file) {
        std::cerr << "error: failed to load point cloud " << filename << std::endl;
        return nullptr;
    }

    std::vector<float> buffer(1000000);
    size_t num_points =
            fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
    fclose(file);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    cloud->resize(num_points);

    for (int i = 0; i < num_points; i++) {
        auto &pt = cloud->at(i);
        pt.x = buffer[i * 4];
        pt.y = buffer[i * 4 + 1];
        pt.z = buffer[i * 4 + 2];
        pt.intensity = buffer[i * 4 + 3];
    }

    return cloud;
}