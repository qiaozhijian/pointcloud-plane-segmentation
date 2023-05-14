//
// Created by qzj on 23-5-12.
//
#ifndef O3DAPP_UTLIS_H
#define O3DAPP_UTLIS_H
#include <chrono>
#include <limits>
#include <memory>
#include <iostream>
#include <set>
#include <vector>
#include <Eigen/Dense>
#include <open3d/Open3D.h>

using namespace open3d;
namespace rspd{
    inline std::shared_ptr<geometry::TriangleMesh> makePlane(
            const Eigen::Vector3d& center, const Eigen::Vector3d& normal,
            const Eigen::Vector3d& basisU, const Eigen::Vector3d& basisV)
    {
        auto mesh_ptr = std::make_shared<geometry::TriangleMesh>();
        //std::cout << "center: " << center.transpose() << std::endl;
        //std::cout << "normal: " << normal.transpose() << std::endl;
        //std::cout << "basisU: " << basisU.transpose() << ", norm: " << basisU.norm() << std::endl;
        //std::cout << "basisV: " << basisV.transpose() << ", norm: " << basisV.norm() << std::endl;
        mesh_ptr->vertices_.resize(8);
        mesh_ptr->vertices_[0] = center - basisU - basisV;
        mesh_ptr->vertices_[1] = center - basisU + basisV;
        mesh_ptr->vertices_[2] = center + basisU - basisV;
        mesh_ptr->vertices_[3] = center + basisU + basisV;

        mesh_ptr->vertices_[4] = center + 0.01*normal - basisU - basisV;
        mesh_ptr->vertices_[5] = center + 0.01*normal - basisU + basisV;
        mesh_ptr->vertices_[6] = center + 0.01*normal + basisU - basisV;
        mesh_ptr->vertices_[7] = center + 0.01*normal + basisU + basisV;
        mesh_ptr->triangles_.push_back(Eigen::Vector3i(4, 7, 5));
        mesh_ptr->triangles_.push_back(Eigen::Vector3i(4, 6, 7));
        mesh_ptr->triangles_.push_back(Eigen::Vector3i(0, 2, 4));
        mesh_ptr->triangles_.push_back(Eigen::Vector3i(2, 6, 4));
        mesh_ptr->triangles_.push_back(Eigen::Vector3i(0, 1, 2));
        mesh_ptr->triangles_.push_back(Eigen::Vector3i(1, 3, 2));
        mesh_ptr->triangles_.push_back(Eigen::Vector3i(1, 5, 7));
        mesh_ptr->triangles_.push_back(Eigen::Vector3i(1, 7, 3));
        mesh_ptr->triangles_.push_back(Eigen::Vector3i(2, 3, 7));
        mesh_ptr->triangles_.push_back(Eigen::Vector3i(2, 7, 6));
        mesh_ptr->triangles_.push_back(Eigen::Vector3i(0, 4, 1));
        mesh_ptr->triangles_.push_back(Eigen::Vector3i(1, 4, 5));
        return mesh_ptr;
    }
}
#endif //O3DAPP_UTLIS_H
