#ifndef PLANEDETECTOR_H
#define PLANEDETECTOR_H

#include "planarpatch.h"
#include "boundaryvolumehierarchy.h"
#include "rspd/unionfind.h"
#include <chrono>
#include <iostream>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <open3d/Open3D.h>
#include <pcl/io/pcd_io.h>

class PlaneDetector
{

#ifdef DEBUG
#   define IF_DEBUG( ... )   __VA_ARGS__
#else
#   define IF_DEBUG( ... )
#endif /* DEBUG */
public:
    using PointCloudConstPtr = std::shared_ptr<const open3d::geometry::PointCloud>;

    PlaneDetector(const PointCloudConstPtr& pointCloud, std::vector<std::vector<int>>& neighbors,
                  double minNormalDiff = 60.0, double maxDist = 75.0,
                  double outlierRatio = 0.75, double minPlaneEdgeLength = 1.0, size_t minNumPoints = 30)
            : mPointCloud(pointCloud)
    {

        mMinNormalDiff = std::cos(minNormalDiff * M_PI / 180);
        mMaxDist = std::cos(maxDist * M_PI / 180);
        mOutlierRatio = outlierRatio;
        mMinNumPoints = minNumPoints;
        m_min_plane_edge_length = minPlaneEdgeLength;

        mNeighbors.swap(neighbors);

        mBottomLeft = pointCloud->GetMinBound();
        mTopRight = pointCloud->GetMaxBound();
        mExtCenter = (mBottomLeft + mTopRight) / 2;

        double maxSize = 0;
        for (size_t dim = 0; dim<3; dim++) {
            maxSize = std::max(maxSize, mTopRight(dim) - mBottomLeft(dim));
        }
        mMaxSize = maxSize;
    }

    double minNormalDiff() const
    {
        return mMinNormalDiff;
    }

    void minNormalDiff(double minNormalDiff)
    {
        mMinNormalDiff = minNormalDiff;
    }

    double maxDist() const
    {
        return mMaxDist;
    }

    void maxDist(double maxDist)
    {
        mMaxDist = maxDist;
    }

    double outlierRatio() const
    {
        return mOutlierRatio;
    }

    void outlierRatio(double outlierRatio)
    {
        mOutlierRatio = outlierRatio;
    }

    std::set<Plane*> detect()
    {
        IF_DEBUG(
                double timeDetectPatches = 0;
        double timeMerge = 0;
        double timeGrowth = 0;
        double timeRelaxedGrowth = 0;
        double timeUpdate = 0;
        double timeDelimit = 0;
        )

        IF_DEBUG(
                auto t1 = std::chrono::high_resolution_clock::now();
        )
        StatisticsUtils statistics(pointCloud()->points_.size());
        BVH3d octree(pointCloud());
        std::vector<PlanarPatch*> patches;
        detectPlanarPatches(&octree, &statistics, patches);
        IF_DEBUG(
                timeDetectPatches += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t1).count();
        )

        mPatchPoints = std::vector<PlanarPatch*>(pointCloud()->points_.size(), NULL);
        for (PlanarPatch *patch : patches)
        {
            for (const size_t &point : patch->points())
            {
                mPatchPoints[point] = patch;
            }
        }

        IF_DEBUG(
                std::cout << "#" << patches.size() << std::endl;
        )
        bool changed = false;
        do
        {
            IF_DEBUG(
                    t1 = std::chrono::high_resolution_clock::now();
            )
            growPatches(patches);
            IF_DEBUG(
                    timeGrowth += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t1).count();
            t1 = std::chrono::high_resolution_clock::now();
            )
            mergePatches(patches);
            IF_DEBUG(
                    timeMerge += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t1).count();
            std::cout << "#" << patches.size() << std::endl;
            t1 = std::chrono::high_resolution_clock::now();
            )
            changed = updatePatches(patches);
            IF_DEBUG(
                    timeUpdate += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t1).count();
            )
        } while (changed);

        IF_DEBUG(
                t1 = std::chrono::high_resolution_clock::now();
        )
        growPatches(patches, true);
        IF_DEBUG(
                timeRelaxedGrowth += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t1).count();
        t1 = std::chrono::high_resolution_clock::now();
        )
        std::vector<PlanarPatch*> truePositivePatches;
        for (PlanarPatch *patch : patches)
        {
            delimitPlane(patch);
            if (isFalsePositive(patch))
            {
                delete patch;
            }
            else
            {
                truePositivePatches.push_back(patch);
            }
        }

        patches = truePositivePatches;
        IF_DEBUG(
                std::cout << "#" << patches.size() << std::endl;
        timeDelimit += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t1).count();
        )
        std::set<Plane*> planes;
        for (PlanarPatch *patch : patches)
        {
            Plane *plane = new Plane(patch->plane());
            plane->inliers(patch->points());
            planes.insert(plane);
            delete patch;
        }

        IF_DEBUG(
                std::cout << "#" << planes.size() << std::endl;
        std::cout << "Detect planar patches - Time elapsed: " << timeDetectPatches << "s" <<  std::endl;
        std::cout << "Merge patches - Time elapsed: " << timeMerge << "s" <<  std::endl;
        std::cout << "Grow patches - Time elapsed: " << timeGrowth << "s" <<  std::endl;
        std::cout << "Relaxed Grow patches - Time elapsed: " << timeRelaxedGrowth << "s" <<  std::endl;
        std::cout << "Update - Time elapsed: " << timeUpdate << "s" <<  std::endl;
        std::cout << "Total time elapsed: " << timeDetectPatches + timeMerge + timeGrowth + timeRelaxedGrowth + timeUpdate << "s" << std::endl;
        std::cout << "Delimit planes - Time elapsed: " << timeDelimit << "s" <<  std::endl;
        )
        return planes;
    }

    const PointCloudConstPtr& pointCloud() const
    {
        return mPointCloud;
    }

private:
    PointCloudConstPtr mPointCloud;
    std::vector<std::vector<int>> mNeighbors;

    Eigen::Vector3d mBottomLeft;
    Eigen::Vector3d mTopRight;
    Eigen::Vector3d mExtCenter;
    double mMaxSize; // pointcloud rect extension max size

    std::vector<PlanarPatch*> mPatchPoints;
    double mMinNormalDiff;
    double mMaxDist;
    double mOutlierRatio;
    size_t mMinNumPoints;
    double m_min_plane_edge_length;

    bool detectPlanarPatches(BVH3d *node, StatisticsUtils *statistics, std::vector<PlanarPatch*> &patches)
    {
        if (node->numPoints() < mMinNumPoints) return false;
        node->partition(1, mMinNumPoints);
        bool iHavePlanarPatch = false;
        bool childCouldHavePlanarPatch = false;
        for (size_t i = 0; i < 8; i++)
        {
            if (node->child(i) != NULL && detectPlanarPatches(node->child(i), statistics, patches))
            {
                childCouldHavePlanarPatch = true;
            }
        }
        if (!childCouldHavePlanarPatch && node->level() > 2)
        {
            PlanarPatch *patch = new PlanarPatch(pointCloud(), statistics, node->points(), mMinNormalDiff, mMaxDist, mOutlierRatio);
            if (patch->isPlanar())
            {
                patches.push_back(patch);
                iHavePlanarPatch = true;
            }
            else
            {
                delete patch;
            }
        }
        return iHavePlanarPatch || childCouldHavePlanarPatch;
    }

    void growPatches(std::vector<PlanarPatch*> &patches, bool relaxed = false)
    {
        std::sort(patches.begin(), patches.end(), [](const PlanarPatch *a, const PlanarPatch *b) {
            return a->minNormalDiff() > b->minNormalDiff();
        });
        std::queue<size_t> queue;
        for (PlanarPatch *patch : patches)
        {
            if (patch->stable()) continue;
            for (const size_t &point : patch->points())
            {
                queue.push(point);
            }
            while (!queue.empty())
            {
                size_t point = queue.front();
                queue.pop();
                for (int neighbor : mNeighbors[point])
                {
                    if (mPatchPoints[neighbor] != NULL || (!relaxed && patch->isVisited(neighbor))) continue;
                    if ((!relaxed && patch->isInlier(neighbor)) || (relaxed && std::abs(patch->plane().getSignedDistanceFromSurface(pointCloud()->points_[neighbor])) < patch->maxDistPlane()))
                    {
                        queue.push(neighbor);
                        patch->addPoint(neighbor);
                        mPatchPoints[neighbor] = patch;
                    }
                    else
                    {
                        patch->visit(neighbor);
                    }
                }
            }
        }
    }


    void mergePatches(std::vector<PlanarPatch*> &patches)
    {
        size_t n = patches.size();
        for (size_t i = 0; i < n; i++)
        {
            patches[i]->index(i);
        }
        std::vector<bool> graph(n * n, false);
        std::vector<bool> disconnectedPatches(n * n, false);
        for (size_t i = 0; i < patches.size(); i++)
        {
            for (size_t j = i + 1; j < patches.size(); j++)
            {
                double normalThreshold = std::min(patches[i]->minNormalDiff(), patches[j]->minNormalDiff());
                disconnectedPatches[i * n + j] = std::abs(patches[i]->plane().normal().dot(patches[j]->plane().normal())) < normalThreshold;
                disconnectedPatches[j * n + i] = disconnectedPatches[i * n + j];
            }
        }
        for (PlanarPatch *p : patches)
        {
            for (const size_t &point : p->points())
            {
                for (int neighbor : mNeighbors[point])
                {
                    PlanarPatch *np = mPatchPoints[neighbor];
                    if (p == np || np == NULL || graph[np->index() * n + p->index()] || graph[p->index() * n + np->index()] ||
                        disconnectedPatches[p->index() * n + np->index()] || p->isVisited(neighbor) || np->isVisited(point)) continue;
                    p->visit(neighbor);
                    np->visit(point);
                    const Eigen::Vector3d &p1 = pointCloud()->points_[point];
                    const Eigen::Vector3d &n1 = pointCloud()->normals_[point];
                    const Eigen::Vector3d &p2 = pointCloud()->points_[neighbor];
                    const Eigen::Vector3d &n2 = pointCloud()->normals_[neighbor];
                    double distThreshold = std::max(p->maxDistPlane(), np->maxDistPlane());
                    double normalThreshold = std::min(p->minNormalDiff(), np->minNormalDiff());
                    graph[p->index() * n + np->index()] = std::abs(p->plane().normal().dot(n2)) > normalThreshold &&
                                                          std::abs(np->plane().normal().dot(n1)) > normalThreshold &&
                                                          std::abs(p->plane().getSignedDistanceFromSurface(p2)) < distThreshold &&
                                                          std::abs(np->plane().getSignedDistanceFromSurface(p1)) < distThreshold;
                }
            }
        }
        UnionFind uf(patches.size());
        for (size_t i = 0; i < patches.size(); i++)
        {
            for (size_t j = i + 1; j < patches.size(); j++)
            {
                if (graph[i * n + j] || graph[j * n + i])
                {
                    uf.join(i, j);
                }
            }
        }
        std::vector<size_t> largestPatch(patches.size());
        std::iota(largestPatch.begin(), largestPatch.end(), 0);
        for (size_t i = 0; i < patches.size(); i++)
        {
            int root = uf.root(i);
            if (patches[largestPatch[root]]->points().size() < patches[i]->points().size())
            {
                largestPatch[root] = i;
            }
        }
        for (size_t i = 0; i < patches.size(); i++)
        {
            size_t root = largestPatch[uf.root(i)];
            if (root != i)
            {
                for (const size_t &point : patches[i]->points())
                {
                    patches[root]->addPoint(point);
                    mPatchPoints[point] = patches[root];
                }
                patches[root]->maxDistPlane(std::max(patches[root]->maxDistPlane(), patches[i]->maxDistPlane()));
                patches[root]->minNormalDiff(std::min(patches[root]->minNormalDiff(), patches[i]->minNormalDiff()));
                delete patches[i];
                patches[i] = NULL;
            }
        }
        patches.erase(std::remove_if(patches.begin(), patches.end(), [](PlanarPatch *patch) {
            return patch == NULL;
        }), patches.end());
    }

    bool updatePatches(std::vector<PlanarPatch*> &patches)
    {
        bool changed = false;
        for (PlanarPatch *patch : patches)
        {
            if (patch->numNewPoints() > (patch->points().size() - patch->numNewPoints()) / 2)
            {
                patch->updatePlane();
                patch->stable() = false;
                changed = true;
            }
            else
            {
                patch->stable() = true;
            }
        }
        return changed;
    }

    void getPlaneOutlier(const PlanarPatch *patch, std::vector<size_t> &outlier)
    {
        Eigen::Vector3d basisU, basisV;
        GeometryUtils::orthogonalBasis(patch->plane().normal(), basisU, basisV);
        std::vector<Eigen::Vector2d> projectedPoints(patch->points().size());
        for (size_t i = 0; i < patch->points().size(); i++)
        {
            Eigen::Vector3d position = pointCloud()->points_[patch->points()[i]];
            projectedPoints[i] = GeometryUtils::projectOntoOrthogonalBasis(position, basisU, basisV);
        }
        GeometryUtils::convexHull(projectedPoints, outlier);
        for (size_t i = 0; i < outlier.size(); i++)
        {
            outlier[i] = patch->points()[outlier[i]];
        }
    }
    // determine the boundary of the plane
    void delimitPlane(PlanarPatch *patch)
    {
        std::vector<size_t> outlier;
        getPlaneOutlier(patch, outlier);
        Eigen::Vector3d normal = patch->plane().normal();
        Eigen::Vector3d basisU, basisV;
        GeometryUtils::orthogonalBasis(normal, basisU, basisV);
        Eigen::Matrix3d basis;
        basis << basisU, basisV, normal;
        Eigen::Matrix3Xd matrix(3, outlier.size());
        for (size_t i = 0; i < outlier.size(); i++)
        {
            matrix.col(i) = pointCloud()->points_[outlier[i]];
        }
        double minAngle = 0;
        double maxAngle = 90;
        while (maxAngle - minAngle > 5)
        {
            double mid = (maxAngle + minAngle) / 2;
            double left = (minAngle + mid) / 2;
            double right = (maxAngle + mid) / 2;
            RotatedRect leftRect(matrix, basis, left);
            RotatedRect rightRect(matrix, basis, right);
            if (leftRect.area < rightRect.area)
            {
                maxAngle = mid;
            }
            else
            {
                minAngle = mid;
            }
        }
        patch->rect() = RotatedRect(matrix, basis, (minAngle + maxAngle) / 2);
        Eigen::Vector3d center = patch->plane().center();
        Eigen::Vector3d minBasisU = patch->rect().basis.col(0);
        Eigen::Vector3d minBasisV = patch->rect().basis.col(1);
        center -= minBasisU * minBasisU.dot(center);
        center -= minBasisV * minBasisV.dot(center);
        center += minBasisU * (patch->rect().bottomLeft(0) + patch->rect().topRight(0)) / 2;
        center += minBasisV * (patch->rect().bottomLeft(1) + patch->rect().topRight(1)) / 2;
        double lengthU = (patch->rect().topRight(0) - patch->rect().bottomLeft(0)) / 2;
        double lengthV = (patch->rect().topRight(1) - patch->rect().bottomLeft(1)) / 2;
        Plane newPlane(center, patch->plane().normal(), minBasisU * lengthU, minBasisV * lengthV);
        patch->plane(newPlane);
    }

    bool isFalsePositive(PlanarPatch *patch)
    {
        bool b_num = patch->numUpdates() == 0;
        bool b_size = patch->getSize() / mMaxSize < 0.01f;
        if (m_min_plane_edge_length > 0)
        {
            b_size = patch->getSize() < m_min_plane_edge_length;
        }
        return b_num || b_size;
    }

};

#include <open3d/Open3D.h>
namespace rspd{

    std::set<Plane*> planeSegmentation(const std::shared_ptr<open3d::geometry::PointCloud> &cloud_ptr, std::string dataset = "KITTI"){
        static constexpr int nrNeighbors = 75;
        double minNormalDiff = 60.0;
        double maxDist = 75.0;
        double outlierRatio = 0.75;
        double minPlaneEdgeLength = 1.0;
        int minNumPoints = 30;
        if (dataset == "KITTI"){
            minNormalDiff = 10.0;
            maxDist = 10.0;
            outlierRatio = 0.1;
            minPlaneEdgeLength = 1.0;
            minNumPoints = 30;
        }

        const open3d::geometry::KDTreeSearchParam &search_param = open3d::geometry::KDTreeSearchParamKNN(nrNeighbors);
        cloud_ptr->EstimateNormals(search_param); // 108.6ms for 122k points in kitti bin file

        open3d::geometry::KDTreeFlann kdtree;
        kdtree.SetGeometry(*cloud_ptr); //116.5ms for 122k points in kitti bin file
        std::vector<std::vector<int>> neighbors;
        neighbors.resize(cloud_ptr->points_.size());
#pragma omp parallel for schedule(static)
        for (int i = 0; i < (int)cloud_ptr->points_.size(); i++) {
            std::vector<int> indices;
            std::vector<double> distance2;
            if (kdtree.Search(cloud_ptr->points_[i], search_param, indices, distance2)/* >= 3*/) {
                neighbors[i] = indices;
            }
        }

        PlaneDetector rspd(cloud_ptr, neighbors, minNormalDiff, maxDist, outlierRatio, minPlaneEdgeLength, minNumPoints);
        std::set<Plane*> planes = rspd.detect(); //159.6ms for 122k points in kitti bin file

        std::cout << "Detected the following " << planes.size() << " planes:" << std::endl;
        for (const auto& p : planes) {
            std::cout << p->normal().transpose() << " ";
            std::cout << p->distanceFromOrigin() << "\t";
            std::cout << p->center().transpose() << "\t";
            std::cout << p->basisU().transpose() << "\t";
            std::cout << p->basisV().transpose() << std::endl;
        }
        return planes;
    }
}

#endif // PLANEDETECTOR_H
