#ifndef PLANEDETECTORPATCH_H
#define PLANEDETECTORPATCH_H

#include <memory>
#include <unordered_set>
#include <unordered_map>
#include <iostream>
#include <Eigen/Core>
#include "plane.h"
// #include "pointcloud.h"
#include "statisticsutils.h"

#include <open3d/Open3D.h>

struct RotatedRect
{
    Eigen::Matrix3Xd matrix;
    Eigen::Matrix3d basis;
    double area;
    Eigen::Vector3d bottomLeft;
    Eigen::Vector3d topRight;
    Eigen::Matrix3d R_12;

    RotatedRect() : area(std::numeric_limits<double>::max()) {}

    RotatedRect(const Eigen::Matrix3Xd &matrix, const Eigen::Matrix3d &basis, double degrees)
    {
        Eigen::Matrix3d R;
        R_12 = Eigen::AngleAxisd(degrees * 3.14159 / 180., Eigen::Vector3d::UnitZ());

        Eigen::Matrix3d newBasis = basis * R_12;
        Eigen::Matrix3Xd newMatrix = newBasis.transpose() * matrix;
        Eigen::Vector3d max = newMatrix.rowwise().maxCoeff();
        Eigen::Vector3d min = newMatrix.rowwise().minCoeff();
        double w = max(0) - min(0);
        double h = max(1) - min(1);
        double area = w * h;
        this->matrix = newMatrix;
        this->area = area;
        this->basis = newBasis;
        this->bottomLeft = min;
        this->topRight = max;
        this->R_12 = R_12;
    }

};

class PlanarPatch
{
public:
    using PointCloudConstPtr = std::shared_ptr<const open3d::geometry::PointCloud>;
    PlanarPatch(const PointCloudConstPtr& pointCloud, StatisticsUtils *statistics,
                             const std::vector<size_t> &points, double minAllowedNormal,
                             double maxAllowedDist, double outlierRatio)
            : mPointCloud(pointCloud)
            , mStatistics(statistics)
            , mPoints(points)
            , mOriginalSize(getSize())
            , mNumNewPoints(0)
            , mNumUpdates(0)
            , mStable(false)
            , mMinAllowedNormal(minAllowedNormal)
            , mMaxAllowedDist(maxAllowedDist)
            , mOutlierRatio(outlierRatio)
            , mUsedVisited2(false){}

    size_t index() const
    {
        return mIndex;
    }

    void index(size_t index)
    {
        mIndex = index;
    }

    inline bool isInlier(size_t point) const
    {
        return std::abs(mPlane.normal().dot(mPointCloud->normals_[point])) > mMinNormalDiff &&
                std::abs(mPlane.getSignedDistanceFromSurface(mPointCloud->points_[point])) < mMaxDistPlane;
    }

    inline bool isVisited(size_t point) const
    {
        if (!mVisited2.empty()) {
            return mVisited2[point];
        }
        return mVisited.find(point) != mVisited.end();
    }

    inline void visit(size_t point)
    {
        if (!mVisited2.empty()) {
            mVisited2[point] = true;
        } else {
            mVisited.insert(point);
        }
    }

    inline void addPoint(size_t point)
    {
        mPoints.push_back(point);
        ++mNumNewPoints;
        mOutliers[point] = false;
    }

    bool isPlanar()
    {
        mPlane = getPlane();

        mMinNormalDiff = getMinNormalDiff();
        if (!isNormalValid()) return false;
        size_t countOutliers = 0;
        for (size_t i = 0; i < mPoints.size(); i++)
        {
            bool outlier = mStatistics->dataBuffer()[i] < mMinNormalDiff;
            mOutliers[mPoints[i]] = outlier;
            countOutliers += outlier;
        }
        if (countOutliers > mPoints.size() * mOutlierRatio) return false;

        mMaxDistPlane = getMaxPlaneDist();
        if (!isDistValid()) return false;
        countOutliers = 0;
        for (size_t i = 0; i < mPoints.size(); i++)
        {
            bool outlier = mOutliers[mPoints[i]] || mStatistics->dataBuffer()[i] > mMaxDistPlane;
            mOutliers[mPoints[i]] = outlier;
            countOutliers += outlier;
        }
        if (countOutliers < mPoints.size() * mOutlierRatio)
        {
            removeOutliers();
            return true;
        }
        return false;
    }

    void updatePlane()
    {
        mPlane = getPlane();
        mVisited.clear();
        if (mNumUpdates > 1)
        {
            mUsedVisited2 = true;
        }
        if (mUsedVisited2) {
            if (mVisited2.empty()) {
                mVisited2 = std::vector<bool>(mPointCloud->points_.size(), false);
            } else {
                std::fill(mVisited2.begin(), mVisited2.end(), false);
            }
        }
        mNumNewPoints = 0;
        ++mNumUpdates;
    }

    const std::vector<size_t>& points() const
    {
        return mPoints;
    }

    void points(const std::vector<size_t> &points)
    {
        mPoints = points;
    }

    const Plane& plane() const
    {
        return mPlane;
    }

    void plane(const Plane &plane)
    {
        mPlane = plane;
    }

    const Eigen::Vector3d center() const
    {
        return mPlane.center();
    }

    void center(const Eigen::Vector3d &center)
    {
        mPlane.center(center);
    }

    const Eigen::Vector3d normal() const
    {
        return mPlane.normal();
    }

    void normal(const Eigen::Vector3d &normal)
    {
        mPlane.normal(normal);
    }

    double maxDistPlane() const
    {
        return mMaxDistPlane;
    }

    void maxDistPlane(double maxDistPlane)
    {
        mMaxDistPlane = maxDistPlane;
    }

    double minNormalDiff() const
    {
        return mMinNormalDiff;
    }

    void minNormalDiff(double minNormalDiff)
    {
        mMinNormalDiff = minNormalDiff;
    }

    RotatedRect& rect()
    {
        return mRect;
    }

    size_t& numNewPoints()
    {
        return mNumNewPoints;
    }

    bool& stable()
    {
        return mStable;
    }

    size_t numUpdates() const
    {
        return mNumUpdates;
    }

    size_t originalSize() const
    {
        return mOriginalSize;
    }

    void removeOutliers()
    {
        mPoints.erase(std::remove_if(mPoints.begin(), mPoints.end(), [this](const size_t &point) {
            return mOutliers[point];
        }), mPoints.end());
        mOutliers.clear();
    }

    double getSize() const{
        Eigen::Vector3d min = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
        Eigen::Vector3d max = -min;
        for (const size_t &point : mPoints)
        {
            Eigen::Vector3d position = mPointCloud->points_[point];
            for (size_t i = 0; i < 3; i++)
            {
                min(i) = std::min(min(i), position(i));
                max(i) = std::max(max(i), position(i));
            }
        }

        double maxSize = 0;
        for (size_t dim = 0; dim < 3; dim++) {
            maxSize = std::max(maxSize, max(dim) - min(dim));
        }
        return maxSize;
    }

// private:
    PointCloudConstPtr mPointCloud;
    StatisticsUtils *mStatistics;
    std::vector<size_t> mPoints;
    double mOriginalSize;
    size_t mIndex;
    Plane mPlane;
    double mMaxDistPlane;
    double mMinNormalDiff;
    RotatedRect mRect;
    size_t mNumNewPoints;
    size_t mNumUpdates;
    std::unordered_set<size_t> mVisited;
    std::vector<bool> mVisited2;
    std::unordered_map<size_t, bool> mOutliers;
    bool mStable;
    double mMinAllowedNormal;
    double mMaxAllowedDist;
    double mOutlierRatio;
    bool mUsedVisited2;

    Plane getPlane()
    {
        mStatistics->size(mPoints.size());
        Eigen::Vector3d center;
        for (size_t dim = 0; dim < 3; dim++)
        {
            for (size_t i = 0; i < mPoints.size(); i++)
            {
                mStatistics->dataBuffer()[i] = mPointCloud->points_[mPoints[i]](dim);
            }
            center(dim) = mStatistics->getMedian();
        }
        Eigen::Vector3d normal;
        for (size_t dim = 0; dim < 3; dim++)
        {
            for (size_t i = 0; i < mPoints.size(); i++)
            {
                mStatistics->dataBuffer()[i] = mPointCloud->normals_[mPoints[i]](dim);
            }
            normal(dim) = mStatistics->getMedian();
        }
        normal = normal.normalized();
        return Plane(center, normal);
    }

    double getMaxPlaneDist()
    {
        mStatistics->size(mPoints.size());
        for (size_t i = 0; i < mPoints.size(); i++)
        {
            const Eigen::Vector3d &position = mPointCloud->points_[mPoints[i]];
            mStatistics->dataBuffer()[i] = std::abs(mPlane.getSignedDistanceFromSurface(position));
        }
        double minDist, maxDist;
        mStatistics->getMinMaxRScore(minDist, maxDist, 3);
        return maxDist;
    }

    double getMinNormalDiff()
    {
        mStatistics->size(mPoints.size());
        for (size_t i = 0; i < mPoints.size(); i++)
        {
            const Eigen::Vector3d &normal = mPointCloud->normals_[mPoints[i]];
            mStatistics->dataBuffer()[i] = std::abs(normal.dot(mPlane.normal()));
        }
        double minDiff, maxDiff;
        mStatistics->getMinMaxRScore(minDiff, maxDiff, 3);
        return minDiff;
    }

    bool isNormalValid() const
    {
        return mMinNormalDiff > mMinAllowedNormal;
    }

    bool isDistValid() const
    {
        Eigen::Vector3d basisU, basisV;
        GeometryUtils::orthogonalBasis(mPlane.normal(), basisU, basisV);
        Eigen::Vector3d extreme = basisU * mOriginalSize + mPlane.normal() * mMaxDistPlane;
        return std::abs(extreme.normalized().dot(mPlane.normal())) < mMaxAllowedDist;
    }

};

#endif // PLANEDETECTORPATCH_H
