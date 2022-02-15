/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Thanapong Chuangyanyond CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <type_traits>
#include <utility>
#include <vector>
#include <algorithm>
#include <sys/time.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "EventLogger/EventLogger.hpp"
#include "VisionUtility/PointCloud/PointTypes.hpp"

namespace crf {
namespace utility {
namespace visionutility {
namespace pointcloud {
namespace edge {

/**
 * @brief      Compute the sharpness of each point in the input point cloud by using the
 *             eignevalues of the covariance matrix of neighbouring points.
 *
 * @param[in]  inputCloud The input cloud pointer.
 * @param[in]  kNeighbour The number of neighbour points that will be used to compute the
 *                         sharpness value.
 *
 * @tparam     PointT      Whatever PC type form pcl.
 *
 * @return     A Point cloud pointer with an spatial position of the input with 
 *             newly computed sharpness values (s field).
 */
template <typename PointT, std::enable_if_t<pcl::traits::has_xyz<PointT>::value>* = nullptr>
pcl::PointCloud<pointtypes::PointXYZS>::Ptr computeEigenSharpness(
    typename pcl::PointCloud<PointT>::Ptr inputCloud, unsigned int kNeighbour) {
    crf::utility::logger::EventLogger logger("computeEigenSharpness");

    if (inputCloud == nullptr || inputCloud->size() < 1) {
        logger->warn("Input Point Cloud not valid");
        return nullptr;
    }

    if (inputCloud->points.size() < kNeighbour || !kNeighbour) {
        logger->warn("Invalid number of neighbour points.");
        return nullptr;
    }

    // Create a placeholder for the output point cloud
    pcl::PointCloud<pointtypes::PointXYZS>::Ptr output(new pcl::PointCloud<pointtypes::PointXYZS>);

    // Create D-Tree from the input point clouds.
    std::vector<int> neighbourKdTree(kNeighbour);
    std::vector<float> neighbourKdTreeEuclideanDist(kNeighbour);

    // Create a point cloud with only position and copy input to ouput_pc
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*inputCloud, *inputCloudXYZ);
    pcl::copyPointCloud(*inputCloudXYZ, *output);

    // Create KdTree from xyz point cloud
    typename pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
    kdTree.setInputCloud(inputCloudXYZ);
    pcl::PointXYZ searchPoint;

    // Loop for all point in point cloud.
    for (std::size_t i = 0; i < inputCloud->points.size(); i++) {
        searchPoint = inputCloudXYZ->points[i];
        kdTree.nearestKSearch(searchPoint, kNeighbour, neighbourKdTree,
            neighbourKdTreeEuclideanDist);

        float xSum = 0.0; float ySum = 0.0; float zSum = 0.0;
        float xMean = 0.0; float yMean = 0.0; float zMean = 0.0;
        float xxSum = 0.0; float yySum = 0.0; float zzSum = 0.0;
        float xySum = 0.0; float xzSum = 0.0; float yzSum = 0.0;

        // First compute the neighbour cloud's mean.
        for (std::size_t j = 0; j < neighbourKdTree.size(); j++) {
            pcl::PointXYZ neighbourPoint = inputCloudXYZ->points[neighbourKdTree[j]];
            xSum += neighbourPoint.x;
            ySum += neighbourPoint.y;
            zSum += neighbourPoint.z;
        }

        xMean = xSum/neighbourKdTree.size();
        yMean = ySum/neighbourKdTree.size();
        zMean = zSum/neighbourKdTree.size();

        // Compute the covariance matrix
        for (std::size_t j = 0; j < neighbourKdTree.size(); j++) {
            pcl::PointXYZ neighbourPoint = inputCloudXYZ->points[neighbourKdTree[j]];

            // Compute variance of X, Y, and Z
            xxSum += (neighbourPoint.x - xMean) * (neighbourPoint.x - xMean);
            yySum += (neighbourPoint.y - yMean) * (neighbourPoint.y - yMean);
            zzSum += (neighbourPoint.z - zMean) * (neighbourPoint.z - zMean);

            // Compute covatiance of XY, YX, XZ, ZX, YZ, and ZY
            xySum += (neighbourPoint.x - xMean) * (neighbourPoint.y - yMean);
            xzSum += (neighbourPoint.x - xMean) * (neighbourPoint.z - zMean);
            yzSum += (neighbourPoint.y - yMean) * (neighbourPoint.z - zMean);
        }

        float varX = xxSum/neighbourKdTree.size();
        float varY = yySum/neighbourKdTree.size();
        float varZ = zzSum/neighbourKdTree.size();
        float covXY = xySum/neighbourKdTree.size();
        float covXZ = xzSum/neighbourKdTree.size();
        float covYZ = yzSum/neighbourKdTree.size();

        // Construct covariance matrix.
        Eigen::Matrix3f covarianceMatrix;
        covarianceMatrix << varX, covXY, covXZ, covXY, varY, covYZ, covXZ, covYZ, varZ;

        // Compute the eigenvalues
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covarianceMatrix);
        if (eigensolver.info() != Eigen::Success) {
            output->clear();
            return nullptr;
        }

        // Sort the output eigenvalues
        std::vector<float> eigenvals = {eigensolver.eigenvalues()[0], eigensolver.eigenvalues()[1],
            eigensolver.eigenvalues()[2]};
        std::sort(eigenvals.begin(), eigenvals.end());

        output->points[i].s = eigenvals[0]/(eigenvals[0]+eigenvals[1]+eigenvals[2]);
    }
    return output;
}

}  // namespace edge
}  // namespace pointcloud
}  // namespace visionutility
}  // namespace utility
}  // namespace crf
