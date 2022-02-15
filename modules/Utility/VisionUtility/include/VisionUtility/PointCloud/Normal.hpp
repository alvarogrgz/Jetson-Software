/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <type_traits>
#include <utility>
#include <vector>
#include <sys/time.h>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/features/integral_image_normal.h>

#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace utility {
namespace visionutility {
namespace pointcloud {
namespace normal {

/**
 * @brief      Removes nan normals.
 *
 * @param[in]  inputCloud    The input cloud
 * @param[in]  inputNormals  The input normals
 *
 * @tparam     PointT        Whatever point cloud form pcl
 *
 * @return     The point cloud (PointT and Normal types) without the NaNs 
 */
template<typename PointT, std::enable_if_t<pcl::traits::has_xyz<PointT>::value>* = nullptr>
std::pair<typename pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> removeNaNNormals(  // NOLINT
    typename pcl::PointCloud<PointT>::Ptr inputCloud,
    pcl::PointCloud<pcl::Normal>::Ptr inputNormals) {
    crf::utility::logger::EventLogger logger("removeNaNNormals");
    logger->debug("Removing NaN from the normals");
    if (inputCloud == nullptr || inputCloud->size() < 1 || inputNormals == nullptr ||
            inputNormals->size() < 1) {
        logger->error("Wrong input cloud");
        return std::make_pair(nullptr, nullptr);
    }

    typename pcl::PointCloud<PointT>::Ptr outputCloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr outputNormals (new pcl::PointCloud<pcl::Normal>);
    std::vector<int> index;

    pcl::removeNaNNormalsFromPointCloud(*inputNormals, *outputNormals, index);
    unsigned int indexSize = index.size();
    for (unsigned int i = 0; i < indexSize; i++) {
        outputCloud->push_back(inputCloud->points[index[i]]);
    }

    return std::make_pair(outputCloud, outputNormals);
}

/**
 * @brief      Calculates the integral normals.
 *
 * @param[in]  cloud                       The cloud (organized mainly)
 * @param[in]  estimationMethod            The estimation method ( COVARIANCE_MATRIX     
                                                                   AVERAGE_3D_GRADIENT     
                                                                   AVERAGE_DEPTH_CHANGE    
                                                                   SIMPLE_3D_GRADIENT )
 * @param[in]  normalMaxDepthChangeFactor  The normal maximum depth change factor
 * @param[in]  normalSmoothingSize         The normal smoothing size
 * @param[in]  cameraPoseX                 The camera pose x
 * @param[in]  cameraPoseY                 The camera pose y
 * @param[in]  cameraPoseZ                 The camera pose z
 *
 * @tparam     PointT                      Whatever point cloud form pcl
 *
 * @return     The integral normals.
 */
template<typename PointT, std::enable_if_t<pcl::traits::has_xyz<PointT>::value>* = nullptr>
pcl::PointCloud<pcl::Normal>::Ptr computeIntegralNormals(
        const typename pcl::PointCloud<PointT>::Ptr cloud,
        typename pcl::IntegralImageNormalEstimation<PointT, pcl::Normal>::
            NormalEstimationMethod estimationMethod,
        float normalMaxDepthChangeFactor, float normalSmoothingSize,
        float cameraPoseX, float cameraPoseY, float cameraPoseZ) {
    crf::utility::logger::EventLogger logger("computeIntegralNormals");
    logger->debug("Computing integral normals");
    if (cloud == nullptr || cloud->size() < 1) {
        logger->error("Wrong input cloud");
        return nullptr;
    }

    // Start the chrono
    std::chrono::time_point<std::chrono::system_clock> start, end;
    int elapsedMilliseconds {0};
    start = std::chrono::system_clock::now();

    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals (new pcl::PointCloud<pcl::Normal>);
    typename pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> sourceNormalEstimation;

    sourceNormalEstimation.setNormalEstimationMethod(estimationMethod);

    if ((normalMaxDepthChangeFactor < 0) || (normalSmoothingSize < 0)) {
        logger->error("normalMaxDepthChangeFactor should be greater or equal to 0");
        return nullptr;
    }

    sourceNormalEstimation.setMaxDepthChangeFactor(normalMaxDepthChangeFactor);
    sourceNormalEstimation.setNormalSmoothingSize(normalSmoothingSize);
    sourceNormalEstimation.setInputCloud(cloud);
    sourceNormalEstimation.setViewPoint(cameraPoseX, cameraPoseY, cameraPoseZ);
    sourceNormalEstimation.compute(*sourceNormals);

    end = std::chrono::system_clock::now();
    elapsedMilliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    logger->debug("Integral normals computing elapsed time: {0} milliseconds", elapsedMilliseconds);
    return sourceNormals;
}

/**
 * @brief      Calculates the normals.
 *
 * @param[in]  inputCloud                    The input cloud (unorganized)
 * @param[in]  normalEstimationSearchRadius  The normal estimation search radius
 *
 * @tparam     PointT                        Whatever point cloud form pcl
 *
 * @return     The normals.
 */
template<typename PointT, std::enable_if_t<pcl::traits::has_xyz<PointT>::value>* = nullptr>
pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
        const typename pcl::PointCloud<PointT>::Ptr inputCloud,
        float normalEstimationSearchRadius) {
    crf::utility::logger::EventLogger logger("computeNormals");
    if (inputCloud == nullptr || inputCloud->size() < 1) {
        logger->error("Wrong input cloud");
        return nullptr;
    }

    // Start the chrono
    std::chrono::time_point<std::chrono::system_clock> start, end;
    int elapsedMilliseconds {0};
    start = std::chrono::system_clock::now();

    logger->debug("Computing normals");

    if (normalEstimationSearchRadius <= 0) {
        logger->error("normalEstimationSearchRadius should be greater than 0");
        return nullptr;
    }
    pcl::PointCloud<pcl::Normal>::Ptr outputCloud(new pcl::PointCloud<pcl::Normal>());

    typename pcl::NormalEstimation<PointT, pcl::Normal> normalEstimationFilter;
    typename pcl::search::KdTree<PointT>::Ptr searchTree(new pcl::search::KdTree<PointT>);
    normalEstimationFilter.setInputCloud(inputCloud);
    normalEstimationFilter.setSearchMethod(searchTree);
    normalEstimationFilter.setRadiusSearch(normalEstimationSearchRadius);
    normalEstimationFilter.compute(*outputCloud);

    end = std::chrono::system_clock::now();
    elapsedMilliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    logger->debug("Normals computing elapsed time: {0} milliseconds", elapsedMilliseconds);

    return outputCloud;
}

}  // namespace normal
}  // namespace pointcloud
}  // namespace visionutility
}  // namespace utility
}  // namespace crf
