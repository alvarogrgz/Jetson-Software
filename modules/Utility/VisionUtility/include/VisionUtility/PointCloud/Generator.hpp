/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero and Alejandro Dias Rosales CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <type_traits>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/generate.h>
#include <pcl/common/random.h>

#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace utility {
namespace visionutility {
namespace pointcloud {
namespace generator {

/**
 * @brief      Generate a organized point cloud
 *
 * @param[in]  width      The width
 * @param[in]  height     The height
 * @param[in]  minX       The minimum x
 * @param[in]  maxX       The maximum x
 * @param[in]  minY       The minimum y
 * @param[in]  maxY       The maximum y
 * @param[in]  minZ       The minimum z
 * @param[in]  maxZ       The maximum z
 *
 * @tparam     PointT     Whatever point cloud form pcl
 *
 * @return     The cloud
 */
template<typename PointT, std::enable_if_t<pcl::traits::has_xyz<PointT>::value>* = nullptr>
typename pcl::PointCloud<PointT>::Ptr uniformPointCloudGenerator(int width, int height,
    float minX, float maxX,
    float minY, float maxY,
    float minZ, float maxZ) {
    crf::utility::logger::EventLogger logger("uniformPointCloudGenerator");

    if (width <= 0) {
        logger->error("The width of the Point Cloud must be 1 or more");
        return nullptr;
    }
    if (height <= 0) {
        logger->error("The height of the Point Cloud must be 1 or more");
        return nullptr;
    }
    if (minX > maxX) {
        logger->error("The minimum X can't be bigger than the maximum");
        return nullptr;
    }
    if (minY > maxY) {
        logger->error("The minimum Y can't be bigger than the maximum");
        return nullptr;
    }
    if (minZ > maxZ) {
        logger->error("The minimum Z can't be bigger than the maximum");
        return nullptr;
    }
    typename pcl::common::CloudGenerator<PointT, pcl::common::UniformGenerator<float>> generator;
    std::uint32_t seed = static_cast<std::uint32_t>(time(nullptr));
    pcl::common::UniformGenerator<float>::Parameters xParameters(minX, maxX, seed++);
    pcl::common::UniformGenerator<float>::Parameters yParameters(minY, maxY, seed++);
    pcl::common::UniformGenerator<float>::Parameters zParameters(minZ, maxZ, seed++);
    generator.setParametersForX(xParameters);
    generator.setParametersForY(yParameters);
    generator.setParametersForZ(zParameters);
    typename pcl::PointCloud<PointT> auxCloud;
    if (generator.fill(width, height, auxCloud) != 0)
        return nullptr;
    typename pcl::PointCloud<PointT>::Ptr outputCloud(new pcl::PointCloud<PointT>(auxCloud));
    return outputCloud;
}

/**
 * @brief      Generate a point cloud of normals
 *
 * @param[in]  width      The width
 * @param[in]  height     The height
 * @param[in]  meanX      The mean x
 * @param[in]  stdDevX    The standard dev x
 * @param[in]  meanY      The mean y
 * @param[in]  stdDevY    The standard dev y
 * @param[in]  meanZ      The mean z
 * @param[in]  stdDevZ    The standard dev z
 *
 * @tparam     PointT     Whatever point cloud form pcl
 *
 * @return     The normals point cloud
 */
template<typename PointT, std::enable_if_t<pcl::traits::has_xyz<PointT>::value>* = nullptr>
typename pcl::PointCloud<PointT>::Ptr normalPointCloudGenerator(int width, int height,
    float meanX, float stdDevX,
    float meanY, float stdDevY,
    float meanZ, float stdDevZ) {
    crf::utility::logger::EventLogger logger("normalPointCloudGenerator");

    if (width <= 0) {
        logger->error("The width of the Point Cloud must be 1 or more");
        return nullptr;
    }
    if (height <= 0) {
        logger->error("The height of the Point Cloud must be 1 or more");
        return nullptr;
    }
    if (stdDevX < 0) {
        logger->error("The standard deviation in X can't be negative");
        return nullptr;
    }
    if (stdDevY < 0) {
        logger->error("The standard deviation in Y can't be negative");
        return nullptr;
    }
    if (stdDevZ < 0) {
        logger->error("The standard deviation in Z can't be negative");
        return nullptr;
    }
    typename pcl::common::CloudGenerator<PointT, pcl::common::NormalGenerator<float>> generator;
    std::uint32_t seed = static_cast<std::uint32_t>(time(nullptr));
    pcl::common::NormalGenerator<float>::Parameters xParameters(meanX, stdDevX, seed++);
    pcl::common::NormalGenerator<float>::Parameters yParameters(meanY, stdDevY, seed++);
    pcl::common::NormalGenerator<float>::Parameters zParameters(meanZ, stdDevZ, seed++);
    generator.setParametersForX(xParameters);
    generator.setParametersForY(yParameters);
    generator.setParametersForZ(zParameters);
    typename pcl::PointCloud<PointT> auxCloud;
    if (generator.fill(width, height, auxCloud) != 0)
        return nullptr;
    typename pcl::PointCloud<PointT>::Ptr outputCloud(new pcl::PointCloud<PointT>(auxCloud));
    return outputCloud;
}

}  // namespace generator
}  // namespace pointcloud
}  // namespace visionutility
}  // namespace utility
}  // namespace crf
