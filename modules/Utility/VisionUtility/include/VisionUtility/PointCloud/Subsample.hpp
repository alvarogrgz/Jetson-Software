/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <type_traits>
#include <vector>
#include <sys/time.h>
#include <boost/optional.hpp>
#include <utility>
#include <tgmath.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/normal_space.h>
#include <pcl/common/point_tests.h>

#include "EventLogger/EventLogger.hpp"

#define PI 3.14159265

namespace crf {
namespace utility {
namespace visionutility {
namespace pointcloud {
namespace subsample {

/**
 * @brief      Subsample the PC through voxels
 *
 * @param[in]  inputCloud  The input cloud
 * @param[in]  sideLength  The side length
 *
 * @tparam     PointT      Whatever PC type form pcl
 *
 * @return     Subsampled cloud
 */
template<typename PointT, std::enable_if_t<pcl::traits::has_xyz<PointT>::value>* = nullptr>
typename pcl::PointCloud<PointT>::Ptr voxelGridSubsample(
    const typename pcl::PointCloud<PointT>::Ptr inputCloud,
    const std::vector<float> &sideLength) {
    crf::utility::logger::EventLogger logger("voxelGridSubsample");
    logger->debug("voxelGridSubsample");

    if ((inputCloud == nullptr) || (inputCloud->size() < 1)) {
        logger->error("Input Point Cloud not valid");
        return nullptr;
    }
    if (sideLength.size() != 3) {
        logger->error("The side length vector must have 3 elements");
        return nullptr;
    }

    for (unsigned int i = 0; i < sideLength.size(); i++) {
        if (sideLength.at(i) <= 0) {
            logger->error("The side length {} must be greater than 0", i);
            return nullptr;
        }
    }
    typename pcl::PointCloud<PointT>::Ptr outputCloud(new typename pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr auxiliar(new typename pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*inputCloud, *auxiliar);

    typename pcl::VoxelGrid<PointT> subsamplingFilter;
    subsamplingFilter.setInputCloud(auxiliar);
    subsamplingFilter.setLeafSize(sideLength.at(0), sideLength.at(1), sideLength.at(2));
    subsamplingFilter.filter(*outputCloud);

    logger->debug("Size of the outputCloud = {}", outputCloud->size());
    if (outputCloud->size() <= 1) {
        logger->error("Unable to subsample the point cloud");
        return nullptr;
    }

    return outputCloud;
}

/**
 * @brief      Subsample the PC through random selection of points
 *
 * @param[in]  cloud                               The cloud (without NaN)
 * @param[in]  normals                             The normals cloud
 * @param[in]  randomlSamplingTotalPointsDivision  The randoml sampling total points division
 *
 * @tparam     PointT                              Whatever PC type form pcl
 *
 * @return     Both cloud and normals subsampled
 */

template<typename PointT, std::enable_if_t<pcl::traits::has_xyz<PointT>::value>* = nullptr>
std::pair<typename pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> randomSubsampling(  // NOLINT
        const typename pcl::PointCloud<PointT>::Ptr cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr normals,
        int randomlSamplingTotalPointsDivision) {
    // filterInputPointcloud
    crf::utility::logger::EventLogger logger("randomSubsampling");
    logger->debug("Random sumsampling");
    if (cloud == nullptr || cloud->size() < 1 || normals == nullptr || normals->size() < 1) {
        logger->error("Input Point Cloud not valid");
        return std::make_pair(nullptr, nullptr);
    }

    std::vector<int> removedIndices;

    if (randomlSamplingTotalPointsDivision <= 0) {
        logger->error("randomlSamplingTotalPointsDivision should be greater than 0");
        return std::make_pair(nullptr, nullptr);
    }

    typename pcl::RandomSample<PointT> srcRandomSampling;
    srcRandomSampling.setInputCloud(cloud);
    srcRandomSampling.setSample(static_cast<unsigned int>
        (cloud->size()/randomlSamplingTotalPointsDivision));
    srcRandomSampling.filter(removedIndices);

    typename pcl::PointCloud<PointT>::Ptr cloud2 (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<pcl::Normal>::Ptr normals2 (new pcl::PointCloud<pcl::Normal>());
    for (int i = 0; i < static_cast<int>(removedIndices.size()); i++) {
        cloud2->push_back(cloud->points[removedIndices.at(i)]);
        normals2->push_back(normals->points[removedIndices.at(i)]);
    }

    for (int i = 0; i < static_cast<int>(normals2->points.size()); i++) {
        if (!pcl::isFinite<pcl::Normal>(normals2->points[i])) {
            logger->warn("normals[{}] is not finite\n", i);
            return std::make_pair(nullptr, nullptr);
        }
    }

    logger->debug("Random sumsampling finished");
    return std::make_pair(cloud2, normals2);
}

/**
 * @brief      Subsample the PC through a normal distribution selection of points
 *
 * @param[in]  cloud                              The cloud (without NaN)
 * @param[in]  normals                            The normals cloud
 * @param[in]  normalSamplingTotalPointsDivision  The normal sampling total points division
 * @param[in]  normalSamplingBinSize              The normal sampling bin size
 *
 * @tparam     PointT                             Whatever PC type form pcl
 *
 * @return     Both cloud and normals subsampled
 */

template<typename PointT, std::enable_if_t<pcl::traits::has_xyz<PointT>::value>* = nullptr>
std::pair<typename pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr> normalsSubsampling(  // NOLINT
        const typename pcl::PointCloud<PointT>::Ptr cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr normals,
        int normalSamplingTotalPointsDivision, int normalSamplingBinSize) {
    // filterInputPointcloud
    crf::utility::logger::EventLogger logger("normalsSubsampling");
    logger->debug("Normals subsampling");
    if (cloud == nullptr || cloud->size() < 1 || normals == nullptr || normals->size() < 1) {
        logger->error("Input Point Clouds not valid");
        return std::make_pair(nullptr, nullptr);
    }
    std::vector<int> removedIndices;

    if (normalSamplingTotalPointsDivision <= 0) {
        logger->error("normalSamplingTotalPointsDivision should be greater than 0");
        return std::make_pair(nullptr, nullptr);
    }
    if (normalSamplingBinSize <= 0) {
        logger->error("normalSamplingBinSize should be greater than 0");
        return std::make_pair(nullptr, nullptr);
    }
    // Source Normal space subsampling
    typename pcl::NormalSpaceSampling<PointT, pcl::Normal> srcNormalSampling;
    srcNormalSampling.setInputCloud(cloud);
    srcNormalSampling.setNormals(normals);
    srcNormalSampling.setBins(normalSamplingBinSize, normalSamplingBinSize, normalSamplingBinSize);
    srcNormalSampling.setSeed(0);
    srcNormalSampling.setSample(
        static_cast<unsigned int>(cloud->size())/normalSamplingTotalPointsDivision);
    srcNormalSampling.filter(removedIndices);

    typename pcl::PointCloud<PointT>::Ptr cloud2 (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<pcl::Normal>::Ptr normals2 (new pcl::PointCloud<pcl::Normal>());
    for (int i = 0; i < static_cast<int>(removedIndices.size()); i++) {
        cloud2->push_back(cloud->points[removedIndices.at(i)]);
        normals2->push_back(normals->points[removedIndices.at(i)]);
    }

    for (int i = 0; i < static_cast<int>(normals2->points.size()); i++) {
        if (!pcl::isFinite<pcl::Normal>(normals2->points[i])) {
            logger->warn("normals[{}] is not finite\n", i);
            return std::make_pair(nullptr, nullptr);
        }
    }

    logger->debug("Normals sumsampling finished");
    return std::make_pair(cloud2, normals2);
}

/**
 * @brief      Function that subsample a point cloud making a difference between the area of
 *             interest and the rest of the point cloud. Area of interest is important for the
 *             operator, while the rest of the point cloud is important to avoid collisions
 *
 * @param[in]  cloud                      The cloud
 * @param[in]  highVerticalFieldOfView    The high vertical field of view (degrees)
 * @param[in]  highHorizontalFieldOfView  The high horizontal field of view (degrees)
 * @param[in]  lowVerticalFieldOfView     The low vertical field of view (degrees)
 * @param[in]  lowHorizontalFieldOfView   The low horizontal field of view (degrees)
 * @param[in]  internalSubsampleSize      The internal subsample size
 * @param[in]  externalSubsampleSize      The external subsample size
 *
 * @tparam     PointT                     Whatever point cloud type of PCL
 *
 * @return     The cloud with different subsamples within the areas
 */
template<typename PointT, std::enable_if_t<pcl::traits::has_xyz<PointT>::value>* = nullptr>
typename pcl::PointCloud<PointT>::Ptr focusInAreaOfInterest(
        const typename pcl::PointCloud<PointT>::Ptr cloud,
        float highVerticalFieldOfView, float highHorizontalFieldOfView,
        float lowVerticalFieldOfView, float lowHorizontalFieldOfView,
        float internalSubsampleSize, float externalSubsampleSize) {
    crf::utility::logger::EventLogger logger("focusInAreaOfInterest");
    logger->debug("Focusing in the area of interest");

    if (cloud == nullptr || cloud->size() < 1) {
        logger->error("Input Point Cloud not valid");
        return nullptr;
    }
    if ((highVerticalFieldOfView <= lowVerticalFieldOfView) ||
        (highHorizontalFieldOfView <= lowHorizontalFieldOfView)) {
        logger->error("Wrong field of view parameters");
        return nullptr;
    }
    if (internalSubsampleSize <= 0 || externalSubsampleSize <= 0) {
        logger->error("Subsample sizes not valid");
        return nullptr;
    }

    // Start the chrono
    std::chrono::time_point<std::chrono::system_clock> start, end;
    int elapsedMilliseconds {0};
    start = std::chrono::system_clock::now();

    // Point declarations
    Eigen::Vector3f p0(0.0, 0.0, 0.0);
    Eigen::Vector3f plu(
        tan(PI * highHorizontalFieldOfView / 180), tan(PI * highVerticalFieldOfView / 180), 1);
    Eigen::Vector3f pld(
        tan(PI * highHorizontalFieldOfView / 180), tan(PI * lowVerticalFieldOfView / 180), 1);
    Eigen::Vector3f pru(
        tan(PI * lowHorizontalFieldOfView / 180), tan(PI * highVerticalFieldOfView / 180), 1);
    Eigen::Vector3f prd(
        tan(PI * lowHorizontalFieldOfView / 180), tan(PI * lowVerticalFieldOfView / 180), 1);

    // Plane obtain
    Eigen::Vector3f normalLeft = plu.cross(pld);
    Eigen::Vector3f normalRight = pru.cross(prd);
    Eigen::Vector3f normalUp = plu.cross(pru);
    Eigen::Vector3f normalDown = pld.cross(prd);

    // Difference between internal and external sides
    typename pcl::PointCloud<PointT>::Ptr internalCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr externalCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr outCloud (new pcl::PointCloud<PointT>());
    for (size_t i = 0; i < cloud->size(); i++) {
        float value = normalLeft(0) * cloud->at(i).x + normalLeft(1) * cloud->at(i).y +
            normalLeft(2) * cloud->at(i).z;
        if (value > 0) {
            externalCloud->points.push_back(cloud->points.at(i));
            continue;
        }
        value = normalRight(0) * cloud->at(i).x + normalRight(1) * cloud->at(i).y +
            normalRight(2) * cloud->at(i).z;
        if (value < 0) {
            externalCloud->points.push_back(cloud->points.at(i));
            continue;
        }
        value = normalUp(0) * cloud->at(i).x + normalUp(1) * cloud->at(i).y +
            normalUp(2) * cloud->at(i).z;
        if (value < 0) {
            externalCloud->points.push_back(cloud->points.at(i));
            continue;
        }
        value = normalDown(0) * cloud->at(i).x + normalDown(1) * cloud->at(i).y +
            normalDown(2) * cloud->at(i).z;
        if (value > 0) {
            externalCloud->points.push_back(cloud->points.at(i));
            continue;
        }

        internalCloud->points.push_back(cloud->points.at(i));
    }

    // Subsample
    std::vector<float> internalSideLength(3, internalSubsampleSize);
    internalCloud = voxelGridSubsample<PointT>(internalCloud, internalSideLength);
    if (internalCloud == nullptr)
        return nullptr;

    std::vector<float> externalSideLength(3, externalSubsampleSize);
    externalCloud = voxelGridSubsample<PointT>(externalCloud, externalSideLength);
    if (externalCloud == nullptr)
        return nullptr;

    *outCloud = *internalCloud;
    size_t cloudSize{outCloud->size()};
    for (size_t j = cloudSize; j < (cloudSize + externalCloud->size()); j++) {
        outCloud->push_back(externalCloud->points[j - cloudSize]);
    }

    end = std::chrono::system_clock::now();
    elapsedMilliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    logger->debug("Computing elapsed time: {0} milliseconds", elapsedMilliseconds);

    logger->debug("Size of the input cloud: {}", cloud->size());
    logger->debug("Size of the output cloud: {}", outCloud->size());

    return outCloud;
}

}  // namespace subsample
}  // namespace pointcloud
}  // namespace visionutility
}  // namespace utility
}  // namespace crf
