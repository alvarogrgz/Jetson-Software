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
#include <memory>
#include <sys/time.h>
#include <boost/optional.hpp>
#include <limits>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/random_sample.h>
#include <pcl/common/transforms.h>

#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace utility {
namespace visionutility {
namespace pointcloud {
namespace filter {

struct FiltersParameters {
    bool fastBilateral;
    bool passThrough;
    bool voxelGrid;
    bool statisticalOutlier;
    bool mls;
    float fastBilateralSigmaR;
    float fastBilateralSigmaS;
    float passThroughMinLimit;
    float passThroughMaxLimit;
    float voxelGridLeafSizeX;
    float voxelGridLeafSizeY;
    float voxelGridLeafSizeZ;
    int statisticalOutlierMeanK;
    float statisticalOutlierStddevMulThresh;
    int mlsPolynomialOrder;
    float mlsSearchRadius;
};

/**
 * @brief      Range-based filter: it filter a point cloud removing the points closes and further
 *
 * @param[in]  cloud                The cloud to filter
 * @param[in]  passThroughMinLimit  The pass through minimum limit (m)
 * @param[in]  passThroughMaxLimit  The pass through maximum limit (m)
 *
 * @tparam     PointT               Whatever point cloud type of PCL
 *
 * @return     Filtered point cloud
 */
template<typename PointT, std::enable_if_t<pcl::traits::has_xyz<PointT>::value>* = nullptr>
typename pcl::PointCloud<PointT>::Ptr basedRangeFilter(
        const typename pcl::PointCloud<PointT>::Ptr &cloud,
        float passThroughMinLimit, float passThroughMaxLimit) {
    crf::utility::logger::EventLogger logger("basedRangeFilter");
    if ((cloud->size() < 1) || (passThroughMinLimit >= passThroughMaxLimit) ||
            (passThroughMinLimit < 0) || (passThroughMaxLimit < 0)) {
        logger->warn("Cloud, minimum or maximum values provided invalid");
        return nullptr;
    }
    logger->debug("Filtering point cloud by range...");
    typename pcl::PointCloud<PointT>::Ptr auxCloud (new pcl::PointCloud<PointT>());
    int counter{0};
    float dist{0.0};
    for (size_t it = 0; it < cloud->size(); it++) {
        dist = sqrt(pow(cloud->at(it).x, 2) + pow(cloud->at(it).y, 2) + pow(cloud->at(it).z, 2));
        if (!(dist < passThroughMinLimit) && !(dist > passThroughMaxLimit)) {
            auxCloud->push_back(cloud->at(it));
            counter++;
        }
    }

    logger->debug("basedRangeFilter done removing {} points", (cloud->size() - counter));
    return auxCloud;
}


/**
 * @brief      Plane-based filter, it filter a point cloud removing the points of one hemisphere.
 *             Plane definition: A*x + B*y + C*z + D = 0
 *
 * @param[in]  cloud      The cloud to filter
 * @param[in]  A          x constant
 * @param[in]  B          y constant
 * @param[in]  C          z constant
 * @param[in]  D          function weight (m)
 * @param[in]  upper      Keep the upper hemisphere if true, lower if false
 *
 * @tparam     PointT     Whatever point cloud type of PCL
 *
 * @return     Filtered point cloud
 */
template<typename PointT, std::enable_if_t<pcl::traits::has_xyz<PointT>::value>* = nullptr>
typename pcl::PointCloud<PointT>::Ptr cutPointCloud(
        const typename pcl::PointCloud<PointT>::Ptr &cloud,
        float A, float B, float C, float D, bool upper = true) {
    crf::utility::logger::EventLogger logger("cutPointCloud");
    if (cloud->size() < 1) {
        logger->warn("Cloud, minimum or maximum values provided invalid");
        return nullptr;
    }

    logger->debug("Cutting a point cloud through a plane given by {}*x + {}*y + {}*z + {} = 0...",
        A, B, C, D);

    typename pcl::PointCloud<PointT>::Ptr auxCloud (new pcl::PointCloud<PointT>());
    int counter{0};

    for (size_t it = 0; it < cloud->size(); it++) {
        float value = A * cloud->at(it).x + B * cloud->at(it).y + C * cloud->at(it).z + D;
        if (upper && (value >= 0)) {
            auxCloud->push_back(cloud->at(it));
            counter++;
        } else if (!upper && (value <= 0)) {
            auxCloud->push_back(cloud->at(it));
            counter++;
        }
    }

    logger->debug("Removed {} points", cloud->size() - auxCloud->size());
    logger->debug("New size of {} points", auxCloud->size());
    return auxCloud;
}

/**
 * @brief      Filter the artifcats of a point cloud, i.e. provide a good format for the cloud treatment
 *
 * @param[in]  cloud                The cloud to filter (Reccomendable for PCs from the RGBd camera)
 * @param[in]  passThroughMinLimit  The pass through minimum limit (m)
 *
 * @tparam     PointT               Whatever point cloud type of PCL
 *
 * @return     Filtered point cloud if success, if there are not very close objects
 */
template<typename PointT, std::enable_if_t<pcl::traits::has_xyz<PointT>::value>* = nullptr>
typename pcl::PointCloud<PointT>::Ptr checkForArtifactsInPointcloud(
    const typename pcl::PointCloud<PointT>::Ptr &cloud, float passThroughMinLimit) {
    crf::utility::logger::EventLogger logger("checkForArtifactsInPointcloud");
    logger->debug("Artifacts checking");
    if ((cloud->size() < 1) || (passThroughMinLimit < 0)) {
        logger->warn("Cloud, minimum or maximum values provided invalid");
        return nullptr;
    }

    typename pcl::PointCloud<PointT>::Ptr auxCloud;
    auxCloud = cloud;
    // If there are points closer than passThroughMinLimit, dont take into account this
    // pointcloud to avoid artifacts.
     for (typename pcl::PointCloud<PointT>::iterator it = auxCloud->begin();
            it != auxCloud->end(); it++) {
        if (it->x == 0 && it->y == 0 && it->z == 0) {
            it->x = std::numeric_limits<float>::quiet_NaN();
            it->y = std::numeric_limits<float>::quiet_NaN();
            it->z = std::numeric_limits<float>::quiet_NaN();
        }
        if (it->z < passThroughMinLimit && it->z > 0) {
            logger->warn("Found points too close to the camera");
            logger->warn("Removing pointcloud from pipeline");
            return nullptr;
        }
    }
    logger->debug("Artifacts checked");
    return auxCloud;
}

/**
 * @brief      Filter point cloud in three senses: (1) remove the points closer than a threshold
 * through the passThrough argument, (2) remove the outliers, and (3) smooth the cloud
 *
 * @param[in]  rawCloud     The raw cloud (Reccomendable organized cloud)
 * @param[in]  filtersData  The filters data
 *
 * @tparam     PointT       Whatever point cloud type of PCL
 *
 * @return     Filtered point cloud
 */
template<typename PointT, std::enable_if_t<pcl::traits::has_xyz<PointT>::value>* = nullptr>
typename pcl::PointCloud<PointT>::Ptr filterInputPointcloud(
    const typename pcl::PointCloud<PointT>::Ptr &rawCloud, const FiltersParameters &filtersData) {
    crf::utility::logger::EventLogger logger("filterInputPointcloud");

    if (rawCloud->size() < 1) {
        logger->warn("Cloud, minimum or maximum values provided invalid");
        return nullptr;
    }

    // Start the chrono
    std::chrono::time_point<std::chrono::system_clock> start, end;
    int elapsedMilliseconds {0};
    start = std::chrono::system_clock::now();

    logger->debug("Filtering input cloud");
    typename pcl::PointCloud<PointT>::Ptr trimmedPointCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr filteredPointCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr smothedPointCloud(new pcl::PointCloud<PointT>());

    if (!rawCloud->isOrganized()) {
        if (filtersData.passThrough || filtersData.statisticalOutlier ||
            filtersData.fastBilateral) {
            logger->error("Input cloud is not organized, not possible to apply filters");
            return nullptr;
        }
    }

    // Remove points distant more than passThroughMaxLimit meters from camera (keeping organized)
    if (filtersData.passThrough) {
        typename pcl::PassThrough<PointT> pass;
        pass.setKeepOrganized(true);
        pass.setInputCloud(rawCloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(filtersData.passThroughMinLimit, filtersData.passThroughMaxLimit);
        pass.filter(*trimmedPointCloud);
    } else {
        *trimmedPointCloud = *rawCloud;
    }

    // Remove outliers
    if (filtersData.statisticalOutlier) {
        typename pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setKeepOrganized(true);
        sor.setInputCloud(trimmedPointCloud);
        sor.setMeanK(filtersData.statisticalOutlierMeanK);
        sor.setStddevMulThresh(filtersData.statisticalOutlierStddevMulThresh);
        sor.filter(*filteredPointCloud);
    } else {
        *filteredPointCloud = *trimmedPointCloud;
    }

    // Smooth cloud
    if (filtersData.fastBilateral) {
        typename pcl::FastBilateralFilter<PointT> bilateral_filter;
        bilateral_filter.setInputCloud(filteredPointCloud);
        bilateral_filter.setSigmaS(filtersData.fastBilateralSigmaS);
        bilateral_filter.setSigmaR(filtersData.fastBilateralSigmaR);
        bilateral_filter.filter(*smothedPointCloud);
    } else {
        *smothedPointCloud = *filteredPointCloud;
    }

    end = std::chrono::system_clock::now();
    elapsedMilliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    logger->debug("Input cloud filtering elapsed time: {0} milliseconds", elapsedMilliseconds);

    return smothedPointCloud;
}

}  // namespace filter
}  // namespace pointcloud
}  // namespace visionutility
}  // namespace utility
}  // namespace crf
