/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO 2020
 *         Alejandro Diaz Rosales CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <type_traits>
#include <vector>
#include <utility>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/norms.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/shot_omp.h>
#include <pcl/impl/point_types.hpp>

#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace utility {
namespace visionutility {
namespace pointcloud {
namespace keypoint {

/**
 * @brief      Gets the fpfh keypoints.
 *
 * @param[in]  inputCloud        The input cloud
 * @param[in]  inputNormals      The input normals
 * @param[in]  scalesVector      The scales vector
 * @param[in]  persintenceAlpha  The persintence alpha
 * @param[in]  normMethod        The normalize method
 *
 * @tparam     PointT            Whatever point cloud form pcl
 *
 * @return     The fpfh keypoints.
 */
template<typename PointT, std::enable_if_t<pcl::traits::has_xyz<PointT>::value>* = nullptr>
std::pair<typename pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::FPFHSignature33>::Ptr> getFPFHKeypoints(  // NOLINT
    const typename pcl::PointCloud<PointT>::Ptr &inputCloud,
    const pcl::PointCloud<pcl::Normal>::Ptr &inputNormals,
    std::vector<float> scalesVector,
    float persintenceAlpha,
    pcl::NormType normMethod) {
    crf::utility::logger::EventLogger logger("getFPFHKeypoints");
    logger->debug("getFPFHKeypoints");

    if (inputCloud == nullptr || inputNormals == nullptr || inputCloud->size() < 1 ||
            inputNormals->size() < 1) {
        logger->error("Input Point Clouds not valid");
        return std::make_pair(nullptr, nullptr);
    }
    if (scalesVector.size() != 3) {
        logger->error("The scalesVector vector must have 3 elements");
        return std::make_pair(nullptr, nullptr);
    }
    for (unsigned int i = 0; i < scalesVector.size(); i++) {
        if (scalesVector.at(i) <= 0) {
            logger->error("The scale vector {} must be bigger than 0", i);
            return std::make_pair(nullptr, nullptr);
        }
    }
    if (persintenceAlpha <= 0) {
        logger->error("The persistance alpha must be bigger than 0");
        return std::make_pair(nullptr, nullptr);
    }
    typename pcl::PointCloud<PointT>::Ptr outputKeypoints(
        new typename pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr outputFeatures(
        new pcl::PointCloud<pcl::FPFHSignature33>);

    typename pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33>::Ptr fpfhEstimation(
        new typename pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33>);
    fpfhEstimation->setInputCloud(inputCloud);
    fpfhEstimation->setInputNormals(inputNormals);

    typename pcl::MultiscaleFeaturePersistence<PointT, pcl::FPFHSignature33> featurePersistence;
    pcl::IndicesPtr outputIndices(new std::vector<int>());
    featurePersistence.setScalesVector(scalesVector);
    featurePersistence.setAlpha(persintenceAlpha);
    featurePersistence.setFeatureEstimator(fpfhEstimation);
    featurePersistence.setDistanceMetric(normMethod);
    featurePersistence.determinePersistentFeatures(*outputFeatures, outputIndices);

    typename pcl::ExtractIndices<PointT> extractIndices;
    extractIndices.setInputCloud(inputCloud);
    extractIndices.setIndices(outputIndices);
    extractIndices.setNegative(false);
    extractIndices.filter(*outputKeypoints);
    return std::make_pair(outputKeypoints, outputFeatures);
}

/**
 * @brief      Gets the shot color keypoints.
 *
 * @param[in]  inputCloud          The input cloud
 * @param[in]  inputNormals        The input normals
 * @param[in]  computationThreads  The computation threads
 * @param[in]  scalesVector        The scales vector
 * @param[in]  persintenceAlpha    The persintence alpha
 * @param[in]  normMethod          The normalize method
 *
 * @tparam     PointT              Whatever point cloud form pcl
 *
 * @return     The shot color keypoints.
 */
template<typename PointT, std::enable_if_t<pcl::traits::has_xyz<PointT>::value && pcl::traits::has_color<PointT>::value>* = nullptr>  // NOLINT
std::pair<typename pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::SHOT1344>::Ptr> getSHOTColorKeypoints(  // NOLINT
    typename pcl::PointCloud<PointT>::Ptr inputCloud,
    pcl::PointCloud<pcl::Normal>::Ptr inputNormals,
    unsigned int computationThreads,
    std::vector<float> scalesVector,
    float persintenceAlpha,
    pcl::NormType normMethod) {
    crf::utility::logger::EventLogger logger("getSHOTColorKeypoints");
    logger->debug("getSHOTColorKeypoints");

    if (inputCloud == nullptr || inputCloud == nullptr || inputCloud->size() < 1 ||
            inputNormals->size() < 1) {
        logger->error("Input Point Clouds not valid");
        return std::make_pair(nullptr, nullptr);
    }
    if (computationThreads < 1) {
        logger->error("The persistance alpah must be bigger than 0");
        return std::make_pair(nullptr, nullptr);
    }
    for (unsigned int i = 0; i < scalesVector.size(); i++) {
        if (scalesVector.at(i) <= 0) {
            logger->error("The scale vector {} must be bigger than 0", i);
            return std::make_pair(nullptr, nullptr);
        }
    }
    if (persintenceAlpha <= 0) {
        logger->error("The persistance alpha must be bigger than 0");
        return std::make_pair(nullptr, nullptr);
    }
    typename pcl::PointCloud<PointT>::Ptr outputKeypoints(
        new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::SHOT1344>::Ptr outputFeatures(
        new pcl::PointCloud<pcl::SHOT1344>);

    typename pcl::SHOTColorEstimationOMP<PointT, pcl::Normal, pcl::SHOT1344>::Ptr shotEstimation(
        new pcl::SHOTColorEstimationOMP<PointT, pcl::Normal, pcl::SHOT1344>);
    shotEstimation->setInputCloud(inputCloud);
    shotEstimation->setInputNormals(inputNormals);
    shotEstimation->setNumberOfThreads(computationThreads);

    pcl::MultiscaleFeaturePersistence<PointT, pcl::SHOT1344> featurePersistence;
    pcl::IndicesPtr outputIndices(new std::vector<int>());
    featurePersistence.setScalesVector(scalesVector);
    featurePersistence.setAlpha(persintenceAlpha);
    featurePersistence.setFeatureEstimator(shotEstimation);
    featurePersistence.setDistanceMetric(normMethod);
    featurePersistence.determinePersistentFeatures(*outputFeatures, outputIndices);

    typename pcl::ExtractIndices<PointT> extractIndices;
    extractIndices.setInputCloud(inputCloud);
    extractIndices.setIndices(outputIndices);
    extractIndices.setNegative(false);
    extractIndices.filter(*outputKeypoints);
    return std::make_pair(outputKeypoints, outputFeatures);
}

}  // namespace keypoint
}  // namespace pointcloud
}  // namespace visionutility
}  // namespace utility
}  // namespace crf
