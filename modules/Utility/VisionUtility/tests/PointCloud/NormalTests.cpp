/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <pcl/features/integral_image_normal.h>

#include "EventLogger/EventLogger.hpp"
#include "VisionUtility/PointCloud/Normal.hpp"
#include "VisionUtility/PointCloud/Generator.hpp"

using crf::utility::visionutility::pointcloud::generator::uniformPointCloudGenerator;
using crf::utility::visionutility::pointcloud::normal::computeIntegralNormals;
using crf::utility::visionutility::pointcloud::normal::computeNormals;
using crf::utility::visionutility::pointcloud::normal::removeNaNNormals;

using testing::_;
using testing::Invoke;

class NormalShould: public ::testing::Test {
 protected:
    NormalShould(): logger_("NormalShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~NormalShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
};

TEST_F(NormalShould, checkComputeIntegralNormals) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_EQ(computeIntegralNormals<pcl::PointXYZRGBA>(cloud1,
        pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal>::
        NormalEstimationMethod::COVARIANCE_MATRIX, 0.1, 25.0, 0.05, 0.05, 0.05), nullptr);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 = uniformPointCloudGenerator<pcl::PointXYZRGBA>(
        300, 300, 0.1, 1.0, 0.1, 1.0, 0.1, 1.0);
    ASSERT_NE(computeIntegralNormals<pcl::PointXYZRGBA>(cloud2,
        pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal>::
        NormalEstimationMethod::COVARIANCE_MATRIX, 0.1, 25.0, 0.05, 0.05, 0.05), nullptr);
}

TEST_F(NormalShould, checkComputeNormals) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_EQ(computeNormals<pcl::PointXYZRGBA>(cloud1, 0.01), nullptr);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 = uniformPointCloudGenerator<pcl::PointXYZRGBA>(
        300, 300, 0.1, 1.0, 0.1, 1.0, 0.1, 1.0);
    ASSERT_NE(computeNormals<pcl::PointXYZRGBA>(cloud2, 0.01), nullptr);
}

TEST_F(NormalShould, checkRemoveNaNNormals) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals1 (new pcl::PointCloud<pcl::Normal>());
    auto output1 = removeNaNNormals<pcl::PointXYZRGBA>(cloud1, cloudNormals1);
    ASSERT_EQ(std::get<0>(output1), nullptr);
    ASSERT_EQ(std::get<1>(output1), nullptr);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 = uniformPointCloudGenerator<pcl::PointXYZRGBA>(
        300, 300, -1.0, 1.0, -1.0, 1.0, 0.5, 2.0);
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals2 = computeIntegralNormals<pcl::PointXYZRGBA>(
        cloud2, pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal>::
        NormalEstimationMethod::COVARIANCE_MATRIX,  0.1, 25.0, 0.05, 0.05, 0.05);
    ASSERT_NE(cloudNormals2, nullptr);

    auto output2 = removeNaNNormals<pcl::PointXYZRGBA>(cloud2, cloudNormals2);
    ASSERT_NE(std::get<0>(output2), nullptr);
    ASSERT_NE(std::get<1>(output2), nullptr);
}
