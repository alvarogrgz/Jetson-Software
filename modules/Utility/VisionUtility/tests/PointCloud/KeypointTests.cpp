/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <pcl/common/io.h>
#include <Eigen/Dense>
#include <string>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "VisionUtility/PointCloud/Keypoint.hpp"
#include "VisionUtility/PointCloud/Generator.hpp"
#include "VisionUtility/PointCloud/Normal.hpp"
#include "VisionUtility/PointCloud/Filter.hpp"

using crf::utility::visionutility::pointcloud::generator::uniformPointCloudGenerator;
using crf::utility::visionutility::pointcloud::normal::computeIntegralNormals;
using crf::utility::visionutility::pointcloud::keypoint::getFPFHKeypoints;
using crf::utility::visionutility::pointcloud::keypoint::getSHOTColorKeypoints;
using crf::utility::visionutility::pointcloud::normal::removeNaNNormals;
using crf::utility::visionutility::pointcloud::filter::checkForArtifactsInPointcloud;
using crf::utility::visionutility::pointcloud::filter::filterInputPointcloud;

using testing::_;
using testing::Invoke;

class KeypointShould: public ::testing::Test {
 protected:
    KeypointShould(): logger_("KeypointShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        buildDirName_ = __FILE__;
        buildDirName_ = buildDirName_.substr(0,
            buildDirName_.find("tests/Utility/VisionUtilityTests/PointCloud/KeypointTests.cpp"));
        buildDirName_ += "build/";
    }
    ~KeypointShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::string buildDirName_;
};

TEST_F(KeypointShould, checkGetFPFHKeypoints) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals1 (new pcl::PointCloud<pcl::Normal>());
    std::vector<float> scales{0.1, 0.2, 0.3};
    auto output = getFPFHKeypoints<pcl::PointXYZ>(cloud1, cloudNormals1, scales, 1.3, pcl::CS);
    ASSERT_EQ(std::get<0>(output), nullptr);
    ASSERT_EQ(std::get<1>(output), nullptr);

    std::vector<float> scales1{-0.1, 0.2, 0.3};
    output = getFPFHKeypoints<pcl::PointXYZ>(cloud1, cloudNormals1, scales1, 1.3, pcl::CS);
    ASSERT_EQ(std::get<0>(output), nullptr);
    ASSERT_EQ(std::get<1>(output), nullptr);

    std::vector<float> scales2{0.1, -0.2, 0.3};
    output = getFPFHKeypoints<pcl::PointXYZ>(cloud1, cloudNormals1, scales2, 1.3, pcl::CS);
    ASSERT_EQ(std::get<0>(output), nullptr);
    ASSERT_EQ(std::get<1>(output), nullptr);

    std::vector<float> scales3{0.1, 0.2, -0.3};
    output = getFPFHKeypoints<pcl::PointXYZ>(cloud1, cloudNormals1, scales3, 1.3, pcl::CS);
    ASSERT_EQ(std::get<0>(output), nullptr);
    ASSERT_EQ(std::get<1>(output), nullptr);

    output = getFPFHKeypoints<pcl::PointXYZ>(cloud1, cloudNormals1, scales, -1.3, pcl::CS);
    ASSERT_EQ(std::get<0>(output), nullptr);
    ASSERT_EQ(std::get<1>(output), nullptr);
}
