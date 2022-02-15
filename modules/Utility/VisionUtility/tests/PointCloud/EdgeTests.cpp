/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/random_sample.h>
#include <pcl/io/pcd_io.h>

#include "EventLogger/EventLogger.hpp"
#include "VisionUtility/PointCloud/Edge.hpp"
#include "VisionUtility/PointCloud/PointTypes.hpp"
#include "VisionUtility/PointCloud/Generator.hpp"

using testing::_;
using testing::Invoke;

using crf::utility::visionutility::pointcloud::edge::computeEigenSharpness;
using crf::utility::visionutility::pointcloud::pointtypes::PointXYZS;
using crf::utility::visionutility::pointcloud::generator::uniformPointCloudGenerator;

class EdgeShould: public ::testing::Test {
 protected:
    EdgeShould(): logger_("EdgeShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~EdgeShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
};

TEST_F(EdgeShould, checkComputeEigenSharpness) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input(new pcl::PointCloud<pcl::PointXYZRGBA>());
    input->width = 500;
    input->height = 1;
    input->is_dense = true;
    input->points.resize(input->width * input->height);
    for (std::size_t i = 0; i < input->points.size(); i++) {
        input->points[i].x = std::rand();
        input->points[i].y = std::rand();
        input->points[i].z = std::rand();
    }
    ASSERT_EQ(computeEigenSharpness<pcl::PointXYZRGBA>(nullptr, 20), nullptr);
    ASSERT_EQ(computeEigenSharpness<pcl::PointXYZRGBA>(input, 0), nullptr);
    ASSERT_NE(computeEigenSharpness<pcl::PointXYZRGBA>(input, 20), nullptr);
}
