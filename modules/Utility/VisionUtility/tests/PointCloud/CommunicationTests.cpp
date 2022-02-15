/* © Copyright CERN 2020.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@crf.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO
 *
 *  ==================================================================================================
*/

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <string>
#include <sstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>

#include "EventLogger/EventLogger.hpp"
#include "VisionUtility/PointCloud/Communication.hpp"
#include "VisionUtility/PointCloud/Generator.hpp"

using testing::_;
using testing::Invoke;

using crf::utility::visionutility::pointcloud::generator::uniformPointCloudGenerator;
using crf::utility::visionutility::pointcloud::communication::saveCloudinPlyFormat;
using crf::utility::visionutility::pointcloud::communication::serializePointCloud;

class CommunicationShould: public ::testing::Test {
 protected:
    CommunicationShould(): logger_("CommunicationShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~CommunicationShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
};

TEST_F(CommunicationShould, checkSaveInPlyFormat) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGBA>());
    std::string fileName{"test"};
    ASSERT_FALSE(saveCloudinPlyFormat<pcl::PointXYZRGBA>(cloud1, fileName));

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 = uniformPointCloudGenerator<pcl::PointXYZRGBA>(
        30, 30, 0.1, 1.0, 0.1, 1.0, 0.1, 1.0);
    ASSERT_TRUE(saveCloudinPlyFormat<pcl::PointXYZRGBA>(cloud2, fileName));
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2test (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_NE(pcl::io::loadPLYFile(fileName, *cloud2test), -1);

    ASSERT_TRUE(saveCloudinPlyFormat<pcl::PointXYZRGBA>(cloud2, fileName, true));
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2testBinary(
        new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_NE(pcl::io::loadPLYFile(fileName, *cloud2testBinary), -1);

    Eigen::Vector4f origin(0., 0., 0., 0.);
    Eigen::Quaternionf orientation(Eigen::Matrix3f::Identity());
    std::ostringstream oss = serializePointCloud<pcl::PointXYZRGBA>(
        cloud2, origin, orientation, true);

    ASSERT_NE(oss.str(), "Empty");

    std::fstream outFile {fileName, std::ios::out};
    ASSERT_TRUE(outFile.is_open());
    outFile << oss.str();
    outFile.close();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_NE(pcl::io::loadPLYFile(fileName, *cloud3), -1);
}
