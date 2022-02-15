/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO 2020
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
#include "VisionUtility/PointCloud/Filter.hpp"
#include "VisionUtility/PointCloud/Generator.hpp"

using testing::_;
using testing::Invoke;

using crf::utility::visionutility::pointcloud::filter::basedRangeFilter;
using crf::utility::visionutility::pointcloud::filter::cutPointCloud;
using crf::utility::visionutility::pointcloud::filter::checkForArtifactsInPointcloud;
using crf::utility::visionutility::pointcloud::filter::filterInputPointcloud;
using crf::utility::visionutility::pointcloud::generator::uniformPointCloudGenerator;

class FilterShould: public ::testing::Test {
 protected:
    FilterShould(): logger_("FilterShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~FilterShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
};

TEST_F(FilterShould, checkBasedFilter) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_EQ(basedRangeFilter<pcl::PointXYZRGBA>(cloud1, 0.01, 1.0), nullptr);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 = uniformPointCloudGenerator<pcl::PointXYZRGBA>(
        300, 300, 0.1, 1.0, 0.1, 1.0, 0.1, 1.0);
    ASSERT_EQ(basedRangeFilter<pcl::PointXYZRGBA>(cloud2, 0.01, -5.0), nullptr);
    ASSERT_EQ(basedRangeFilter<pcl::PointXYZRGBA>(cloud2, -0.01, 1.0), nullptr);
    ASSERT_EQ(basedRangeFilter<pcl::PointXYZRGBA>(cloud2, 1.01, 1.0), nullptr);
    ASSERT_NE(basedRangeFilter<pcl::PointXYZRGBA>(cloud2, 0.01, 10.0), nullptr);
}

TEST_F(FilterShould, checkCutPointCloud) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_EQ(cutPointCloud<pcl::PointXYZRGBA>(cloud1, 1.0, 1.0, 1.0, 1.0), nullptr);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 = uniformPointCloudGenerator<pcl::PointXYZRGBA>(
        300, 300, 0.1, 1.0, 0.1, 1.0, 0.1, 1.0);
    ASSERT_NE(cutPointCloud<pcl::PointXYZRGBA>(cloud2, 1.0, 1.0, 1.0, 1.0), nullptr);
    ASSERT_NE(cutPointCloud<pcl::PointXYZRGBA>(cloud2, 1.0, 1.0, 1.0, 1.0, false), nullptr);
}

TEST_F(FilterShould, checkCheckForArtifactsInPointcloud) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_EQ(checkForArtifactsInPointcloud<pcl::PointXYZRGBA>(cloud1, 0.01), nullptr);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 = uniformPointCloudGenerator<pcl::PointXYZRGBA>(
        300, 300, 0.1, 1.0, 0.1, 1.0, 0.1, 1.0);
    ASSERT_EQ(checkForArtifactsInPointcloud<pcl::PointXYZRGBA>(cloud2, -1.0), nullptr);
    ASSERT_NE(checkForArtifactsInPointcloud<pcl::PointXYZRGBA>(cloud2, 0.01), nullptr);
}

TEST_F(FilterShould, checkFilterInputPointcloud) {
    crf::utility::visionutility::pointcloud::filter::FiltersParameters filtersData;
    filtersData.passThrough = true;
    filtersData.statisticalOutlier = false;
    filtersData.fastBilateral = false;
    filtersData.passThroughMinLimit = 0.4;
    filtersData.passThroughMaxLimit = 3.0;
    filtersData.statisticalOutlierMeanK = 2.5;
    filtersData.statisticalOutlierStddevMulThresh = 2.5;
    filtersData.fastBilateralSigmaS = 5;
    filtersData.fastBilateralSigmaR = 0.005;


    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGBA>());
    ASSERT_EQ(filterInputPointcloud<pcl::PointXYZRGBA>(cloud1, filtersData), nullptr);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 = uniformPointCloudGenerator<pcl::PointXYZRGBA>(
        300, 300, 0.1, 1.0, 0.1, 1.0, 0.1, 1.0);
    ASSERT_NE(filterInputPointcloud<pcl::PointXYZRGBA>(cloud2, filtersData), nullptr);
}
