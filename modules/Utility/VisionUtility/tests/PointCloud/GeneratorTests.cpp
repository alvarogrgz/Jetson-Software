/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "EventLogger/EventLogger.hpp"
#include "VisionUtility/PointCloud/Generator.hpp"

using testing::_;
using testing::Invoke;
using crf::utility::visionutility::pointcloud::generator::uniformPointCloudGenerator;
using crf::utility::visionutility::pointcloud::generator::normalPointCloudGenerator;

class GeneratorShould: public ::testing::Test {
 protected:
    GeneratorShould(): logger_("GeneratorShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~GeneratorShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
};

TEST_F(GeneratorShould, returnPointCloudWithDifferentPointTypesWithUniformPointCloudGenerator) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr randomXYZCloud =
        uniformPointCloudGenerator<pcl::PointXYZ>(100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_NE(randomXYZCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr randomXYZICloud =
        uniformPointCloudGenerator<pcl::PointXYZI>(100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_NE(randomXYZICloud, nullptr);

    pcl::PointCloud<pcl::PointXYZL>::Ptr randomXYZLCloud =
        uniformPointCloudGenerator<pcl::PointXYZL>(100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_NE(randomXYZLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr randomXYZRGBACloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBA>(100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_NE(randomXYZRGBACloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr randomXYZRGBCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGB>(100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_NE(randomXYZRGBCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr randomXYZRGBLCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBL>(100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_NE(randomXYZRGBLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr randomXYZHSVCloud =
        uniformPointCloudGenerator<pcl::PointXYZHSV>(100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_NE(randomXYZHSVCloud, nullptr);

    pcl::PointCloud<pcl::InterestPoint>::Ptr randomInterestPointCloud =
        uniformPointCloudGenerator<pcl::InterestPoint>(100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_NE(randomInterestPointCloud, nullptr);

    pcl::PointCloud<pcl::PointNormal>::Ptr randomNormalCloud =
        uniformPointCloudGenerator<pcl::PointNormal>(100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_NE(randomNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr randomXYZRGBNormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBNormal>(100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_NE(randomXYZRGBNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr randomXYZINormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZINormal>(100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_NE(randomXYZINormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr randomXYZLNormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZLNormal>(100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_NE(randomXYZLNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointWithRange>::Ptr randomWithRangeCloud =
        uniformPointCloudGenerator<pcl::PointWithRange>(100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_NE(randomWithRangeCloud, nullptr);

    pcl::PointCloud<pcl::PointWithViewpoint>::Ptr randomWithViewPointCloud =
        uniformPointCloudGenerator<pcl::PointWithViewpoint>(100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_NE(randomWithViewPointCloud, nullptr);

    pcl::PointCloud<pcl::PointWithScale>::Ptr randomWithScaleCloud =
        uniformPointCloudGenerator<pcl::PointWithScale>(100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_NE(randomWithScaleCloud, nullptr);

    pcl::PointCloud<pcl::PointSurfel>::Ptr randomSurfelCloud =
        uniformPointCloudGenerator<pcl::PointSurfel>(100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_NE(randomSurfelCloud, nullptr);

    pcl::PointCloud<pcl::PointDEM>::Ptr randomDEMCloud =
        uniformPointCloudGenerator<pcl::PointDEM>(100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_NE(randomDEMCloud, nullptr);
}

TEST_F(GeneratorShould, returnNullPtrIfInvalidWidthWithUniformPointCloudGenerator) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr randomXYZCloud =
        uniformPointCloudGenerator<pcl::PointXYZ>(-100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr randomXYZICloud =
        uniformPointCloudGenerator<pcl::PointXYZI>(-100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZICloud, nullptr);

    pcl::PointCloud<pcl::PointXYZL>::Ptr randomXYZLCloud =
        uniformPointCloudGenerator<pcl::PointXYZL>(-100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr randomXYZRGBACloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBA>(-100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZRGBACloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr randomXYZRGBCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGB>(-100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZRGBCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr randomXYZRGBLCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBL>(-100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZRGBLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr randomXYZHSVCloud =
        uniformPointCloudGenerator<pcl::PointXYZHSV>(-100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZHSVCloud, nullptr);

    pcl::PointCloud<pcl::InterestPoint>::Ptr randomInterestPointCloud =
        uniformPointCloudGenerator<pcl::InterestPoint>(-100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomInterestPointCloud, nullptr);

    pcl::PointCloud<pcl::PointNormal>::Ptr randomNormalCloud =
        uniformPointCloudGenerator<pcl::PointNormal>(-100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr randomXYZRGBNormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBNormal>(-100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZRGBNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr randomXYZINormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZINormal>(-100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZINormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr randomXYZLNormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZLNormal>(-100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZLNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointWithRange>::Ptr randomWithRangeCloud =
        uniformPointCloudGenerator<pcl::PointWithRange>(-100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomWithRangeCloud, nullptr);

    pcl::PointCloud<pcl::PointWithViewpoint>::Ptr randomWithViewPointCloud =
        uniformPointCloudGenerator<pcl::PointWithViewpoint>(-100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomWithViewPointCloud, nullptr);

    pcl::PointCloud<pcl::PointWithScale>::Ptr randomWithScaleCloud =
        uniformPointCloudGenerator<pcl::PointWithScale>(-100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomWithScaleCloud, nullptr);

    pcl::PointCloud<pcl::PointSurfel>::Ptr randomSurfelCloud =
        uniformPointCloudGenerator<pcl::PointSurfel>(-100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomSurfelCloud, nullptr);

    pcl::PointCloud<pcl::PointDEM>::Ptr randomDEMCloud =
        uniformPointCloudGenerator<pcl::PointDEM>(-100, 100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomDEMCloud, nullptr);
}

TEST_F(GeneratorShould, returnNullPtrIfInvalidHeightWithUniformPointCloudGenerator) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr randomXYZCloud =
        uniformPointCloudGenerator<pcl::PointXYZ>(100, -100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr randomXYZICloud =
        uniformPointCloudGenerator<pcl::PointXYZI>(100, -100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZICloud, nullptr);

    pcl::PointCloud<pcl::PointXYZL>::Ptr randomXYZLCloud =
        uniformPointCloudGenerator<pcl::PointXYZL>(100, -100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr randomXYZRGBACloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBA>(100, -100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZRGBACloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr randomXYZRGBCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGB>(100, -100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZRGBCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr randomXYZRGBLCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBL>(100, -100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZRGBLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr randomXYZHSVCloud =
        uniformPointCloudGenerator<pcl::PointXYZHSV>(100, -100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZHSVCloud, nullptr);

    pcl::PointCloud<pcl::InterestPoint>::Ptr randomInterestPointCloud =
        uniformPointCloudGenerator<pcl::InterestPoint>(100, -100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomInterestPointCloud, nullptr);

    pcl::PointCloud<pcl::PointNormal>::Ptr randomNormalCloud =
        uniformPointCloudGenerator<pcl::PointNormal>(100, -100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr randomXYZRGBNormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBNormal>(100, -100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZRGBNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr randomXYZINormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZINormal>(100, -100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZINormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr randomXYZLNormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZLNormal>(100, -100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZLNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointWithRange>::Ptr randomWithRangeCloud =
        uniformPointCloudGenerator<pcl::PointWithRange>(100, -100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomWithRangeCloud, nullptr);

    pcl::PointCloud<pcl::PointWithViewpoint>::Ptr randomWithViewPointCloud =
        uniformPointCloudGenerator<pcl::PointWithViewpoint>(100, -100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomWithViewPointCloud, nullptr);

    pcl::PointCloud<pcl::PointWithScale>::Ptr randomWithScaleCloud =
        uniformPointCloudGenerator<pcl::PointWithScale>(100, -100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomWithScaleCloud, nullptr);

    pcl::PointCloud<pcl::PointSurfel>::Ptr randomSurfelCloud =
        uniformPointCloudGenerator<pcl::PointSurfel>(100, -100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomSurfelCloud, nullptr);

    pcl::PointCloud<pcl::PointDEM>::Ptr randomDEMCloud =
        uniformPointCloudGenerator<pcl::PointDEM>(100, -100, 0, 10, 0, 10, 0, 10);
    ASSERT_EQ(randomDEMCloud, nullptr);
}

TEST_F(GeneratorShould, returnNullPtrIfInvalidXLimitWithUniformPointCloudGenerator) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr randomXYZCloud =
        uniformPointCloudGenerator<pcl::PointXYZ>(100, 100, 0, -10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr randomXYZICloud =
        uniformPointCloudGenerator<pcl::PointXYZI>(100, 100, 0, -10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZICloud, nullptr);

    pcl::PointCloud<pcl::PointXYZL>::Ptr randomXYZLCloud =
        uniformPointCloudGenerator<pcl::PointXYZL>(100, 100, 0, -10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr randomXYZRGBACloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBA>(100, 100, 0, -10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZRGBACloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr randomXYZRGBCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGB>(100, 100, 0, -10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZRGBCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr randomXYZRGBLCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBL>(100, 100, 0, -10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZRGBLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr randomXYZHSVCloud =
        uniformPointCloudGenerator<pcl::PointXYZHSV>(100, 100, 0, -10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZHSVCloud, nullptr);

    pcl::PointCloud<pcl::InterestPoint>::Ptr randomInterestPointCloud =
        uniformPointCloudGenerator<pcl::InterestPoint>(100, 100, 0, -10, 0, 10, 0, 10);
    ASSERT_EQ(randomInterestPointCloud, nullptr);

    pcl::PointCloud<pcl::PointNormal>::Ptr randomNormalCloud =
        uniformPointCloudGenerator<pcl::PointNormal>(100, 100, 0, -10, 0, 10, 0, 10);
    ASSERT_EQ(randomNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr randomXYZRGBNormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBNormal>(100, 100, 0, -10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZRGBNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr randomXYZINormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZINormal>(100, 100, 0, -10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZINormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr randomXYZLNormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZLNormal>(100, 100, 0, -10, 0, 10, 0, 10);
    ASSERT_EQ(randomXYZLNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointWithRange>::Ptr randomWithRangeCloud =
        uniformPointCloudGenerator<pcl::PointWithRange>(100, 100, 0, -10, 0, 10, 0, 10);
    ASSERT_EQ(randomWithRangeCloud, nullptr);

    pcl::PointCloud<pcl::PointWithViewpoint>::Ptr randomWithViewPointCloud =
        uniformPointCloudGenerator<pcl::PointWithViewpoint>(100, 100, 0, -10, 0, 10, 0, 10);
    ASSERT_EQ(randomWithViewPointCloud, nullptr);

    pcl::PointCloud<pcl::PointWithScale>::Ptr randomWithScaleCloud =
        uniformPointCloudGenerator<pcl::PointWithScale>(100, 100, 0, -10, 0, 10, 0, 10);
    ASSERT_EQ(randomWithScaleCloud, nullptr);

    pcl::PointCloud<pcl::PointSurfel>::Ptr randomSurfelCloud =
        uniformPointCloudGenerator<pcl::PointSurfel>(100, 100, 0, -10, 0, 10, 0, 10);
    ASSERT_EQ(randomSurfelCloud, nullptr);

    pcl::PointCloud<pcl::PointDEM>::Ptr randomDEMCloud =
        uniformPointCloudGenerator<pcl::PointDEM>(100, 100, 0, -10, 0, 10, 0, 10);
    ASSERT_EQ(randomDEMCloud, nullptr);
}

TEST_F(GeneratorShould, returnNullPtrIfInvalidYLimitWithUniformPointCloudGenerator) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr randomXYZCloud =
        uniformPointCloudGenerator<pcl::PointXYZ>(100, 100, 0, 10, 0, -10, 0, 10);
    ASSERT_EQ(randomXYZCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr randomXYZICloud =
        uniformPointCloudGenerator<pcl::PointXYZI>(100, 100, 0, 10, 0, -10, 0, 10);
    ASSERT_EQ(randomXYZICloud, nullptr);

    pcl::PointCloud<pcl::PointXYZL>::Ptr randomXYZLCloud =
        uniformPointCloudGenerator<pcl::PointXYZL>(100, 100, 0, 10, 0, -10, 0, 10);
    ASSERT_EQ(randomXYZLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr randomXYZRGBACloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBA>(100, 100, 0, 10, 0, -10, 0, 10);
    ASSERT_EQ(randomXYZRGBACloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr randomXYZRGBCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGB>(100, 100, 0, 10, 0, -10, 0, 10);
    ASSERT_EQ(randomXYZRGBCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr randomXYZRGBLCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBL>(100, 100, 0, 10, 0, -10, 0, 10);
    ASSERT_EQ(randomXYZRGBLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr randomXYZHSVCloud =
        uniformPointCloudGenerator<pcl::PointXYZHSV>(100, 100, 0, 10, 0, -10, 0, 10);
    ASSERT_EQ(randomXYZHSVCloud, nullptr);

    pcl::PointCloud<pcl::InterestPoint>::Ptr randomInterestPointCloud =
        uniformPointCloudGenerator<pcl::InterestPoint>(100, 100, 0, 10, 0, -10, 0, 10);
    ASSERT_EQ(randomInterestPointCloud, nullptr);

    pcl::PointCloud<pcl::PointNormal>::Ptr randomNormalCloud =
        uniformPointCloudGenerator<pcl::PointNormal>(100, 100, 0, 10, 0, -10, 0, 10);
    ASSERT_EQ(randomNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr randomXYZRGBNormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBNormal>(100, 100, 0, 10, 0, -10, 0, 10);
    ASSERT_EQ(randomXYZRGBNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr randomXYZINormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZINormal>(100, 100, 0, 10, 0, -10, 0, 10);
    ASSERT_EQ(randomXYZINormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr randomXYZLNormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZLNormal>(100, 100, 0, 10, 0, -10, 0, 10);
    ASSERT_EQ(randomXYZLNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointWithRange>::Ptr randomWithRangeCloud =
        uniformPointCloudGenerator<pcl::PointWithRange>(100, 100, 0, 10, 0, -10, 0, 10);
    ASSERT_EQ(randomWithRangeCloud, nullptr);

    pcl::PointCloud<pcl::PointWithViewpoint>::Ptr randomWithViewPointCloud =
        uniformPointCloudGenerator<pcl::PointWithViewpoint>(100, 100, 0, 10, 0, -10, 0, 10);
    ASSERT_EQ(randomWithViewPointCloud, nullptr);

    pcl::PointCloud<pcl::PointWithScale>::Ptr randomWithScaleCloud =
        uniformPointCloudGenerator<pcl::PointWithScale>(100, 100, 0, 10, 0, -10, 0, 10);
    ASSERT_EQ(randomWithScaleCloud, nullptr);

    pcl::PointCloud<pcl::PointSurfel>::Ptr randomSurfelCloud =
        uniformPointCloudGenerator<pcl::PointSurfel>(100, 100, 0, 10, 0, -10, 0, 10);
    ASSERT_EQ(randomSurfelCloud, nullptr);

    pcl::PointCloud<pcl::PointDEM>::Ptr randomDEMCloud =
        uniformPointCloudGenerator<pcl::PointDEM>(100, 100, 0, 10, 0, -10, 0, 10);
    ASSERT_EQ(randomDEMCloud, nullptr);
}

TEST_F(GeneratorShould, returnNullPtrIfInvalidZLimitWithUniformPointCloudGenerator) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr randomXYZCloud =
        uniformPointCloudGenerator<pcl::PointXYZ>(100, 100, 0, 10, 0, 10, 0, -10);
    ASSERT_EQ(randomXYZCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr randomXYZICloud =
        uniformPointCloudGenerator<pcl::PointXYZI>(100, 100, 0, 10, 0, 10, 0, -10);
    ASSERT_EQ(randomXYZICloud, nullptr);

    pcl::PointCloud<pcl::PointXYZL>::Ptr randomXYZLCloud =
        uniformPointCloudGenerator<pcl::PointXYZL>(100, 100, 0, 10, 0, 10, 0, -10);
    ASSERT_EQ(randomXYZLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr randomXYZRGBACloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBA>(100, 100, 0, 10, 0, 10, 0, -10);
    ASSERT_EQ(randomXYZRGBACloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr randomXYZRGBCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGB>(100, 100, 0, 10, 0, 10, 0, -10);
    ASSERT_EQ(randomXYZRGBCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr randomXYZRGBLCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBL>(100, 100, 0, 10, 0, 10, 0, -10);
    ASSERT_EQ(randomXYZRGBLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr randomXYZHSVCloud =
        uniformPointCloudGenerator<pcl::PointXYZHSV>(100, 100, 0, 10, 0, 10, 0, -10);
    ASSERT_EQ(randomXYZHSVCloud, nullptr);

    pcl::PointCloud<pcl::InterestPoint>::Ptr randomInterestPointCloud =
        uniformPointCloudGenerator<pcl::InterestPoint>(100, 100, 0, 10, 0, 10, 0, -10);
    ASSERT_EQ(randomInterestPointCloud, nullptr);

    pcl::PointCloud<pcl::PointNormal>::Ptr randomNormalCloud =
        uniformPointCloudGenerator<pcl::PointNormal>(100, 100, 0, 10, 0, 10, 0, -10);
    ASSERT_EQ(randomNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr randomXYZRGBNormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBNormal>(100, 100, 0, 10, 0, 10, 0, -10);
    ASSERT_EQ(randomXYZRGBNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr randomXYZINormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZINormal>(100, 100, 0, 10, 0, 10, 0, -10);
    ASSERT_EQ(randomXYZINormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr randomXYZLNormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZLNormal>(100, 100, 0, 10, 0, 10, 0, -10);
    ASSERT_EQ(randomXYZLNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointWithRange>::Ptr randomWithRangeCloud =
        uniformPointCloudGenerator<pcl::PointWithRange>(100, 100, 0, 10, 0, 10, 0, -10);
    ASSERT_EQ(randomWithRangeCloud, nullptr);

    pcl::PointCloud<pcl::PointWithViewpoint>::Ptr randomWithViewPointCloud =
        uniformPointCloudGenerator<pcl::PointWithViewpoint>(100, 100, 0, 10, 0, 10, 0, -10);
    ASSERT_EQ(randomWithViewPointCloud, nullptr);

    pcl::PointCloud<pcl::PointWithScale>::Ptr randomWithScaleCloud =
        uniformPointCloudGenerator<pcl::PointWithScale>(100, 100, 0, 10, 0, 10, 0, -10);
    ASSERT_EQ(randomWithScaleCloud, nullptr);

    pcl::PointCloud<pcl::PointSurfel>::Ptr randomSurfelCloud =
        uniformPointCloudGenerator<pcl::PointSurfel>(100, 100, 0, 10, 0, 10, 0, -10);
    ASSERT_EQ(randomSurfelCloud, nullptr);

    pcl::PointCloud<pcl::PointDEM>::Ptr randomDEMCloud =
        uniformPointCloudGenerator<pcl::PointDEM>(100, 100, 0, 10, 0, 10, 0, -10);
    ASSERT_EQ(randomDEMCloud, nullptr);
}

TEST_F(GeneratorShould, returnPointCloudWithDifferentPointTypesWithNormalPointCloudGenerator) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr randomXYZCloud =
        normalPointCloudGenerator<pcl::PointXYZ>(100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_NE(randomXYZCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr randomXYZICloud =
        normalPointCloudGenerator<pcl::PointXYZI>(100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_NE(randomXYZICloud, nullptr);

    pcl::PointCloud<pcl::PointXYZL>::Ptr randomXYZLCloud =
        normalPointCloudGenerator<pcl::PointXYZL>(100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_NE(randomXYZLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr randomXYZRGBACloud =
        normalPointCloudGenerator<pcl::PointXYZRGBA>(100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_NE(randomXYZRGBACloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr randomXYZRGBCloud =
        normalPointCloudGenerator<pcl::PointXYZRGB>(100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_NE(randomXYZRGBCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr randomXYZRGBLCloud =
        normalPointCloudGenerator<pcl::PointXYZRGBL>(100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_NE(randomXYZRGBLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr randomXYZHSVCloud =
        normalPointCloudGenerator<pcl::PointXYZHSV>(100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_NE(randomXYZHSVCloud, nullptr);

    pcl::PointCloud<pcl::InterestPoint>::Ptr randomInterestPointCloud =
        normalPointCloudGenerator<pcl::InterestPoint>(100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_NE(randomInterestPointCloud, nullptr);

    pcl::PointCloud<pcl::PointNormal>::Ptr randomNormalCloud =
        normalPointCloudGenerator<pcl::PointNormal>(100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_NE(randomNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr randomXYZRGBNormalCloud =
        normalPointCloudGenerator<pcl::PointXYZRGBNormal>(100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_NE(randomXYZRGBNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr randomXYZINormalCloud =
        normalPointCloudGenerator<pcl::PointXYZINormal>(100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_NE(randomXYZINormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr randomXYZLNormalCloud =
        normalPointCloudGenerator<pcl::PointXYZLNormal>(100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_NE(randomXYZLNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointWithRange>::Ptr randomWithRangeCloud =
        normalPointCloudGenerator<pcl::PointWithRange>(100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_NE(randomWithRangeCloud, nullptr);

    pcl::PointCloud<pcl::PointWithViewpoint>::Ptr randomWithViewPointCloud =
        normalPointCloudGenerator<pcl::PointWithViewpoint>(100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_NE(randomWithViewPointCloud, nullptr);

    pcl::PointCloud<pcl::PointWithScale>::Ptr randomWithScaleCloud =
        normalPointCloudGenerator<pcl::PointWithScale>(100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_NE(randomWithScaleCloud, nullptr);

    pcl::PointCloud<pcl::PointSurfel>::Ptr randomSurfelCloud =
        normalPointCloudGenerator<pcl::PointSurfel>(100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_NE(randomSurfelCloud, nullptr);

    pcl::PointCloud<pcl::PointDEM>::Ptr randomDEMCloud =
        normalPointCloudGenerator<pcl::PointDEM>(100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_NE(randomDEMCloud, nullptr);
}

TEST_F(GeneratorShould, returnNullPtrIfInvalidWidthWithNormalPointCloudGenerator) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr randomXYZCloud =
        normalPointCloudGenerator<pcl::PointXYZ>(-100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr randomXYZICloud =
        normalPointCloudGenerator<pcl::PointXYZI>(-100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZICloud, nullptr);

    pcl::PointCloud<pcl::PointXYZL>::Ptr randomXYZLCloud =
        normalPointCloudGenerator<pcl::PointXYZL>(-100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr randomXYZRGBACloud =
        normalPointCloudGenerator<pcl::PointXYZRGBA>(-100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZRGBACloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr randomXYZRGBCloud =
        normalPointCloudGenerator<pcl::PointXYZRGB>(-100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZRGBCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr randomXYZRGBLCloud =
        normalPointCloudGenerator<pcl::PointXYZRGBL>(-100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZRGBLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr randomXYZHSVCloud =
        normalPointCloudGenerator<pcl::PointXYZHSV>(-100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZHSVCloud, nullptr);

    pcl::PointCloud<pcl::InterestPoint>::Ptr randomInterestPointCloud =
        normalPointCloudGenerator<pcl::InterestPoint>(-100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomInterestPointCloud, nullptr);

    pcl::PointCloud<pcl::PointNormal>::Ptr randomNormalCloud =
        normalPointCloudGenerator<pcl::PointNormal>(-100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr randomXYZRGBNormalCloud =
        normalPointCloudGenerator<pcl::PointXYZRGBNormal>(-100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZRGBNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr randomXYZINormalCloud =
        normalPointCloudGenerator<pcl::PointXYZINormal>(-100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZINormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr randomXYZLNormalCloud =
        normalPointCloudGenerator<pcl::PointXYZLNormal>(-100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZLNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointWithRange>::Ptr randomWithRangeCloud =
        normalPointCloudGenerator<pcl::PointWithRange>(-100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomWithRangeCloud, nullptr);

    pcl::PointCloud<pcl::PointWithViewpoint>::Ptr randomWithViewPointCloud =
        normalPointCloudGenerator<pcl::PointWithViewpoint>(-100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomWithViewPointCloud, nullptr);

    pcl::PointCloud<pcl::PointWithScale>::Ptr randomWithScaleCloud =
        normalPointCloudGenerator<pcl::PointWithScale>(-100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomWithScaleCloud, nullptr);

    pcl::PointCloud<pcl::PointSurfel>::Ptr randomSurfelCloud =
        normalPointCloudGenerator<pcl::PointSurfel>(-100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomSurfelCloud, nullptr);

    pcl::PointCloud<pcl::PointDEM>::Ptr randomDEMCloud =
        normalPointCloudGenerator<pcl::PointDEM>(-100, 100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomDEMCloud, nullptr);
}

TEST_F(GeneratorShould, returnNullPtrIfInvalidHeightWithNormalPointCloudGenerator) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr randomXYZCloud =
        normalPointCloudGenerator<pcl::PointXYZ>(100, -100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr randomXYZICloud =
        normalPointCloudGenerator<pcl::PointXYZI>(100, -100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZICloud, nullptr);

    pcl::PointCloud<pcl::PointXYZL>::Ptr randomXYZLCloud =
        normalPointCloudGenerator<pcl::PointXYZL>(100, -100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr randomXYZRGBACloud =
        normalPointCloudGenerator<pcl::PointXYZRGBA>(100, -100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZRGBACloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr randomXYZRGBCloud =
        normalPointCloudGenerator<pcl::PointXYZRGB>(100, -100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZRGBCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr randomXYZRGBLCloud =
        normalPointCloudGenerator<pcl::PointXYZRGBL>(100, -100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZRGBLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr randomXYZHSVCloud =
        normalPointCloudGenerator<pcl::PointXYZHSV>(100, -100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZHSVCloud, nullptr);

    pcl::PointCloud<pcl::InterestPoint>::Ptr randomInterestPointCloud =
        normalPointCloudGenerator<pcl::InterestPoint>(100, -100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomInterestPointCloud, nullptr);

    pcl::PointCloud<pcl::PointNormal>::Ptr randomNormalCloud =
        normalPointCloudGenerator<pcl::PointNormal>(100, -100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr randomXYZRGBNormalCloud =
        normalPointCloudGenerator<pcl::PointXYZRGBNormal>(100, -100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZRGBNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr randomXYZINormalCloud =
        normalPointCloudGenerator<pcl::PointXYZINormal>(100, -100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZINormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr randomXYZLNormalCloud =
        normalPointCloudGenerator<pcl::PointXYZLNormal>(100, -100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZLNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointWithRange>::Ptr randomWithRangeCloud =
        normalPointCloudGenerator<pcl::PointWithRange>(100, -100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomWithRangeCloud, nullptr);

    pcl::PointCloud<pcl::PointWithViewpoint>::Ptr randomWithViewPointCloud =
        normalPointCloudGenerator<pcl::PointWithViewpoint>(100, -100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomWithViewPointCloud, nullptr);

    pcl::PointCloud<pcl::PointWithScale>::Ptr randomWithScaleCloud =
        normalPointCloudGenerator<pcl::PointWithScale>(100, -100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomWithScaleCloud, nullptr);

    pcl::PointCloud<pcl::PointSurfel>::Ptr randomSurfelCloud =
        normalPointCloudGenerator<pcl::PointSurfel>(100, -100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomSurfelCloud, nullptr);

    pcl::PointCloud<pcl::PointDEM>::Ptr randomDEMCloud =
        normalPointCloudGenerator<pcl::PointDEM>(100, -100, 0, 1, 0, 1, 0, 1);
    ASSERT_EQ(randomDEMCloud, nullptr);
}

TEST_F(GeneratorShould, returnNullPtrIfInvalidStdDevXWithNormalPointCloudGenerator) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr randomXYZCloud =
        normalPointCloudGenerator<pcl::PointXYZ>(100, 100, 0, -1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr randomXYZICloud =
        normalPointCloudGenerator<pcl::PointXYZI>(100, 100, 0, -1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZICloud, nullptr);

    pcl::PointCloud<pcl::PointXYZL>::Ptr randomXYZLCloud =
        normalPointCloudGenerator<pcl::PointXYZL>(100, 100, 0, -1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr randomXYZRGBACloud =
        normalPointCloudGenerator<pcl::PointXYZRGBA>(100, 100, 0, -1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZRGBACloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr randomXYZRGBCloud =
        normalPointCloudGenerator<pcl::PointXYZRGB>(100, 100, 0, -1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZRGBCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr randomXYZRGBLCloud =
        normalPointCloudGenerator<pcl::PointXYZRGBL>(100, 100, 0, -1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZRGBLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr randomXYZHSVCloud =
        normalPointCloudGenerator<pcl::PointXYZHSV>(100, 100, 0, -1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZHSVCloud, nullptr);

    pcl::PointCloud<pcl::InterestPoint>::Ptr randomInterestPointCloud =
        normalPointCloudGenerator<pcl::InterestPoint>(100, 100, 0, -1, 0, 1, 0, 1);
    ASSERT_EQ(randomInterestPointCloud, nullptr);

    pcl::PointCloud<pcl::PointNormal>::Ptr randomNormalCloud =
        normalPointCloudGenerator<pcl::PointNormal>(100, 100, 0, -1, 0, 1, 0, 1);
    ASSERT_EQ(randomNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr randomXYZRGBNormalCloud =
        normalPointCloudGenerator<pcl::PointXYZRGBNormal>(100, 100, 0, -1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZRGBNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr randomXYZINormalCloud =
        normalPointCloudGenerator<pcl::PointXYZINormal>(100, 100, 0, -1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZINormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr randomXYZLNormalCloud =
        normalPointCloudGenerator<pcl::PointXYZLNormal>(100, 100, 0, -1, 0, 1, 0, 1);
    ASSERT_EQ(randomXYZLNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointWithRange>::Ptr randomWithRangeCloud =
        normalPointCloudGenerator<pcl::PointWithRange>(100, 100, 0, -1, 0, 1, 0, 1);
    ASSERT_EQ(randomWithRangeCloud, nullptr);

    pcl::PointCloud<pcl::PointWithViewpoint>::Ptr randomWithViewPointCloud =
        normalPointCloudGenerator<pcl::PointWithViewpoint>(100, 100, 0, -1, 0, 1, 0, 1);
    ASSERT_EQ(randomWithViewPointCloud, nullptr);

    pcl::PointCloud<pcl::PointWithScale>::Ptr randomWithScaleCloud =
        normalPointCloudGenerator<pcl::PointWithScale>(100, 100, 0, -1, 0, 1, 0, 1);
    ASSERT_EQ(randomWithScaleCloud, nullptr);

    pcl::PointCloud<pcl::PointSurfel>::Ptr randomSurfelCloud =
        normalPointCloudGenerator<pcl::PointSurfel>(100, 100, 0, -1, 0, 1, 0, 1);
    ASSERT_EQ(randomSurfelCloud, nullptr);

    pcl::PointCloud<pcl::PointDEM>::Ptr randomDEMCloud =
        normalPointCloudGenerator<pcl::PointDEM>(100, 100, 0, -1, 0, 1, 0, 1);
    ASSERT_EQ(randomDEMCloud, nullptr);
}

TEST_F(GeneratorShould, returnNullPtrIfInvalidStdDevYWithNormalPointCloudGenerator) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr randomXYZCloud =
        normalPointCloudGenerator<pcl::PointXYZ>(100, 100, 0, 1, 0, -1, 0, 1);
    ASSERT_EQ(randomXYZCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr randomXYZICloud =
        normalPointCloudGenerator<pcl::PointXYZI>(100, 100, 0, 1, 0, -1, 0, 1);
    ASSERT_EQ(randomXYZICloud, nullptr);

    pcl::PointCloud<pcl::PointXYZL>::Ptr randomXYZLCloud =
        normalPointCloudGenerator<pcl::PointXYZL>(100, 100, 0, 1, 0, -1, 0, 1);
    ASSERT_EQ(randomXYZLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr randomXYZRGBACloud =
        normalPointCloudGenerator<pcl::PointXYZRGBA>(100, 100, 0, 1, 0, -1, 0, 1);
    ASSERT_EQ(randomXYZRGBACloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr randomXYZRGBCloud =
        normalPointCloudGenerator<pcl::PointXYZRGB>(100, 100, 0, 1, 0, -1, 0, 1);
    ASSERT_EQ(randomXYZRGBCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr randomXYZRGBLCloud =
        normalPointCloudGenerator<pcl::PointXYZRGBL>(100, 100, 0, 1, 0, -1, 0, 1);
    ASSERT_EQ(randomXYZRGBLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr randomXYZHSVCloud =
        normalPointCloudGenerator<pcl::PointXYZHSV>(100, 100, 0, 1, 0, -1, 0, 1);
    ASSERT_EQ(randomXYZHSVCloud, nullptr);

    pcl::PointCloud<pcl::InterestPoint>::Ptr randomInterestPointCloud =
        normalPointCloudGenerator<pcl::InterestPoint>(100, 100, 0, 1, 0, -1, 0, 1);
    ASSERT_EQ(randomInterestPointCloud, nullptr);

    pcl::PointCloud<pcl::PointNormal>::Ptr randomNormalCloud =
        normalPointCloudGenerator<pcl::PointNormal>(100, 100, 0, 1, 0, -1, 0, 1);
    ASSERT_EQ(randomNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr randomXYZRGBNormalCloud =
        normalPointCloudGenerator<pcl::PointXYZRGBNormal>(100, 100, 0, 1, 0, -1, 0, 1);
    ASSERT_EQ(randomXYZRGBNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr randomXYZINormalCloud =
        normalPointCloudGenerator<pcl::PointXYZINormal>(100, 100, 0, 1, 0, -1, 0, 1);
    ASSERT_EQ(randomXYZINormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr randomXYZLNormalCloud =
        normalPointCloudGenerator<pcl::PointXYZLNormal>(100, 100, 0, 1, 0, -1, 0, 1);
    ASSERT_EQ(randomXYZLNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointWithRange>::Ptr randomWithRangeCloud =
        normalPointCloudGenerator<pcl::PointWithRange>(100, 100, 0, 1, 0, -1, 0, 1);
    ASSERT_EQ(randomWithRangeCloud, nullptr);

    pcl::PointCloud<pcl::PointWithViewpoint>::Ptr randomWithViewPointCloud =
        normalPointCloudGenerator<pcl::PointWithViewpoint>(100, 100, 0, 1, 0, -1, 0, 1);
    ASSERT_EQ(randomWithViewPointCloud, nullptr);

    pcl::PointCloud<pcl::PointWithScale>::Ptr randomWithScaleCloud =
        normalPointCloudGenerator<pcl::PointWithScale>(100, 100, 0, 1, 0, -1, 0, 1);
    ASSERT_EQ(randomWithScaleCloud, nullptr);

    pcl::PointCloud<pcl::PointSurfel>::Ptr randomSurfelCloud =
        normalPointCloudGenerator<pcl::PointSurfel>(100, 100, 0, 1, 0, -1, 0, 1);
    ASSERT_EQ(randomSurfelCloud, nullptr);

    pcl::PointCloud<pcl::PointDEM>::Ptr randomDEMCloud =
        normalPointCloudGenerator<pcl::PointDEM>(100, 100, 0, 1, 0, -1, 0, 1);
    ASSERT_EQ(randomDEMCloud, nullptr);
}

TEST_F(GeneratorShould, returnNullPtrIfInvalidStdDevZWithNormalPointCloudGenerator) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr randomXYZCloud =
        normalPointCloudGenerator<pcl::PointXYZ>(100, 100, 0, 1, 0, 1, 0, -1);
    ASSERT_EQ(randomXYZCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr randomXYZICloud =
        normalPointCloudGenerator<pcl::PointXYZI>(100, 100, 0, 1, 0, 1, 0, -1);
    ASSERT_EQ(randomXYZICloud, nullptr);

    pcl::PointCloud<pcl::PointXYZL>::Ptr randomXYZLCloud =
        normalPointCloudGenerator<pcl::PointXYZL>(100, 100, 0, 1, 0, 1, 0, -1);
    ASSERT_EQ(randomXYZLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr randomXYZRGBACloud =
        normalPointCloudGenerator<pcl::PointXYZRGBA>(100, 100, 0, 1, 0, 1, 0, -1);
    ASSERT_EQ(randomXYZRGBACloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr randomXYZRGBCloud =
        normalPointCloudGenerator<pcl::PointXYZRGB>(100, 100, 0, 1, 0, 1, 0, -1);
    ASSERT_EQ(randomXYZRGBCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr randomXYZRGBLCloud =
        normalPointCloudGenerator<pcl::PointXYZRGBL>(100, 100, 0, 1, 0, 1, 0, -1);
    ASSERT_EQ(randomXYZRGBLCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr randomXYZHSVCloud =
        normalPointCloudGenerator<pcl::PointXYZHSV>(100, 100, 0, 1, 0, 1, 0, -1);
    ASSERT_EQ(randomXYZHSVCloud, nullptr);

    pcl::PointCloud<pcl::InterestPoint>::Ptr randomInterestPointCloud =
        normalPointCloudGenerator<pcl::InterestPoint>(100, 100, 0, 1, 0, 1, 0, -1);
    ASSERT_EQ(randomInterestPointCloud, nullptr);

    pcl::PointCloud<pcl::PointNormal>::Ptr randomNormalCloud =
        normalPointCloudGenerator<pcl::PointNormal>(100, 100, 0, 1, 0, 1, 0, -1);
    ASSERT_EQ(randomNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr randomXYZRGBNormalCloud =
        normalPointCloudGenerator<pcl::PointXYZRGBNormal>(100, 100, 0, 1, 0, 1, 0, -1);
    ASSERT_EQ(randomXYZRGBNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr randomXYZINormalCloud =
        normalPointCloudGenerator<pcl::PointXYZINormal>(100, 100, 0, 1, 0, 1, 0, -1);
    ASSERT_EQ(randomXYZINormalCloud, nullptr);

    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr randomXYZLNormalCloud =
        normalPointCloudGenerator<pcl::PointXYZLNormal>(100, 100, 0, 1, 0, 1, 0, -1);
    ASSERT_EQ(randomXYZLNormalCloud, nullptr);

    pcl::PointCloud<pcl::PointWithRange>::Ptr randomWithRangeCloud =
        normalPointCloudGenerator<pcl::PointWithRange>(100, 100, 0, 1, 0, 1, 0, -1);
    ASSERT_EQ(randomWithRangeCloud, nullptr);

    pcl::PointCloud<pcl::PointWithViewpoint>::Ptr randomWithViewPointCloud =
        normalPointCloudGenerator<pcl::PointWithViewpoint>(100, 100, 0, 1, 0, 1, 0, -1);
    ASSERT_EQ(randomWithViewPointCloud, nullptr);

    pcl::PointCloud<pcl::PointWithScale>::Ptr randomWithScaleCloud =
        normalPointCloudGenerator<pcl::PointWithScale>(100, 100, 0, 1, 0, 1, 0, -1);
    ASSERT_EQ(randomWithScaleCloud, nullptr);

    pcl::PointCloud<pcl::PointSurfel>::Ptr randomSurfelCloud =
        normalPointCloudGenerator<pcl::PointSurfel>(100, 100, 0, 1, 0, 1, 0, -1);
    ASSERT_EQ(randomSurfelCloud, nullptr);

    pcl::PointCloud<pcl::PointDEM>::Ptr randomDEMCloud =
        normalPointCloudGenerator<pcl::PointDEM>(100, 100, 0, 1, 0, 1, 0, -1);
    ASSERT_EQ(randomDEMCloud, nullptr);
}
