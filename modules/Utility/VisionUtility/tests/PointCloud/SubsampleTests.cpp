/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO 2019
 *         Alejandro Diaz Rosales CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <string>

#include "EventLogger/EventLogger.hpp"
#include "VisionUtility/PointCloud/Generator.hpp"
#include "VisionUtility/PointCloud/Subsample.hpp"
#include "VisionUtility/PointCloud/Normal.hpp"

using testing::_;
using testing::Invoke;
using crf::utility::visionutility::pointcloud::generator::uniformPointCloudGenerator;
using crf::utility::visionutility::pointcloud::subsample::voxelGridSubsample;
using crf::utility::visionutility::pointcloud::subsample::randomSubsampling;
using crf::utility::visionutility::pointcloud::subsample::normalsSubsampling;
using crf::utility::visionutility::pointcloud::subsample::focusInAreaOfInterest;
using crf::utility::visionutility::pointcloud::normal::computeIntegralNormals;

class SubsampleShould: public ::testing::Test {
 protected:
    SubsampleShould(): logger_("SubsampleShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~SubsampleShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
};

TEST_F(SubsampleShould, returnValidPointCloudWithDifferentPointTypesWithVoxelGridSubsample) {
    std::vector<float> sideLength;
    sideLength.push_back(0.01);
    sideLength.push_back(0.01);
    sideLength.push_back(0.01);

    pcl::PointCloud<pcl::PointXYZ>::Ptr randomXYZCloud =
        uniformPointCloudGenerator<pcl::PointXYZ>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputXYZ =
        voxelGridSubsample<pcl::PointXYZ>(randomXYZCloud, sideLength);
    ASSERT_NE(outputXYZ, nullptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr randomXYZICloud =
        uniformPointCloudGenerator<pcl::PointXYZI>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointXYZI>::Ptr outputXYZI =
        voxelGridSubsample<pcl::PointXYZI>(randomXYZICloud, sideLength);
    ASSERT_NE(outputXYZI, nullptr);

    pcl::PointCloud<pcl::PointXYZL>::Ptr randomXYZLCloud =
        uniformPointCloudGenerator<pcl::PointXYZL>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointXYZL>::Ptr outputXYZL =
        voxelGridSubsample<pcl::PointXYZL>(randomXYZLCloud, sideLength);
    ASSERT_NE(outputXYZL, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr randomXYZRGBACloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBA>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outputRGBA =
        voxelGridSubsample<pcl::PointXYZRGBA>(randomXYZRGBACloud, sideLength);
    ASSERT_NE(outputRGBA, nullptr);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr randomXYZRGBCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGB>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputXYZRGB =
        voxelGridSubsample<pcl::PointXYZRGB>(randomXYZRGBCloud, sideLength);
    ASSERT_NE(outputXYZRGB, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr randomXYZRGBLCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBL>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr outputXYZRGBL =
        voxelGridSubsample<pcl::PointXYZRGBL>(randomXYZRGBLCloud, sideLength);
    ASSERT_NE(outputXYZRGBL, nullptr);

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr randomXYZHSVCloud =
        uniformPointCloudGenerator<pcl::PointXYZHSV>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr outputXYZHSV =
        voxelGridSubsample<pcl::PointXYZHSV>(randomXYZHSVCloud, sideLength);
    ASSERT_NE(outputXYZHSV, nullptr);

    pcl::PointCloud<pcl::InterestPoint>::Ptr randomInterestPointCloud =
        uniformPointCloudGenerator<pcl::InterestPoint>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::InterestPoint>::Ptr outputInterestPoint =
        voxelGridSubsample<pcl::InterestPoint>(randomInterestPointCloud, sideLength);
    ASSERT_NE(outputInterestPoint, nullptr);

    pcl::PointCloud<pcl::PointNormal>::Ptr randomNormalCloud =
        uniformPointCloudGenerator<pcl::PointNormal>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointNormal>::Ptr outputNormal =
        voxelGridSubsample<pcl::PointNormal>(randomNormalCloud, sideLength);
    ASSERT_NE(outputNormal, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr randomXYZRGBNormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBNormal>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputXYZRGBNormal =
        voxelGridSubsample<pcl::PointXYZRGBNormal>(randomXYZRGBNormalCloud, sideLength);
    ASSERT_NE(outputXYZRGBNormal, nullptr);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr randomXYZINormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZINormal>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr outputXYZINormal =
        voxelGridSubsample<pcl::PointXYZINormal>(randomXYZINormalCloud, sideLength);
    ASSERT_NE(outputXYZINormal, nullptr);

    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr randomXYZLNormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZLNormal>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr outputXYZLNormal =
        voxelGridSubsample<pcl::PointXYZLNormal>(randomXYZLNormalCloud, sideLength);
    ASSERT_NE(outputXYZLNormal, nullptr);

    pcl::PointCloud<pcl::PointWithRange>::Ptr randomWithRangeCloud =
        uniformPointCloudGenerator<pcl::PointWithRange>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointWithRange>::Ptr outputWithRange =
        voxelGridSubsample<pcl::PointWithRange>(randomWithRangeCloud, sideLength);
    ASSERT_NE(outputWithRange, nullptr);

    pcl::PointCloud<pcl::PointWithViewpoint>::Ptr randomWithViewPointCloud =
        uniformPointCloudGenerator<pcl::PointWithViewpoint>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointWithViewpoint>::Ptr outputWithViewPoint =
        voxelGridSubsample<pcl::PointWithViewpoint>(randomWithViewPointCloud, sideLength);
    ASSERT_NE(outputWithViewPoint, nullptr);

    pcl::PointCloud<pcl::PointWithScale>::Ptr randomWithScaleCloud =
        uniformPointCloudGenerator<pcl::PointWithScale>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointWithScale>::Ptr outputWithScale =
        voxelGridSubsample<pcl::PointWithScale>(randomWithScaleCloud, sideLength);
    ASSERT_NE(outputWithScale, nullptr);

    pcl::PointCloud<pcl::PointSurfel>::Ptr randomSurfelCloud =
        uniformPointCloudGenerator<pcl::PointSurfel>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointSurfel>::Ptr outputSurfel =
        voxelGridSubsample<pcl::PointSurfel>(randomSurfelCloud, sideLength);
    ASSERT_NE(outputSurfel, nullptr);

    pcl::PointCloud<pcl::PointDEM>::Ptr randomDEMCloud =
        uniformPointCloudGenerator<pcl::PointDEM>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointDEM>::Ptr outputDEM =
        voxelGridSubsample<pcl::PointDEM>(randomDEMCloud, sideLength);
    ASSERT_NE(outputDEM, nullptr);
}

TEST_F(SubsampleShould, returnNullPtrIfSideLengtIsInvalidWithVoxelGridSubsample) {
    std::vector<float> invalidSideLength1;
    invalidSideLength1.push_back(0.01);
    invalidSideLength1.push_back(-0.01);
    invalidSideLength1.push_back(0.01);

    std::vector<float> invalidSideLength2;
    invalidSideLength2.push_back(0.01);
    invalidSideLength2.push_back(0.01);
    invalidSideLength2.push_back(0.01);
    invalidSideLength2.push_back(0.01);

    pcl::PointCloud<pcl::PointXYZ>::Ptr randomXYZCloud =
        uniformPointCloudGenerator<pcl::PointXYZ>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputXYZ1 =
        voxelGridSubsample<pcl::PointXYZ>(randomXYZCloud, invalidSideLength1);
    ASSERT_EQ(outputXYZ1, nullptr);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputXYZ2 =
        voxelGridSubsample<pcl::PointXYZ>(randomXYZCloud, invalidSideLength2);
    ASSERT_EQ(outputXYZ2, nullptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr randomXYZICloud =
        uniformPointCloudGenerator<pcl::PointXYZI>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointXYZI>::Ptr outputXYZI1 =
        voxelGridSubsample<pcl::PointXYZI>(randomXYZICloud, invalidSideLength1);
    ASSERT_EQ(outputXYZI1, nullptr);
    pcl::PointCloud<pcl::PointXYZI>::Ptr outputXYZI2 =
        voxelGridSubsample<pcl::PointXYZI>(randomXYZICloud, invalidSideLength2);
    ASSERT_EQ(outputXYZI2, nullptr);

    pcl::PointCloud<pcl::PointXYZL>::Ptr randomXYZLCloud =
        uniformPointCloudGenerator<pcl::PointXYZL>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointXYZL>::Ptr outputXYZL1 =
        voxelGridSubsample<pcl::PointXYZL>(randomXYZLCloud, invalidSideLength1);
    ASSERT_EQ(outputXYZL1, nullptr);
    pcl::PointCloud<pcl::PointXYZL>::Ptr outputXYZL2 =
        voxelGridSubsample<pcl::PointXYZL>(randomXYZLCloud, invalidSideLength2);
    ASSERT_EQ(outputXYZL2, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr randomXYZRGBACloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBA>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outputRGBA1 =
        voxelGridSubsample<pcl::PointXYZRGBA>(randomXYZRGBACloud, invalidSideLength1);
    ASSERT_EQ(outputRGBA1, nullptr);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outputRGBA2 =
        voxelGridSubsample<pcl::PointXYZRGBA>(randomXYZRGBACloud, invalidSideLength2);
    ASSERT_EQ(outputRGBA2, nullptr);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr randomXYZRGBCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGB>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputXYZRGB1 =
        voxelGridSubsample<pcl::PointXYZRGB>(randomXYZRGBCloud, invalidSideLength1);
    ASSERT_EQ(outputXYZRGB1, nullptr);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputXYZRGB2 =
        voxelGridSubsample<pcl::PointXYZRGB>(randomXYZRGBCloud, invalidSideLength2);
    ASSERT_EQ(outputXYZRGB2, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr randomXYZRGBLCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBL>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr outputXYZRGBL1 =
        voxelGridSubsample<pcl::PointXYZRGBL>(randomXYZRGBLCloud, invalidSideLength1);
    ASSERT_EQ(outputXYZRGBL1, nullptr);
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr outputXYZRGBL2 =
        voxelGridSubsample<pcl::PointXYZRGBL>(randomXYZRGBLCloud, invalidSideLength2);
    ASSERT_EQ(outputXYZRGBL2, nullptr);

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr randomXYZHSVCloud =
        uniformPointCloudGenerator<pcl::PointXYZHSV>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr outputXYZHSV1 =
        voxelGridSubsample<pcl::PointXYZHSV>(randomXYZHSVCloud, invalidSideLength1);
    ASSERT_EQ(outputXYZHSV1, nullptr);
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr outputXYZHSV2 =
        voxelGridSubsample<pcl::PointXYZHSV>(randomXYZHSVCloud, invalidSideLength2);
    ASSERT_EQ(outputXYZHSV2, nullptr);

    pcl::PointCloud<pcl::InterestPoint>::Ptr randomInterestPointCloud =
        uniformPointCloudGenerator<pcl::InterestPoint>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::InterestPoint>::Ptr outputInterestPoint1 =
        voxelGridSubsample<pcl::InterestPoint>(randomInterestPointCloud, invalidSideLength1);
    ASSERT_EQ(outputInterestPoint1, nullptr);
    pcl::PointCloud<pcl::InterestPoint>::Ptr outputInterestPoint2 =
        voxelGridSubsample<pcl::InterestPoint>(randomInterestPointCloud, invalidSideLength2);
    ASSERT_EQ(outputInterestPoint2, nullptr);

    pcl::PointCloud<pcl::PointNormal>::Ptr randomNormalCloud =
        uniformPointCloudGenerator<pcl::PointNormal>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointNormal>::Ptr outputNormal1 =
        voxelGridSubsample<pcl::PointNormal>(randomNormalCloud, invalidSideLength1);
    ASSERT_EQ(outputNormal1, nullptr);
    pcl::PointCloud<pcl::PointNormal>::Ptr outputNormal2 =
        voxelGridSubsample<pcl::PointNormal>(randomNormalCloud, invalidSideLength2);
    ASSERT_EQ(outputNormal2, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr randomXYZRGBNormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZRGBNormal>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputXYZRGBNormal1 =
        voxelGridSubsample<pcl::PointXYZRGBNormal>(randomXYZRGBNormalCloud, invalidSideLength1);
    ASSERT_EQ(outputXYZRGBNormal1, nullptr);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputXYZRGBNormal2 =
        voxelGridSubsample<pcl::PointXYZRGBNormal>(randomXYZRGBNormalCloud, invalidSideLength2);
    ASSERT_EQ(outputXYZRGBNormal2, nullptr);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr randomXYZINormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZINormal>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr outputXYZINormal1 =
        voxelGridSubsample<pcl::PointXYZINormal>(randomXYZINormalCloud, invalidSideLength1);
    ASSERT_EQ(outputXYZINormal1, nullptr);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr outputXYZINormal2 =
        voxelGridSubsample<pcl::PointXYZINormal>(randomXYZINormalCloud, invalidSideLength2);
    ASSERT_EQ(outputXYZINormal2, nullptr);

    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr randomXYZLNormalCloud =
        uniformPointCloudGenerator<pcl::PointXYZLNormal>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr outputXYZLNormal1 =
        voxelGridSubsample<pcl::PointXYZLNormal>(randomXYZLNormalCloud, invalidSideLength1);
    ASSERT_EQ(outputXYZLNormal1, nullptr);
    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr outputXYZLNormal2 =
        voxelGridSubsample<pcl::PointXYZLNormal>(randomXYZLNormalCloud, invalidSideLength2);
    ASSERT_EQ(outputXYZLNormal2, nullptr);

    pcl::PointCloud<pcl::PointWithRange>::Ptr randomWithRangeCloud =
        uniformPointCloudGenerator<pcl::PointWithRange>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointWithRange>::Ptr outputWithRange1 =
        voxelGridSubsample<pcl::PointWithRange>(randomWithRangeCloud, invalidSideLength1);
    ASSERT_EQ(outputWithRange1, nullptr);
    pcl::PointCloud<pcl::PointWithRange>::Ptr outputWithRange2 =
        voxelGridSubsample<pcl::PointWithRange>(randomWithRangeCloud, invalidSideLength2);
    ASSERT_EQ(outputWithRange2, nullptr);

    pcl::PointCloud<pcl::PointWithViewpoint>::Ptr randomWithViewPointCloud =
        uniformPointCloudGenerator<pcl::PointWithViewpoint>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointWithViewpoint>::Ptr outputWithViewPoint1 =
        voxelGridSubsample<pcl::PointWithViewpoint>(randomWithViewPointCloud, invalidSideLength1);
    ASSERT_EQ(outputWithViewPoint1, nullptr);
    pcl::PointCloud<pcl::PointWithViewpoint>::Ptr outputWithViewPoint2 =
        voxelGridSubsample<pcl::PointWithViewpoint>(randomWithViewPointCloud, invalidSideLength2);
    ASSERT_EQ(outputWithViewPoint2, nullptr);

    pcl::PointCloud<pcl::PointWithScale>::Ptr randomWithScaleCloud =
        uniformPointCloudGenerator<pcl::PointWithScale>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointWithScale>::Ptr outputWithScale1 =
        voxelGridSubsample<pcl::PointWithScale>(randomWithScaleCloud, invalidSideLength1);
    ASSERT_EQ(outputWithScale1, nullptr);
    pcl::PointCloud<pcl::PointWithScale>::Ptr outputWithScale2 =
        voxelGridSubsample<pcl::PointWithScale>(randomWithScaleCloud, invalidSideLength2);
    ASSERT_EQ(outputWithScale2, nullptr);

    pcl::PointCloud<pcl::PointSurfel>::Ptr randomSurfelCloud =
        uniformPointCloudGenerator<pcl::PointSurfel>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointSurfel>::Ptr outputSurfel1 =
        voxelGridSubsample<pcl::PointSurfel>(randomSurfelCloud, invalidSideLength1);
    ASSERT_EQ(outputSurfel1, nullptr);
    pcl::PointCloud<pcl::PointSurfel>::Ptr outputSurfel2 =
        voxelGridSubsample<pcl::PointSurfel>(randomSurfelCloud, invalidSideLength2);
    ASSERT_EQ(outputSurfel2, nullptr);

    pcl::PointCloud<pcl::PointDEM>::Ptr randomDEMCloud =
        uniformPointCloudGenerator<pcl::PointDEM>(100, 100, 0, 10, 0, 10, 0, 10);
    pcl::PointCloud<pcl::PointDEM>::Ptr outputDEM1 =
        voxelGridSubsample<pcl::PointDEM>(randomDEMCloud, invalidSideLength1);
    ASSERT_EQ(outputDEM1, nullptr);
    pcl::PointCloud<pcl::PointDEM>::Ptr outputDEM2 =
        voxelGridSubsample<pcl::PointDEM>(randomDEMCloud, invalidSideLength2);
    ASSERT_EQ(outputDEM2, nullptr);
}

TEST_F(SubsampleShould, returnNullPtrIfInputIsNullWithVoxelGridSubsample) {
    std::vector<float> sideLength;
    sideLength.push_back(0.01);
    sideLength.push_back(0.01);
    sideLength.push_back(0.01);

    pcl::PointCloud<pcl::PointXYZ>::Ptr outputXYZ =
        voxelGridSubsample<pcl::PointXYZ>(nullptr, sideLength);
    ASSERT_EQ(outputXYZ, nullptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr outputXYZI =
        voxelGridSubsample<pcl::PointXYZI>(nullptr, sideLength);
    ASSERT_EQ(outputXYZI, nullptr);

    pcl::PointCloud<pcl::PointXYZL>::Ptr outputXYZL =
        voxelGridSubsample<pcl::PointXYZL>(nullptr, sideLength);
    ASSERT_EQ(outputXYZL, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outputRGBA =
        voxelGridSubsample<pcl::PointXYZRGBA>(nullptr, sideLength);
    ASSERT_EQ(outputRGBA, nullptr);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputXYZRGB =
        voxelGridSubsample<pcl::PointXYZRGB>(nullptr, sideLength);
    ASSERT_EQ(outputXYZRGB, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr outputXYZRGBL =
        voxelGridSubsample<pcl::PointXYZRGBL>(nullptr, sideLength);
    ASSERT_EQ(outputXYZRGBL, nullptr);

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr outputXYZHSV =
        voxelGridSubsample<pcl::PointXYZHSV>(nullptr, sideLength);
    ASSERT_EQ(outputXYZHSV, nullptr);

    pcl::PointCloud<pcl::InterestPoint>::Ptr outputInterestPoint =
        voxelGridSubsample<pcl::InterestPoint>(nullptr, sideLength);
    ASSERT_EQ(outputInterestPoint, nullptr);

    pcl::PointCloud<pcl::PointNormal>::Ptr outputNormal =
        voxelGridSubsample<pcl::PointNormal>(nullptr, sideLength);
    ASSERT_EQ(outputNormal, nullptr);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputXYZRGBNormal =
        voxelGridSubsample<pcl::PointXYZRGBNormal>(nullptr, sideLength);
    ASSERT_EQ(outputXYZRGBNormal, nullptr);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr outputXYZINormal =
        voxelGridSubsample<pcl::PointXYZINormal>(nullptr, sideLength);
    ASSERT_EQ(outputXYZINormal, nullptr);

    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr outputXYZLNormal =
        voxelGridSubsample<pcl::PointXYZLNormal>(nullptr, sideLength);
    ASSERT_EQ(outputXYZLNormal, nullptr);

    pcl::PointCloud<pcl::PointWithRange>::Ptr outputWithRange =
        voxelGridSubsample<pcl::PointWithRange>(nullptr, sideLength);
    ASSERT_EQ(outputWithRange, nullptr);

    pcl::PointCloud<pcl::PointWithViewpoint>::Ptr outputWithViewPoint =
        voxelGridSubsample<pcl::PointWithViewpoint>(nullptr, sideLength);
    ASSERT_EQ(outputWithViewPoint, nullptr);

    pcl::PointCloud<pcl::PointWithScale>::Ptr outputWithScale =
        voxelGridSubsample<pcl::PointWithScale>(nullptr, sideLength);
    ASSERT_EQ(outputWithScale, nullptr);

    pcl::PointCloud<pcl::PointSurfel>::Ptr outputSurfel =
        voxelGridSubsample<pcl::PointSurfel>(nullptr, sideLength);
    ASSERT_EQ(outputSurfel, nullptr);

    pcl::PointCloud<pcl::PointDEM>::Ptr outputDEM =
        voxelGridSubsample<pcl::PointDEM>(nullptr, sideLength);
    ASSERT_EQ(outputDEM, nullptr);
}

TEST_F(SubsampleShould, checkRandomSubsample) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals1 (new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>());
    std::string buildDirName_ = __FILE__;
    buildDirName_ = buildDirName_.substr(0,
        buildDirName_.find("cpproboticframework"));
    buildDirName_ += "cpproboticframework/bin/testfiles/";
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (buildDirName_ + "OrganizedPointCloud.pcd",
        *cloud2), -1);

    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals2 = computeIntegralNormals<pcl::PointXYZRGBA>(
        cloud2, pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal>::
        NormalEstimationMethod::COVARIANCE_MATRIX,  0.1, 25.0, 0.05, 0.05, 0.05);
    ASSERT_NE(cloudNormals2, nullptr);

    // Remove NaN from source cloud
    std::vector<int> sourceInd;
    pcl::PointCloud<pcl::Normal>::Ptr normal3 (new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::removeNaNNormalsFromPointCloud(*cloudNormals2, *normal3, sourceInd);
    for (int i = 0; i < static_cast<int>(sourceInd.size()); i++) {
        cloud3->push_back(cloud2->points[sourceInd[i]]);
    }

    auto output = randomSubsampling<pcl::PointXYZRGBA>(cloud1, cloudNormals1, 50);
    ASSERT_EQ(std::get<0>(output), nullptr);
    ASSERT_EQ(std::get<1>(output), nullptr);

    output = randomSubsampling<pcl::PointXYZRGBA>(cloud2, cloudNormals1, 50);
    ASSERT_EQ(std::get<0>(output), nullptr);
    ASSERT_EQ(std::get<1>(output), nullptr);

    output = randomSubsampling<pcl::PointXYZRGBA>(cloud1, cloudNormals2, 50);
    ASSERT_EQ(std::get<0>(output), nullptr);
    ASSERT_EQ(std::get<1>(output), nullptr);

    output = randomSubsampling<pcl::PointXYZRGBA>(cloud2, cloudNormals2, -1);
    ASSERT_EQ(std::get<0>(output), nullptr);
    ASSERT_EQ(std::get<1>(output), nullptr);

    output = randomSubsampling<pcl::PointXYZRGBA>(cloud3, normal3, 50);
    ASSERT_NE(std::get<0>(output), nullptr);
    ASSERT_NE(std::get<1>(output), nullptr);
}

TEST_F(SubsampleShould, checkNormalSubsample) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals1 (new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>());
    std::string buildDirName_ = __FILE__;
    buildDirName_ = buildDirName_.substr(0,
        buildDirName_.find("cpproboticframework"));
    buildDirName_ += "cpproboticframework/bin/testfiles/";
    ASSERT_NE(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (buildDirName_ + "OrganizedPointCloud.pcd",
        *cloud2), -1);
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals2 = computeIntegralNormals<pcl::PointXYZRGBA>(
        cloud2, pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal>::
        NormalEstimationMethod::COVARIANCE_MATRIX,  0.1, 25.0, 0.05, 0.05, 0.05);
    ASSERT_NE(cloudNormals2, nullptr);

    // Remove NaN from source cloud
    std::vector<int> sourceInd;
    pcl::PointCloud<pcl::Normal>::Ptr normal3 (new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::removeNaNNormalsFromPointCloud(*cloudNormals2, *normal3, sourceInd);
    for (int i = 0; i < static_cast<int>(sourceInd.size()); i++) {
        cloud3->push_back(cloud2->points[sourceInd[i]]);
    }

    auto output = normalsSubsampling<pcl::PointXYZRGBA>(cloud1, cloudNormals1, 50, 16);
    ASSERT_EQ(std::get<0>(output), nullptr);
    ASSERT_EQ(std::get<1>(output), nullptr);

    output = normalsSubsampling<pcl::PointXYZRGBA>(cloud2, cloudNormals1, 50, 16);
    ASSERT_EQ(std::get<0>(output), nullptr);
    ASSERT_EQ(std::get<1>(output), nullptr);

    output = normalsSubsampling<pcl::PointXYZRGBA>(cloud1, cloudNormals2, 50, 16);
    ASSERT_EQ(std::get<0>(output), nullptr);
    ASSERT_EQ(std::get<1>(output), nullptr);

    output = normalsSubsampling<pcl::PointXYZRGBA>(cloud2, cloudNormals2, -1, 16);
    ASSERT_EQ(std::get<0>(output), nullptr);
    ASSERT_EQ(std::get<1>(output), nullptr);

    output = normalsSubsampling<pcl::PointXYZRGBA>(cloud2, cloudNormals2, 50, -16);
    ASSERT_EQ(std::get<0>(output), nullptr);
    ASSERT_EQ(std::get<1>(output), nullptr);

    output = normalsSubsampling<pcl::PointXYZRGBA>(cloud3, normal3, 50, 16);
    ASSERT_NE(std::get<0>(output), nullptr);
    ASSERT_NE(std::get<1>(output), nullptr);
}

TEST_F(SubsampleShould, checkFocusInAreaOfInterest) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud2 = uniformPointCloudGenerator<pcl::PointXYZRGBA>(200, 200, -1., 1., -1., 1., 1., 3.);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr focus =
        focusInAreaOfInterest<pcl::PointXYZRGBA>(cloud1, 10.0, 10.0, -10.0, -10.0, 0.02, 0.05);
    ASSERT_EQ(focus, nullptr);

    focus = focusInAreaOfInterest<pcl::PointXYZRGBA>(cloud2, -10.0, 10.0, -10.0, -10.0, 0.02, 0.05);
    ASSERT_EQ(focus, nullptr);

    focus = focusInAreaOfInterest<pcl::PointXYZRGBA>(cloud2, 10.0, -10.0, -10.0, -10.0, 0.02, 0.05);
    ASSERT_EQ(focus, nullptr);

    focus = focusInAreaOfInterest<pcl::PointXYZRGBA>(cloud2, -10.0, 0.0, -10.0, -10.0, -0.02, 0.05);
    ASSERT_EQ(focus, nullptr);

    focus = focusInAreaOfInterest<pcl::PointXYZRGBA>(cloud2, -10.0, 10.0, -10.0, -10.0, 0.02, -0.5);
    ASSERT_EQ(focus, nullptr);

    focus = focusInAreaOfInterest<pcl::PointXYZRGBA>(cloud2, 10.0, 10.0, -10.0, -10.0, 0.02, 0.05);
    ASSERT_NE(focus, nullptr);
}

