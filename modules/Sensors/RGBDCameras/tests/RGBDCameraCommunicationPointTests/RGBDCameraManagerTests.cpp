/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */


#include <future>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "RGBDCameras/RGBDCameraCommunicationPoint/RGBDCameraManager.hpp"
#include "VisionUtility/Image/ImageJSONConverter.hpp"
#include "../IRGBDCameraSimulator.hpp"
#include "EventLogger/EventLogger.hpp"
#include "RGBDCameras/RGBDCameraMock.hpp"


using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::sensors::rgbdcameras::RGBDCameraMock;
using crf::sensors::rgbdcameras::RGBDCameraManager;

class RGBDCameraManagerShould : public ::testing::Test {
 protected:
    RGBDCameraManagerShould() :
        logger_("RGBDCameraManagerShould"),
        mock_(new NiceMock<RGBDCameraMock>),
        simulator_(new IRGBDCameraSimulator(mock_)) {
            logger_->info("{0} BEGIN",
                testing::UnitTest::GetInstance()->current_test_info()->name());
            simulator_->initializeDelay = std::chrono::milliseconds(1);
    }

    ~RGBDCameraManagerShould() {
        logger_->info("{0} END with {1}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;

    std::shared_ptr<RGBDCameraMock> mock_;
    std::unique_ptr<IRGBDCameraSimulator> simulator_;
    std::unique_ptr<RGBDCameraManager> manager_;
};


TEST_F(RGBDCameraManagerShould, getFrameFailsIfNotRequestedBefore) {
    manager_.reset(new RGBDCameraManager(mock_));
    EXPECT_THROW({
        try{
            auto mat = manager_->getFrame(0);
        }
        catch(const std::runtime_error& err) {
            EXPECT_STREQ(err.what(), "Frame grabber is not running" );
            throw;
        }
    }, std::runtime_error);
}

TEST_F(RGBDCameraManagerShould, getFrameFailsIfRequestedSocketIsNotActive) {
    manager_.reset(new RGBDCameraManager(mock_));

    ASSERT_TRUE(manager_->startFrameStream(0, true, false, cv::Size(1280, 720), 30, cv::Size(640, 480), 30));
    EXPECT_THROW({
        try{
            auto mat = manager_->getFrame(1);
        }
        catch(const std::runtime_error& err) {
            EXPECT_STREQ(err.what(), "Stream is not registered in manager" );
            throw;
        }
    }, std::runtime_error);
}


TEST_F(RGBDCameraManagerShould, cantRequestResolutionWithOneDimensionEqualTo0) {
    manager_.reset(new RGBDCameraManager(mock_));

    EXPECT_THROW({
        try{
            manager_->startFrameStream(0, true, false, cv::Size(0, 720), 30, cv::Size(640, 480), 30);
        }
        catch(const std::invalid_argument& err) {
            EXPECT_STREQ(err.what(), "Can't request a resolution with a dimension equal to 0" );
            throw;
        }
    }, std::invalid_argument);

    EXPECT_THROW({
        try{
            manager_->startFrameStream(0, true, false, cv::Size(1280, 0), 30, cv::Size(640, 480), 30);
        }
        catch(const std::invalid_argument& err) {
            EXPECT_STREQ(err.what(), "Can't request a resolution with a dimension equal to 0" );
            throw;
        }
    }, std::invalid_argument);

    EXPECT_THROW({
        try{
            manager_->startFrameStream(0, true, false, cv::Size(1280, 720), 30, cv::Size(0, 480), 30);
        }
        catch(const std::invalid_argument& err) {
            EXPECT_STREQ(err.what(), "Can't request a resolution with a dimension equal to 0" );
            throw;
        }
    }, std::invalid_argument);

    EXPECT_THROW({
        try{
            manager_->startFrameStream(0, true, false, cv::Size(1280, 720), 30, cv::Size(640, 0), 30);
        }
        catch(const std::invalid_argument& err) {
            EXPECT_STREQ(err.what(), "Can't request a resolution with a dimension equal to 0" );
            throw;
        }
    }, std::invalid_argument);
}


TEST_F(RGBDCameraManagerShould, correctlyGetFrameAndUpdateResolution) {
    manager_.reset(new RGBDCameraManager(mock_));

    ASSERT_TRUE(manager_->startFrameStream(0, true, false, cv::Size(1280, 720), 30, cv::Size(1280, 720), 30));
    auto mat = manager_->getFrame(0);

    ASSERT_EQ(mat.rows, 720);
    ASSERT_EQ(mat.cols, 1280);

    auto rgbd = manager_->getRGBDFrame(0);
    ASSERT_EQ(rgbd.image.rows, 720);
    ASSERT_EQ(rgbd.image.cols, 1280);
    ASSERT_EQ(rgbd.depth.rows, 720);
    ASSERT_EQ(rgbd.depth.cols, 1280);

    ASSERT_TRUE(manager_->startFrameStream(0, true, false, cv::Size(640, 480), 30, cv::Size(640, 480), 30));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    mat = manager_->getFrame(0);

    ASSERT_EQ(mat.rows, 480);
    ASSERT_EQ(mat.cols, 640);

    rgbd = manager_->getRGBDFrame(0);
    ASSERT_EQ(rgbd.image.rows, 480);
    ASSERT_EQ(rgbd.image.cols, 640);
    ASSERT_EQ(rgbd.depth.rows, 480);
    ASSERT_EQ(rgbd.depth.cols, 640);
}

TEST_F(RGBDCameraManagerShould, requestTwoFramesAtTheSameTime) {
    manager_.reset(new RGBDCameraManager(mock_));
    simulator_->initializeDelay = std::chrono::seconds(1);
    auto firstFrame = std::async(std::launch::async, [this]() {
        manager_->startFrameStream(0, true, false, cv::Size(1280, 720), 30, cv::Size(640, 480), 30);
        return manager_->getFrame(0);
    });

    auto secondFrame = std::async(std::launch::async, [this]() {
        manager_->startFrameStream(1, true, false, cv::Size(1920, 1080), 30, cv::Size(1920, 1080), 30);
        return manager_->getRGBDFrame(1);
    });

    auto mat1 = firstFrame.get();
    ASSERT_EQ(mat1.rows, 720);
    ASSERT_EQ(mat1.cols, 1280);

    auto mat2 = secondFrame.get().depth;

    ASSERT_EQ(mat2.rows, 1080);
    ASSERT_EQ(mat2.cols, 1920);
}

TEST_F(RGBDCameraManagerShould, requestTwoFramesAtTheSameTimeFailsIfCantInitialize) {
    manager_.reset(new RGBDCameraManager(mock_));
    simulator_->failsToInitialize = true;

    auto firstFrame = std::async(std::launch::async, [this]() {       
        EXPECT_THROW({
            try{
                 return manager_->startFrameStream(0, true, false, cv::Size(1920, 1080), 30, cv::Size(640, 480), 30);
            }
            catch(const std::runtime_error& err) {
                EXPECT_STREQ(err.what(), "Cannot initialize the camera" );
                throw;
            }
        }, std::runtime_error);
        return false;
    });

    auto secondFrame = std::async(std::launch::async, [this]() {
        EXPECT_THROW({
            try{
                 return manager_->startFrameStream(1, true, false, cv::Size(1920, 1080), 30, cv::Size(640, 480), 30);
            }
            catch(const std::runtime_error& err) {
                EXPECT_STREQ(err.what(), "Cannot initialize the camera" );
                throw;
            }
        }, std::runtime_error);
        return false;
    });

    ASSERT_FALSE(firstFrame.get());
    ASSERT_FALSE(secondFrame.get());

    simulator_->failsToInitialize = false;
}

TEST_F(RGBDCameraManagerShould, returnsEmptyMatAfterLastFrameAndTimeout) {
    manager_.reset(new RGBDCameraManager(mock_, std::chrono::milliseconds(100)));
    simulator_->initializeDelay = std::chrono::milliseconds(1);

    manager_->startFrameStream(0, true, false, cv::Size(1280, 720), 30, cv::Size(640, 480), 30);
    auto mat = manager_->getRGBDFrame(0, std::chrono::seconds(60));
    ASSERT_EQ(mat.image.rows, 720);
    ASSERT_EQ(mat.image.cols, 1280);

    manager_->stopFrameStream(0);
    mat = manager_->getRGBDFrame(0, std::chrono::seconds(60));
    ASSERT_TRUE(mat.image.empty());

    manager_->startFrameStream(0, true, false, cv::Size(1280, 720), 30, cv::Size(640, 480), 30);
    mat = manager_->getRGBDFrame(0, std::chrono::seconds(60));
    ASSERT_EQ(mat.image.rows, 720);
    ASSERT_EQ(mat.image.cols, 1280);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    mat = manager_->getRGBDFrame(0);
    ASSERT_TRUE(mat.image.empty());
}

TEST_F(RGBDCameraManagerShould, correctlyGetSetParameters) {
    simulator_.reset(new IRGBDCameraSimulator(mock_));
    manager_.reset(new RGBDCameraManager(mock_));
    ASSERT_TRUE(manager_->startFrameStream(0, true, false, cv::Size(1280, 720), 30, cv::Size(640, 480), 30));
    auto parameters = manager_->getStatus();
    ASSERT_TRUE(manager_->setStatus(parameters));
    auto avRes = parameters.at("resolutions").get<std::vector<std::pair<int, int> > >().size();
    ASSERT_EQ(avRes, 6);

    auto avRGBDRes = parameters.at("depth_resolutions")
        .get<std::vector<std::pair<int, int> > >().size();
    ASSERT_EQ(avRGBDRes, 6);

    auto extrinsics = parameters.at("extrinsic").get<cv::Mat>();
    ASSERT_EQ(extrinsics.rows, 4);
    ASSERT_EQ(extrinsics.cols, 4);
    auto depthIntrinsics = parameters.at("depth_intrinsic").get<cv::Mat>();
    ASSERT_EQ(depthIntrinsics.rows, 3);
    ASSERT_EQ(depthIntrinsics.cols, 3);
    auto depthDistortion = parameters.at("depth_distortion").get<cv::Mat>();
    ASSERT_EQ(depthDistortion.rows, 1);
    ASSERT_EQ(depthDistortion.cols, 5);
    auto colorIntrinsics = parameters.at("color_intrinsic").get<cv::Mat>();
    ASSERT_EQ(colorIntrinsics.rows, 3);
    ASSERT_EQ(colorIntrinsics.cols, 3);
    auto colorDistortion = parameters.at("color_distortion").get<cv::Mat>();
    ASSERT_EQ(colorDistortion.rows, 1);
    ASSERT_EQ(colorDistortion.cols, 5);
}

TEST_F(RGBDCameraManagerShould, someParametersFailure) {
    manager_.reset(new RGBDCameraManager(mock_));
    auto parameters = manager_->getStatus();
    parameters["zoom"] = "tmp";
    ASSERT_FALSE(manager_->setStatus(parameters));

    parameters = manager_->getStatus();
    parameters["position"].clear();
    ASSERT_FALSE(manager_->setStatus(parameters));

    parameters = manager_->getStatus();
    parameters["exposure"] = "tmp";
    ASSERT_FALSE(manager_->setStatus(parameters));

    parameters = manager_->getStatus();
    parameters["shutter"] = "tmp";
    ASSERT_FALSE(manager_->setStatus(parameters));

    parameters = manager_->getStatus();
    parameters["focus"] = "tmp";
    ASSERT_FALSE(manager_->setStatus(parameters));

    parameters = manager_->getStatus();
    parameters["iso"] = "tmp";
    ASSERT_FALSE(manager_->setStatus(parameters));

    parameters = manager_->getStatus();
    ASSERT_TRUE(manager_->setStatus(parameters));
}

TEST_F(RGBDCameraManagerShould, failsToSetParametersIfCameraDoesNotSupportIt) {
    manager_.reset(new RGBDCameraManager(mock_));
    auto parameters = manager_->getStatus();
    ASSERT_TRUE(manager_->setStatus(parameters));

    simulator_->canZoom = false;
    ASSERT_FALSE(manager_->setStatus(parameters));

    simulator_->canZoom = true;
    simulator_->canMove = false;
    ASSERT_FALSE(manager_->setStatus(parameters));

    simulator_->canMove = true;
    simulator_->canSetExposure = false;
    ASSERT_FALSE(manager_->setStatus(parameters));

    simulator_->canSetExposure = true;
    simulator_->canSetShutterSpeed = false;
    ASSERT_FALSE(manager_->setStatus(parameters));

    simulator_->canSetShutterSpeed = true;
    simulator_->canSetFocus = false;
    ASSERT_FALSE(manager_->setStatus(parameters));

    simulator_->canSetFocus = true;
    simulator_->canFocus = false;
    ASSERT_FALSE(manager_->setStatus(parameters));

    simulator_->canFocus = true;
    simulator_->canSetIso = false;
    ASSERT_FALSE(manager_->setStatus(parameters));

    simulator_->canSetIso = true;
    ASSERT_TRUE(manager_->setStatus(parameters));
}

TEST_F(RGBDCameraManagerShould, parametersMissingFieldIfOperationNotPermitted) {
    manager_.reset(new RGBDCameraManager(mock_));
    auto parameters = manager_->getStatus();
    ASSERT_NE(parameters.find("zoom"), parameters.end());
    simulator_->canZoom = false;
    parameters = manager_->getStatus();
    ASSERT_EQ(parameters.find("zoom"), parameters.end());

    ASSERT_NE(parameters.find("position"), parameters.end());
    simulator_->canMove = false;
    parameters = manager_->getStatus();
    ASSERT_EQ(parameters.find("position"), parameters.end());

    ASSERT_NE(parameters.find("exposure"), parameters.end());
    simulator_->canSetExposure = false;
    parameters = manager_->getStatus();
    ASSERT_EQ(parameters.find("exposure"), parameters.end());

    ASSERT_NE(parameters.find("shutter"), parameters.end());
    simulator_->canSetShutterSpeed = false;
    parameters = manager_->getStatus();
    ASSERT_EQ(parameters.find("shutter"), parameters.end());

    ASSERT_NE(parameters.find("focusmode"), parameters.end());
    simulator_->canSetFocus = false;
    parameters = manager_->getStatus();
    ASSERT_EQ(parameters.find("focusmode"), parameters.end());

    ASSERT_NE(parameters.find("focus"), parameters.end());
    simulator_->canFocus = false;
    parameters = manager_->getStatus();
    ASSERT_EQ(parameters.find("focus"), parameters.end());

    ASSERT_NE(parameters.find("iso"), parameters.end());
    simulator_->canSetIso = false;
    parameters = manager_->getStatus();
    ASSERT_EQ(parameters.find("iso"), parameters.end());
}
