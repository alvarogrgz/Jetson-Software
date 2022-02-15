/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
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

#include "Cameras/CameraCommunicationPoint/CameraManager.hpp"
#include "../ICameraSimulator.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Cameras/CameraMock.hpp"


using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::sensors::cameras::CameraMock;
using crf::sensors::cameras::CameraManager;

class CameraManagerShould : public ::testing::Test {
 protected:
    CameraManagerShould() :
        logger_("CameraManagerShould"),
        mock_(new NiceMock<CameraMock>),
        simulator_(new ICameraSimulator(mock_)) {
            logger_->info("{0} BEGIN",
                testing::UnitTest::GetInstance()->current_test_info()->name());
            simulator_->initializeDelay = std::chrono::milliseconds(1);
    }

    ~CameraManagerShould() {
        logger_->info("{0} END with {1}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;

    std::shared_ptr<CameraMock> mock_;
    std::unique_ptr<ICameraSimulator> simulator_;
    std::unique_ptr<CameraManager> manager_;
};

TEST_F(CameraManagerShould, getFrameFailsIfNotRequestedBefore) {
    manager_.reset(new CameraManager(mock_));
    EXPECT_THROW({
        try{
            auto mat = manager_->getFrame(0);
        }
        catch(const std::runtime_error& err) {
            EXPECT_STREQ(err.what(), "Frame grabber is not running" );
            throw;
        }
    }, std::runtime_error);
};

TEST_F(CameraManagerShould, getFrameFailsIfRequestedSocketIsNotActive) {
    manager_.reset(new CameraManager(mock_));

    ASSERT_TRUE(manager_->startFrameStream(0, cv::Size(1280, 720), 30));
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


TEST_F(CameraManagerShould, cantRequestResolutionWithOneDimensionEqualTo0) {
    manager_.reset(new CameraManager(mock_));

    EXPECT_THROW({
        try {
            manager_->startFrameStream(0, cv::Size(0, 720), 30);
        } 
        catch( const std::invalid_argument& e ) {
            EXPECT_STREQ( "Can't request a resolution with a dimension equal to 0", e.what() );
            throw;
        }
        }, std::invalid_argument );
            EXPECT_THROW({
        try {
            manager_->startFrameStream(0, cv::Size(1280, 0), 30);
        } 
        catch( const std::invalid_argument& e ) {
            EXPECT_STREQ( "Can't request a resolution with a dimension equal to 0", e.what() );
            throw;
        }
        }, std::invalid_argument );      
}


TEST_F(CameraManagerShould, correctlyGetFrameAndUpdateResolution) {
    manager_.reset(new CameraManager(mock_));

    ASSERT_TRUE(manager_->startFrameStream(0, cv::Size(1280, 720), 30));
    auto mat = manager_->getFrame(0);

    ASSERT_EQ(mat.rows, 720);
    ASSERT_EQ(mat.cols, 1280);

    ASSERT_TRUE(manager_->startFrameStream(0, cv::Size(640, 480), 30));
    mat = manager_->getFrame(0);

    ASSERT_EQ(mat.rows, 480);
    ASSERT_EQ(mat.cols, 640);
}

TEST_F(CameraManagerShould, requestTwoFramesAtTheSameTime) {
    manager_.reset(new CameraManager(mock_));
    simulator_->initializeDelay = std::chrono::seconds(1);
    auto firstFrame = std::async(std::launch::async, [this]() {
        manager_->startFrameStream(0, cv::Size(1280, 720), 30);
        return manager_->getFrame(0);
    });

    auto secondFrame = std::async(std::launch::async, [this]() {
        manager_->startFrameStream(1, cv::Size(1920, 1080), 30);
        return manager_->getFrame(1);
    });
    auto mat1 = firstFrame.get();
    ASSERT_EQ(mat1.rows, 720);
    ASSERT_EQ(mat1.cols, 1280);

    auto mat2 = secondFrame.get();

    ASSERT_EQ(mat2.rows, 1080);
    ASSERT_EQ(mat2.cols, 1920);
}

TEST_F(CameraManagerShould, requestTwoFramesAtTheSameTimeFailsIfCantInitialize) {
    manager_.reset(new CameraManager(mock_));
    simulator_->failsToInitialize = true;


    EXPECT_THROW({
        try {
            manager_->startFrameStream(0, cv::Size(1280, 720), 30);
        } 
        catch( const std::runtime_error& e ) {
            EXPECT_STREQ("Cannot initialize the camera", e.what() );
            throw;
        }
    }, std::runtime_error );

    EXPECT_THROW({
        try {
            manager_->startFrameStream(1, cv::Size(1920, 1080), 30);
        } 
        catch( const std::runtime_error& e ) {
            EXPECT_STREQ("Cannot initialize the camera", e.what() );
            throw;
        }
    }, std::runtime_error );

    simulator_->failsToInitialize = false;
}

TEST_F(CameraManagerShould, returnsEmptyMatAfterTimeout) {
    manager_.reset(new CameraManager(mock_, std::chrono::milliseconds(100)));
    simulator_->initializeDelay = std::chrono::milliseconds(1);

    manager_->startFrameStream(0, cv::Size(1280, 720), 30);
    
    auto mat = manager_->getFrame(0, std::chrono::seconds(60));
    ASSERT_FALSE(mat.empty());

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    EXPECT_THROW({
        try{
            auto mat = manager_->getFrame(0);
        }
        catch(const std::runtime_error& err) {
            EXPECT_STREQ(err.what(), "Frame grabber is not running" );
            throw;
        }
    }, std::runtime_error);
};

TEST_F(CameraManagerShould, correctlyGetSetParameters) {
    simulator_.reset(new ICameraSimulator(mock_));
    manager_.reset(new CameraManager(mock_));
    ASSERT_TRUE(manager_->startFrameStream(0, cv::Size(1280, 720), 30));
    auto parameters = manager_->getStatus();
    ASSERT_TRUE(manager_->setStatus(parameters));

    std::vector<cv::Size> resolutions;
    auto profiles = parameters["profiles"];
    for (nlohmann::json::iterator it = profiles.begin(); it != profiles.end(); ++it) {
        std::pair <int, int> res = it.value()["resolution"].get<std::pair <int, int>>();
        resolutions.push_back(cv::Size(res.first, res.second));
    }
    ASSERT_EQ(resolutions.size(), 6);
}

TEST_F(CameraManagerShould, someParametersFailure) {
    manager_.reset(new CameraManager(mock_));
    ASSERT_TRUE(manager_->startFrameStream(0, cv::Size(1280, 720), 30));
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

TEST_F(CameraManagerShould, failsToSetParametersIfCameraDoesNotSupportIt) {
    manager_.reset(new CameraManager(mock_));
    ASSERT_TRUE(manager_->startFrameStream(0, cv::Size(1280, 720), 30));
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

TEST_F(CameraManagerShould, parametersMissingFieldIfOperationNotPermitted) {
    manager_.reset(new CameraManager(mock_));
    ASSERT_TRUE(manager_->startFrameStream(0, cv::Size(1280, 720), 30));

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
