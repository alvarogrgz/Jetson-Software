/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <thread>
#include <chrono>

#include <nlohmann/json.hpp>

#include "DeviceManager/DeviceManagerWithAutoInitialization/DeviceManagerWithAutoInitialization.hpp"
#include "CommonInterfaces/IInitializable.hpp"
#include "EventLogger/EventLogger.hpp"

#include "CommonInterfaces/InitializableMock.hpp"

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

class DeviceManagerWithAutoInitializationShould : public ::testing::Test {
 protected:
    DeviceManagerWithAutoInitializationShould() :
        logger_("DeviceManagerWithAutoInitializationShould"),
        deviceMock_(new NiceMock<crf::utility::commoninterfaces::InitializableMock>) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        initializeResult_ = true;
        deinitializeResult_ = true;
        initializeStatusJSON_["status"] = "initialize";
        deinitializeStatusJSON_["status"] = "deinitialize";
        configureDevice();
    }

    ~DeviceManagerWithAutoInitializationShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void configureDevice() {
        ON_CALL(*deviceMock_, initialize()).WillByDefault(Invoke([this](){
            return initializeResult_;
        }));
        ON_CALL(*deviceMock_, deinitialize()).WillByDefault(Invoke([this](){
            return deinitializeResult_;
        }));
    }

    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<crf::utility::devicemanager::DeviceManagerWithAutoInitialization> sut_;
    std::shared_ptr<NiceMock<crf::utility::commoninterfaces::InitializableMock>> deviceMock_;
    bool initializeResult_;
    bool deinitializeResult_;
    nlohmann::json initializeStatusJSON_;
    nlohmann::json deinitializeStatusJSON_;
};

TEST_F(DeviceManagerWithAutoInitializationShould, getStatusAccordingToInitialization) {
    std::chrono::milliseconds initializationTimeout(50);
    sut_.reset(new crf::utility::devicemanager::DeviceManagerWithAutoInitialization(
        deviceMock_, initializationTimeout));

    initializeResult_ = true;
    ASSERT_EQ(sut_->getStatus(), initializeStatusJSON_);
    EXPECT_CALL(*deviceMock_, initialize()).Times(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_CALL(*deviceMock_, deinitialize()).Times(1);
    initializeResult_ = false;
    ASSERT_EQ(sut_->getStatus(), deinitializeStatusJSON_);
}

TEST_F(DeviceManagerWithAutoInitializationShould, correctlyDeinitilizeAfterTimeout) {
    std::chrono::milliseconds initializationTimeout(100);
    sut_.reset(new crf::utility::devicemanager::DeviceManagerWithAutoInitialization(
        deviceMock_, initializationTimeout));

    initializeResult_ = true;
    ASSERT_EQ(sut_->getStatus(), initializeStatusJSON_);
    EXPECT_CALL(*deviceMock_, initialize()).Times(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(33));
    ASSERT_EQ(sut_->getStatus(), initializeStatusJSON_);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    EXPECT_CALL(*deviceMock_, deinitialize()).Times(1);
    initializeResult_ = false;
    ASSERT_EQ(sut_->getStatus(), deinitializeStatusJSON_);
}

TEST_F(DeviceManagerWithAutoInitializationShould, failIfTimeoutIs0) {
    std::chrono::milliseconds initializationTimeout(0);
    ASSERT_THROW(sut_.reset(new crf::utility::devicemanager::DeviceManagerWithAutoInitialization(
        deviceMock_, initializationTimeout)), std::runtime_error);
}

TEST_F(DeviceManagerWithAutoInitializationShould, failIfEmptyDevice) {
    std::chrono::milliseconds initializationTimeout(10);
    ASSERT_THROW(sut_.reset(new crf::utility::devicemanager::DeviceManagerWithAutoInitialization(
        nullptr, initializationTimeout)), std::runtime_error);
}
