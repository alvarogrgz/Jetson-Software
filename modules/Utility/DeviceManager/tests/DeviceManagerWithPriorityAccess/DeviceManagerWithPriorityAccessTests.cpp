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

#include "DeviceManager/DeviceManagerWithPriorityAccess/DeviceManagerWithPriorityAccess.hpp"
#include "CommonInterfaces/IInitializable.hpp"
#include "EventLogger/EventLogger.hpp"

#include "CommonInterfaces/InitializableMock.hpp"

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

class DeviceManagerWithPriorityAccessShould : public ::testing::Test {
 protected:
    DeviceManagerWithPriorityAccessShould() :
        logger_("DeviceManagerWithPriorityAccessShould"),
        deviceMock_(new NiceMock<crf::utility::commoninterfaces::InitializableMock>) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        initializeResult_ = true;
        deinitializeResult_ = true;
        statusJSON_["status"] = "initialize";
        statusJSON_["priorityUnderControl"] = 0;
        configureDevice();
    }

    ~DeviceManagerWithPriorityAccessShould() {
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
    std::unique_ptr<crf::utility::devicemanager::DeviceManagerWithPriorityAccess> sut_;
    std::shared_ptr<NiceMock<crf::utility::commoninterfaces::InitializableMock>> deviceMock_;
    bool initializeResult_;
    bool deinitializeResult_;
    nlohmann::json statusJSON_;
};

TEST_F(DeviceManagerWithPriorityAccessShould, getStatusAccordingToLockControl) {
    std::chrono::milliseconds initializationTimeout(50);
    std::chrono::milliseconds controlAccessTimeout(10);
    sut_.reset(new crf::utility::devicemanager::DeviceManagerWithPriorityAccess(
        deviceMock_, initializationTimeout, controlAccessTimeout));

    initializeResult_ = true;
    ASSERT_EQ(sut_->getStatus(), statusJSON_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    statusJSON_["status"] = "deinitialize";
    initializeResult_ = false;
    ASSERT_EQ(sut_->getStatus(), statusJSON_);

    initializeResult_ = true;
    statusJSON_["status"] = "initialize";
    statusJSON_["priorityUnderControl"] = 1;
    ASSERT_TRUE(sut_->lockControl(1));
    ASSERT_EQ(sut_->getStatus(), statusJSON_);

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    statusJSON_["priorityUnderControl"] = 0;
    ASSERT_EQ(sut_->getStatus(), statusJSON_);
}

TEST_F(DeviceManagerWithPriorityAccessShould, properlyRevokeControl) {
    std::chrono::milliseconds initializationTimeout(50);
    std::chrono::milliseconds controlAccessTimeout(100);
    sut_.reset(new crf::utility::devicemanager::DeviceManagerWithPriorityAccess(
        deviceMock_, initializationTimeout, controlAccessTimeout));

    initializeResult_ = true;
    statusJSON_["status"] = "initialize";
    statusJSON_["priorityUnderControl"] = 0;
    ASSERT_EQ(sut_->getStatus(), statusJSON_);

    statusJSON_["priorityUnderControl"] = 1;
    ASSERT_TRUE(sut_->lockControl(1));
    ASSERT_EQ(sut_->getStatus(), statusJSON_);

    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    statusJSON_["priorityUnderControl"] = 0;
    ASSERT_EQ(sut_->getStatus(), statusJSON_);

    statusJSON_["priorityUnderControl"] = 100;
    ASSERT_TRUE(sut_->lockControl(100));
    ASSERT_EQ(sut_->getStatus(), statusJSON_);

    statusJSON_["priorityUnderControl"] = 100;
    ASSERT_FALSE(sut_->lockControl(200));
    ASSERT_EQ(sut_->getStatus(), statusJSON_);

    statusJSON_["priorityUnderControl"] = 1;
    ASSERT_TRUE(sut_->lockControl(1));
    ASSERT_EQ(sut_->getStatus(), statusJSON_);
}

TEST_F(DeviceManagerWithPriorityAccessShould, properlyLockAndUnlockControl) {
    std::chrono::milliseconds initializationTimeout(50);
    std::chrono::milliseconds controlAccessTimeout(10);
    sut_.reset(new crf::utility::devicemanager::DeviceManagerWithPriorityAccess(
        deviceMock_, initializationTimeout, controlAccessTimeout));

    initializeResult_ = true;
    statusJSON_["status"] = "initialize";
    statusJSON_["priorityUnderControl"] = 0;
    ASSERT_EQ(sut_->getStatus(), statusJSON_);

    statusJSON_["priorityUnderControl"] = 1;
    ASSERT_TRUE(sut_->lockControl(1));
    ASSERT_EQ(sut_->getStatus(), statusJSON_);

    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    statusJSON_["priorityUnderControl"] = 0;
    ASSERT_FALSE(sut_->unlockControl(1));
    ASSERT_EQ(sut_->getStatus(), statusJSON_);

    statusJSON_["priorityUnderControl"] = 100;
    ASSERT_TRUE(sut_->lockControl(100));
    ASSERT_EQ(sut_->getStatus(), statusJSON_);

    statusJSON_["priorityUnderControl"] = 0;
    ASSERT_TRUE(sut_->unlockControl(100));
    ASSERT_EQ(sut_->getStatus(), statusJSON_);
}

TEST_F(DeviceManagerWithPriorityAccessShould, failIfTimeoutIs0) {
    std::chrono::milliseconds unvalidTimeout(0);
    std::chrono::milliseconds validTimeout(10);
    ASSERT_THROW(sut_.reset(new crf::utility::devicemanager::DeviceManagerWithPriorityAccess(
        deviceMock_, unvalidTimeout, validTimeout)), std::runtime_error);
    ASSERT_THROW(sut_.reset(new crf::utility::devicemanager::DeviceManagerWithPriorityAccess(
        deviceMock_, validTimeout, unvalidTimeout)), std::runtime_error);
}

TEST_F(DeviceManagerWithPriorityAccessShould, failIfEmptyDevice) {
    std::chrono::milliseconds validTimeout(10);
    ASSERT_THROW(sut_.reset(new crf::utility::devicemanager::DeviceManagerWithPriorityAccess(
        nullptr, validTimeout, validTimeout)), std::runtime_error);
}
