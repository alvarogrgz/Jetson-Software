/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *         Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <memory>
#include <thread>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "EventLogger/EventLogger.hpp"
#include "ComponentAccessControl/SimpleAccessControl/SimpleAccessControl.hpp"

using testing::_;
using testing::Return;

using crf::communication::componentaccesscontrol::SimpleAccessControl;

class SimpleAccessControlShould : public ::testing::Test {
 protected:
    SimpleAccessControlShould() :
        logger_("SimpleAccessControlShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~SimpleAccessControlShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<crf::communication::componentaccesscontrol::SimpleAccessControl> sut_;
};

TEST_F(SimpleAccessControlShould, rejectPriorityEqualToZero) {
    sut_.reset(new crf::communication::componentaccesscontrol::SimpleAccessControl());

    ASSERT_FALSE(sut_->requestAccess(0));
    ASSERT_FALSE(sut_->releaseAccess(0));
}

TEST_F(SimpleAccessControlShould, correctlyAddAndRemoveSinglePriority) {
    sut_.reset(new crf::communication::componentaccesscontrol::SimpleAccessControl());

    ASSERT_TRUE(sut_->requestAccess(1));
    ASSERT_TRUE(sut_->requestAccess(1));
    ASSERT_TRUE(sut_->releaseAccess(1));
    ASSERT_FALSE(sut_->releaseAccess(1));

    ASSERT_TRUE(sut_->requestAccess(1));
    ASSERT_TRUE(sut_->requestAccess(1));
    ASSERT_TRUE(sut_->releaseAccess(1));
    ASSERT_FALSE(sut_->releaseAccess(1));
}

TEST_F(SimpleAccessControlShould, correctlyAddAndRemoveMultiplePriorities) {
    sut_.reset(new crf::communication::componentaccesscontrol::SimpleAccessControl());

    ASSERT_TRUE(sut_->requestAccess(2));
    ASSERT_TRUE(sut_->requestAccess(1));
    ASSERT_FALSE(sut_->requestAccess(2));
    ASSERT_TRUE(sut_->requestAccess(1));
    ASSERT_TRUE(sut_->releaseAccess(1));
    ASSERT_TRUE(sut_->requestAccess(2));
    ASSERT_TRUE(sut_->releaseAccess(2));

    ASSERT_TRUE(sut_->requestAccess(2));
    ASSERT_TRUE(sut_->requestAccess(1));
    ASSERT_FALSE(sut_->requestAccess(2));
    ASSERT_TRUE(sut_->requestAccess(1));
    ASSERT_TRUE(sut_->releaseAccess(1));
    ASSERT_TRUE(sut_->requestAccess(2));
    ASSERT_TRUE(sut_->releaseAccess(2));
}

TEST_F(SimpleAccessControlShould, correctlyWorkExpiredAccess) {
    sut_.reset(new crf::communication::componentaccesscontrol::SimpleAccessControl(
        std::chrono::milliseconds(100)));

    ASSERT_TRUE(sut_->requestAccess(2));
    ASSERT_TRUE(sut_->requestAccess(1));
    ASSERT_FALSE(sut_->requestAccess(2));

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    ASSERT_TRUE(sut_->requestAccess(2));
    ASSERT_FALSE(sut_->releaseAccess(1));
    ASSERT_TRUE(sut_->releaseAccess(2));
}
