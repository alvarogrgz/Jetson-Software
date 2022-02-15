/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#include <condition_variable>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <mutex>
#include <string>

#include "CommunicationPointServer/CommunicationPointServer.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Sockets/SocketServerMock.hpp"
#include "Sockets/SocketMock.hpp"
#include "CommunicationPointServer/CommunicationPointFactoryMock.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::communication::communicationpointserver::CommunicationPointServer;
using crf::communication::sockets::SocketServerMock;
using crf::communication::sockets::SocketMock;
using crf::communication::communicationpointserver::CommunicationPointFactoryMock;

class CommunicationPointServerShould : public ::testing::Test {
 protected:
    CommunicationPointServerShould() :
        logger_("CommunicationPointServerShould"),
        sut_(),
        server_(new NiceMock<SocketServerMock>),
        socket_(new NiceMock<SocketMock>),
        factory_(new NiceMock<CommunicationPointFactoryMock>) {
            logger_->info("{0} BEGIN",
                testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    void SetUp() override {
        ON_CALL(*server_, open()).WillByDefault(Return(true));
        ON_CALL(*server_, close()).WillByDefault(Return(true));

        ON_CALL(*server_, acceptConnection()).WillByDefault(Invoke([this]() {
            return boost::none;
        }));

        ON_CALL(*factory_, create(_)).WillByDefault(Return(boost::none));
    }

    ~CommunicationPointServerShould() {
        logger_->info("{0} END with {1}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;

    std::unique_ptr<CommunicationPointServer> sut_;
    std::shared_ptr<SocketServerMock> server_;
    std::shared_ptr<SocketMock> socket_;
    std::shared_ptr<CommunicationPointFactoryMock> factory_;
};

TEST_F(CommunicationPointServerShould, initializeDeinitializeSequence) {
    sut_.reset(new CommunicationPointServer(server_, factory_));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(CommunicationPointServerShould, correctlyAcceptCall) {
    std::mutex mtx;
    std::condition_variable cv;
    bool called = false;
    ON_CALL(*server_, acceptConnection()).WillByDefault(
        Invoke([this, &called, &mtx, &cv]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            std::unique_lock<std::mutex> lock(mtx);
            called = true;
            cv.notify_all();
            return boost::none;
    }));

    sut_.reset(new CommunicationPointServer(server_, factory_));

    ASSERT_TRUE(sut_->initialize());
    {
        std::unique_lock<std::mutex> lock(mtx);
        ASSERT_TRUE(cv.wait_for(lock, std::chrono::milliseconds(100),
            [&called]() { return called; }));
    }
    ASSERT_TRUE(sut_->deinitialize());
}
