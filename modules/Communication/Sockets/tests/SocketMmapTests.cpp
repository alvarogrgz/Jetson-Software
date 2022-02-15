/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#define MMAP_SIZE 4096

#include <future>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "EventLogger/EventLogger.hpp"
#include "Sockets/IPC/internal/SocketMmap.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::communication::sockets::internal::SocketMmap;

class SocketMmapShould : public ::testing::Test {
 protected:
    SocketMmapShould() :
        logger_("SocketMmapShould") {
            logger_->info("{0} BEGIN",
                testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~SocketMmapShould() {
        logger_->info("{0} END with {1}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;

    std::unique_ptr<SocketMmap> server_;
    std::unique_ptr<SocketMmap> client_;
};

TEST_F(SocketMmapShould, correctlyCloseAndOpenServerSequence) {
    server_.reset(new SocketMmap("/tmp/servermmap", SocketMmap::Roles::Server, MMAP_SIZE));
    ASSERT_TRUE(server_->open());
    ASSERT_FALSE(server_->open());
    ASSERT_TRUE(server_->close());
    ASSERT_FALSE(server_->close());

    ASSERT_TRUE(server_->open());
    ASSERT_FALSE(server_->open());
    ASSERT_TRUE(server_->close());
    ASSERT_FALSE(server_->close());
}

TEST_F(SocketMmapShould, correctlyWriteUntilTheresNoMoreSpace) {
    server_.reset(new SocketMmap("/tmp/servermmap", SocketMmap::Roles::Server, MMAP_SIZE));
    client_.reset(new SocketMmap("/tmp/servermmap", SocketMmap::Roles::Client, MMAP_SIZE));
    std::string buffer;
    buffer.resize(MMAP_SIZE/4);
    ASSERT_FALSE(client_->open());
    ASSERT_TRUE(server_->open());
    ASSERT_TRUE(client_->open());
    std::cout << "Qui" << std::endl;
    ASSERT_TRUE(server_->write(buffer));
    std::cout << "Qui" << std::endl;
    ASSERT_TRUE(server_->write(buffer));
    std::cout << "Qui" << std::endl;
    ASSERT_TRUE(server_->write(buffer));
    ASSERT_TRUE(server_->write(buffer));
    std::cout << "Qui" << std::endl;
    ASSERT_FALSE(server_->write(buffer));
    ASSERT_TRUE(server_->close());
}

TEST_F(SocketMmapShould, readWriteOperationWorkingProperly) {
    server_.reset(new SocketMmap("/tmp/servermmap", SocketMmap::Roles::Server, MMAP_SIZE));
    client_.reset(new SocketMmap("/tmp/servermmap", SocketMmap::Roles::Client, MMAP_SIZE));
    std::string buffer;
    buffer.resize(MMAP_SIZE/4);
    ASSERT_TRUE(server_->open());
    ASSERT_TRUE(client_->open());

    for (int i=0; i < 100; i++) {
        ASSERT_TRUE(server_->write(buffer));
        ASSERT_EQ(client_->read(MMAP_SIZE/4).length(), MMAP_SIZE/4);
        ASSERT_TRUE(client_->write(buffer));
        ASSERT_EQ(server_->read(MMAP_SIZE/8).length(), MMAP_SIZE/8);
        ASSERT_EQ(server_->read(MMAP_SIZE/8).length(), MMAP_SIZE/8);
    }

    ASSERT_TRUE(client_->close());
    ASSERT_TRUE(server_->close());
}

TEST_F(SocketMmapShould, readWriteWithBlockingRead) {
    server_.reset(new SocketMmap("/tmp/servermmap", SocketMmap::Roles::Server, MMAP_SIZE));
    client_.reset(new SocketMmap("/tmp/servermmap", SocketMmap::Roles::Client, MMAP_SIZE));
    std::string buffer("Ciao");
    ASSERT_TRUE(server_->open());
    ASSERT_TRUE(client_->open());

    auto readCompleted = std::async(std::launch::async, [this]() {
        return client_->read(4);
    });

    ASSERT_TRUE(server_->write(buffer));
    ASSERT_EQ(readCompleted.get(), "Ciao");
    ASSERT_TRUE(client_->close());
    ASSERT_TRUE(server_->close());
}

TEST_F(SocketMmapShould, readWriteWithTimeoutRead) {
    server_.reset(new SocketMmap("/tmp/servermmap", SocketMmap::Roles::Server, MMAP_SIZE));
    client_.reset(new SocketMmap("/tmp/servermmap", SocketMmap::Roles::Client, MMAP_SIZE));
    std::string buffer("Ciao2");
    ASSERT_TRUE(server_->open());
    ASSERT_TRUE(client_->open());

    ASSERT_EQ(client_->read(5, std::chrono::milliseconds(10)).length(), 0);
    ASSERT_EQ(client_->read(5, std::chrono::milliseconds(0)).length(), 0);

    auto readCompleted = std::async(std::launch::async, [this]() {
        return client_->read(5, std::chrono::milliseconds(1000));
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ASSERT_TRUE(server_->write(buffer));
    ASSERT_EQ(readCompleted.get(), "Ciao2");
    ASSERT_TRUE(client_->close());
    ASSERT_TRUE(server_->close());
}

TEST_F(SocketMmapShould, readFromServer) {
    server_.reset(new SocketMmap("/tmp/servermmap", SocketMmap::Roles::Server, MMAP_SIZE));
    client_.reset(new SocketMmap("/tmp/servermmap", SocketMmap::Roles::Client, MMAP_SIZE));
    std::string buffer("Ciao3");
    ASSERT_TRUE(server_->open());
    ASSERT_TRUE(client_->open());

    ASSERT_EQ(server_->read(5, std::chrono::milliseconds(10)).length(), 0);
    ASSERT_EQ(server_->read(5, std::chrono::milliseconds(0)).length(), 0);

    auto readCompleted = std::async(std::launch::async, [this]() {
        return server_->read(5, std::chrono::milliseconds(1000));
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ASSERT_TRUE(client_->write(buffer));
    ASSERT_EQ(readCompleted.get(), "Ciao3");
    readCompleted = std::async(std::launch::async, [this]() {
        return server_->read(5, std::chrono::milliseconds(1000));
    });

    ASSERT_TRUE(client_->write(buffer));
    ASSERT_EQ(readCompleted.get(), "Ciao3");
    ASSERT_TRUE(client_->close());
    ASSERT_TRUE(server_->close());
}


TEST_F(SocketMmapShould, cantWriteOrReadIfPartnerNotAlive) {
    server_.reset(new SocketMmap("/tmp/servermmap", SocketMmap::Roles::Server, MMAP_SIZE));
    client_.reset(new SocketMmap("/tmp/servermmap", SocketMmap::Roles::Client, MMAP_SIZE));
    std::string buffer;
    buffer.resize(MMAP_SIZE/4);
    ASSERT_TRUE(server_->open());
    ASSERT_TRUE(client_->open());

    ASSERT_TRUE(server_->write(buffer));
    ASSERT_TRUE(server_->close());

    ASSERT_FALSE(client_->write(buffer));
    ASSERT_EQ(client_->read(MMAP_SIZE/4).length(), 0);

    ASSERT_TRUE(client_->close());
}

TEST_F(SocketMmapShould, clientCloseAndOpenSequence) {
    server_.reset(new SocketMmap("/tmp/servermmap", SocketMmap::Roles::Server, MMAP_SIZE));
    client_.reset(new SocketMmap("/tmp/servermmap", SocketMmap::Roles::Client, MMAP_SIZE));
    std::string buffer;
    buffer.resize(MMAP_SIZE/4);
    ASSERT_TRUE(server_->open());
    ASSERT_TRUE(client_->open());
    ASSERT_FALSE(client_->open());
    ASSERT_TRUE(client_->close());
    ASSERT_FALSE(client_->close());

    ASSERT_TRUE(client_->open());
    ASSERT_FALSE(client_->open());
    ASSERT_TRUE(client_->close());
    ASSERT_FALSE(client_->close());

    ASSERT_TRUE(server_->close());
}

TEST_F(SocketMmapShould, operationsWorkIfCloseOnAnotherThread) {
    server_.reset(new SocketMmap("/tmp/servermmap", SocketMmap::Roles::Server, MMAP_SIZE));
    std::string buffer;
    buffer.resize(MMAP_SIZE/4);
    ASSERT_TRUE(server_->open());
    auto readCompleted = std::async(std::launch::async, [this]() {
        return server_->read(MMAP_SIZE/4);
    });

    ASSERT_TRUE(server_->close());
    ASSERT_EQ(readCompleted.get().length(), 0);
}
