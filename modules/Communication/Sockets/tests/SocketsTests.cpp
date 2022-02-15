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

#include "EventLogger/EventLogger.hpp"
#include "Sockets/ISocketServer.hpp"
#include "Sockets/ISocket.hpp"

#include "Sockets/TCP/TCPServer.hpp"
#include "Sockets/TCP/TCPSocket.hpp"

#include "Sockets/IPC/UnixSocket.hpp"
#include "Sockets/IPC/UnixSocketServer.hpp"

// #include "Sockets/IPC/IPCSocket.hpp"
// #include "Sockets/IPC/IPCServer.hpp"

#define MB_100 104857600

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::communication::sockets::ISocketServer;
using crf::communication::sockets::ISocket;

struct SocketsStruct {
    std::shared_ptr<ISocketServer> server;
    std::shared_ptr<ISocketServer> server2;
    std::shared_ptr<ISocket> client1;
    std::shared_ptr<ISocket> client2;
};

class SocketsShould : public ::testing::TestWithParam<SocketsStruct> {
 protected:
    SocketsShould() :
        logger_("SocketsShould") {
            logger_->info("{0} BEGIN",
                testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    void SetUp() override {
        server_ = GetParam().server;
        server2_ = GetParam().server2;
        client1_ = GetParam().client1;
        client2_ = GetParam().client2;
    }

    void TearDown() override {
        client1_->close();
        client2_->close();
        server_->close();
        server2_->close();
    }

    ~SocketsShould() {
        logger_->info("{0} END with {1}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;

    std::shared_ptr<ISocketServer> server_;
    std::shared_ptr<ISocketServer> server2_;
    std::shared_ptr<ISocket> client1_;
    std::shared_ptr<ISocket> client2_;
};

INSTANTIATE_TEST_CASE_P(TCPSocket, SocketsShould,
    ::testing::Values(
        SocketsStruct{
            std::make_shared<crf::communication::sockets::TCPServer>(65031),
            std::make_shared<crf::communication::sockets::TCPServer>(65031),
            std::make_shared<crf::communication::sockets::TCPSocket>("localhost", 65031),
            std::make_shared<crf::communication::sockets::TCPSocket>("127.0.0.1", 65031),
        }
));

INSTANTIATE_TEST_CASE_P(IPCUnixSocket, SocketsShould,
    ::testing::Values(
        SocketsStruct{
            std::make_shared<crf::communication::sockets::UnixSocketServer>("ipctest"),
            std::make_shared<crf::communication::sockets::UnixSocketServer>("ipctest"),
            std::make_shared<crf::communication::sockets::UnixSocket>("ipctest"),
            std::make_shared<crf::communication::sockets::UnixSocket>("ipctest"),
        }
));

/*INSTANTIATE_TEST_CASE_P(IPCMmapSocket, SocketsShould,
    ::testing::Values(
        SocketsStruct{
            std::make_shared<crf::communication::sockets::IPCServer>("ipctest"),
            std::make_shared<crf::communication::sockets::IPCServer>("ipctest"),
            std::make_shared<crf::communication::sockets::IPCSocket>("ipctest", MB_100),
            std::make_shared<crf::communication::sockets::IPCSocket>("ipctest", MB_100),
        }
));*/

TEST_P(SocketsShould, openCloseServerSequence) {
    ASSERT_FALSE(server_->isOpen());
    ASSERT_TRUE(server_->open());
    ASSERT_TRUE(server_->isOpen());
    ASSERT_FALSE(server_->open());
    ASSERT_TRUE(server_->close());
    ASSERT_FALSE(server_->close());

    ASSERT_TRUE(server_->open());
    ASSERT_FALSE(server_->open());
    ASSERT_TRUE(server_->close());
    ASSERT_FALSE(server_->close());
}

TEST_P(SocketsShould, failsToOpenSecondServerOnSameResource) {
    ASSERT_TRUE(server_->open());
    ASSERT_FALSE(server2_->open());
    ASSERT_TRUE(server_->close());
    ASSERT_TRUE(server2_->open());
    ASSERT_TRUE(server2_->close());
}

TEST_P(SocketsShould, correctlyConnectClient) {
    ASSERT_FALSE(client1_->open());
    ASSERT_TRUE(server_->open());
    auto waitForConnection = std::async(std::launch::async, [this] {
        return server_->acceptConnection();
    });
    ASSERT_FALSE(client1_->isOpen());
    ASSERT_TRUE(client1_->open());
    ASSERT_TRUE(client1_->isOpen());
    auto serverSocket = waitForConnection.get();
    ASSERT_TRUE(serverSocket);
    ASSERT_TRUE(client1_->close());
    ASSERT_TRUE(server_->close());
}

TEST_P(SocketsShould, acceptConnectionReturnsIfServerCloses) {
    ASSERT_TRUE(server_->open());
    auto waitForConnection = std::async(std::launch::async, [this] {
        server_->acceptConnection();
        return true;
    });

    ASSERT_TRUE(server_->close());
    ASSERT_TRUE(waitForConnection.get());
}

TEST_P(SocketsShould, clientConnectDisconnectSequence) {
    ASSERT_FALSE(client1_->open());
    ASSERT_FALSE(client2_->open());
    ASSERT_TRUE(server_->open());

    auto waitForConnection = std::async(std::launch::async, [this] {
        return server_->acceptConnection();
    });
    logger_->info("Client 1 open");
    ASSERT_TRUE(client1_->open());
    ASSERT_TRUE(waitForConnection.get());
    waitForConnection = std::async(std::launch::async, [this] {
        return server_->acceptConnection();
    });
    logger_->info("Client 2 open");
    ASSERT_TRUE(client2_->open());
    ASSERT_TRUE(waitForConnection.get());

    ASSERT_FALSE(client1_->open());
    ASSERT_FALSE(client2_->open());
    logger_->info("Client 1 close");
    ASSERT_TRUE(client1_->close());
    logger_->info("Client 2 close");
    ASSERT_TRUE(client2_->close());
    ASSERT_FALSE(client1_->close());
    ASSERT_FALSE(client2_->close());

    waitForConnection = std::async(std::launch::async, [this] {
        return server_->acceptConnection();
    });
    logger_->info("Client 1 open");
    ASSERT_TRUE(client1_->open());
    ASSERT_TRUE(waitForConnection.get());
    waitForConnection = std::async(std::launch::async, [this] {
        return server_->acceptConnection();
    });
    logger_->info("Client 2 open");
    ASSERT_TRUE(client2_->open());
    ASSERT_TRUE(waitForConnection.get());

    ASSERT_FALSE(client1_->open());
    ASSERT_FALSE(client2_->open());

    logger_->info("Client 1 close");
    ASSERT_TRUE(client1_->close());
    logger_->info("Client 2 close");
    ASSERT_TRUE(client2_->close());
    ASSERT_FALSE(client1_->close());
    ASSERT_FALSE(client2_->close());

    ASSERT_TRUE(server_->close());
}

TEST_P(SocketsShould, writeReadSuccessFromClient) {
    ASSERT_TRUE(server_->open());

    auto waitForConnection = std::async(std::launch::async, [this] {
        return server_->acceptConnection();
    });
    ASSERT_TRUE(client1_->open());
    auto serverSocketOpt = waitForConnection.get();
    ASSERT_TRUE(serverSocketOpt);
    auto serverSock = serverSocketOpt.get();

    std::string buffer("Ciao");
    ASSERT_TRUE(client1_->write(buffer));
    std::string read = serverSock->read(4, std::chrono::milliseconds(100));
    ASSERT_EQ(read.length(), 4);
    ASSERT_TRUE(serverSock->write(buffer));
    read = client1_->read(4, std::chrono::milliseconds(100));
    ASSERT_EQ(read.length(), 4);

    ASSERT_TRUE(client1_->close());
    ASSERT_TRUE(server_->close());
}

TEST_P(SocketsShould, readTimeout) {
    ASSERT_TRUE(server_->open());

    auto waitForConnection = std::async(std::launch::async, [this] {
        return server_->acceptConnection();
    });
    ASSERT_TRUE(client1_->open());
    auto serverSocketOpt = waitForConnection.get();
    ASSERT_TRUE(serverSocketOpt);
    auto serverSock = serverSocketOpt.get();

    std::string read = serverSock->read(4, std::chrono::milliseconds(5));
    ASSERT_EQ(read.length(), 0);
    read = client1_->read(4, std::chrono::milliseconds(5));
    ASSERT_EQ(read.length(), 0);

    ASSERT_TRUE(client1_->close());
    ASSERT_TRUE(server_->close());
}

TEST_P(SocketsShould, acceptSocketReturnsIfServerGetsClosed) {
    ASSERT_TRUE(server_->open());
    auto waitForConnection = std::async(std::launch::async, [this] {
        return server_->acceptConnection();
    });

    ASSERT_TRUE(server_->close());
    ASSERT_FALSE(waitForConnection.get());
}

TEST_P(SocketsShould, correctlyUseBlockingAndNonBlockingRead) {
    ASSERT_TRUE(server_->open());

    auto waitForConnection = std::async(std::launch::async, [this] {
        return server_->acceptConnection();
    });
    ASSERT_TRUE(client1_->open());
    auto serverSocketOpt = waitForConnection.get();
    ASSERT_TRUE(serverSocketOpt);
    auto serverSock = serverSocketOpt.get();

    std::string buffer("Ciao");
    ASSERT_TRUE(serverSock->write(buffer));
    std::string read = client1_->read(4, std::chrono::milliseconds(100));
    ASSERT_EQ(read.length(), 4);
    ASSERT_TRUE(serverSock->write(buffer));
    read = client1_->read(4);
    ASSERT_EQ(read.length(), 4);

    ASSERT_TRUE(client1_->close());
    ASSERT_TRUE(server_->close());
}

TEST_P(SocketsShould, correctlyUseFullyNonBlockingRead) {
    ASSERT_TRUE(server_->open());

    auto waitForConnection = std::async(std::launch::async, [this] {
        return server_->acceptConnection();
    });
    ASSERT_TRUE(client1_->open());
    auto serverSocketOpt = waitForConnection.get();
    ASSERT_TRUE(serverSocketOpt);
    auto serverSock = serverSocketOpt.get();

    std::string buffer("Ciao");
    ASSERT_TRUE(serverSock->write(buffer));
    std::string read = client1_->read(4, std::chrono::milliseconds(0));
    ASSERT_GT(read.length(), 0);
    ASSERT_TRUE(serverSock->write(buffer));

    ASSERT_TRUE(client1_->close());
    ASSERT_TRUE(server_->close());
}

TEST_P(SocketsShould, readOperationReturnsIfSameSocketGetsClosedFullBlocking) {
    std::mutex mtx;
    std::condition_variable cv;
    bool inread = false;
    ASSERT_TRUE(server_->open());

    auto waitForConnection = std::async(std::launch::async, [this] {
        return server_->acceptConnection();
    });
    ASSERT_TRUE(client1_->open());
    auto serverSocketOpt = waitForConnection.get();
    ASSERT_TRUE(serverSocketOpt);
    auto serverSock = serverSocketOpt.get();

    auto readReturn = std::async(std::launch::async, [this, &mtx, &cv, &inread] {
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv.notify_all();
            inread = true;
        }
        auto read = client1_->read(4);
        return read.length() != 0;
    });

    std::unique_lock<std::mutex> lock(mtx);
    ASSERT_TRUE(cv.wait_for(lock, std::chrono::seconds(1), [&inread] {
            return inread;
        }));
    ASSERT_TRUE(client1_->close());
    ASSERT_FALSE(readReturn.get());
    ASSERT_TRUE(server_->close());
}

TEST_P(SocketsShould, readOperationReturnsIfSameSocketGetsClosedTimeout) {
    std::mutex mtx;
    std::condition_variable cv;
    bool inread = false;
    ASSERT_TRUE(server_->open());

    auto waitForConnection = std::async(std::launch::async, [this] {
        return server_->acceptConnection();
    });
    ASSERT_TRUE(client1_->open());
    auto serverSocketOpt = waitForConnection.get();
    ASSERT_TRUE(serverSocketOpt);
    auto serverSock = serverSocketOpt.get();

    auto readReturn = std::async(std::launch::async, [this, &mtx, &cv, &inread] {
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv.notify_all();
            inread = true;
        }
        auto read = client1_->read(4, std::chrono::seconds(10));
        return read.length() != 0;
    });

    std::unique_lock<std::mutex> lock(mtx);
    ASSERT_TRUE(cv.wait_for(lock, std::chrono::seconds(1), [&inread] {
            return inread;
        }));
    ASSERT_TRUE(client1_->close());
    ASSERT_FALSE(readReturn.get());
    ASSERT_TRUE(server_->close());
}

TEST_P(SocketsShould, DISABLED_readOperationReturnsPartialBufferIfSameSocketGetsClosedTimeout) {
    std::mutex mtx;
    std::condition_variable cv;
    bool inread = false;
    ASSERT_TRUE(server_->open());

    auto waitForConnection = std::async(std::launch::async, [this] {
        return server_->acceptConnection();
    });
    ASSERT_TRUE(client1_->open());
    auto serverSocketOpt = waitForConnection.get();
    ASSERT_TRUE(serverSocketOpt);
    auto serverSock = serverSocketOpt.get();

    serverSock->write("Ci");

    auto readReturn = std::async(std::launch::async, [this, &mtx, &cv, &inread] {
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv.notify_all();
            inread = true;
        }
        auto read = client1_->read(4, std::chrono::seconds(10));
        return read.length();
    });

    std::unique_lock<std::mutex> lock(mtx);
    ASSERT_TRUE(cv.wait_for(lock, std::chrono::seconds(1), [&inread] {
            return inread;
        }));
    ASSERT_TRUE(client1_->close());
    ASSERT_EQ(readReturn.get(), 2);
    ASSERT_TRUE(server_->close());
}


TEST_P(SocketsShould, readWriteOperationReturnsIfServerSocketGetsClosed) {
    ASSERT_TRUE(server_->open());

    auto waitForConnection = std::async(std::launch::async, [this] {
        return server_->acceptConnection();
    });
    ASSERT_TRUE(client1_->open());
    auto serverSocketOpt = waitForConnection.get();
    ASSERT_TRUE(serverSocketOpt);
    auto serverSock = serverSocketOpt.get();


    auto readReturn = std::async(std::launch::async, [this] {
        auto read = client1_->read(4);
        return read.length() != 0;
    });

    ASSERT_TRUE(serverSock->close());

    ASSERT_FALSE(readReturn.get());
    readReturn = std::async(std::launch::async, [this] {
        auto read = client1_->read(4);
        return read.length() != 0;
    });
    ASSERT_FALSE(readReturn.get());
    readReturn = std::async(std::launch::async, [this] {
        auto read = client1_->read(4, std::chrono::milliseconds(100));
        return read.length() != 0;
    });
    ASSERT_FALSE(readReturn.get());
    ASSERT_TRUE(client1_->close());
    ASSERT_TRUE(server_->close());
}

TEST_P(SocketsShould, readWriteOperationReturnsIfClientSocketGetsClosed) {
    ASSERT_TRUE(server_->open());

    auto waitForConnection = std::async(std::launch::async, [this] {
        return server_->acceptConnection();
    });
    ASSERT_TRUE(client1_->open());
    auto serverSocketOpt = waitForConnection.get();
    ASSERT_TRUE(serverSocketOpt);
    auto serverSock = serverSocketOpt.get();


    auto readReturn = std::async(std::launch::async, [this, &serverSock] {
        auto read = serverSock->read(4);
        return read.length() != 0;
    });

    ASSERT_TRUE(client1_->close());
    ASSERT_FALSE(readReturn.get());
    readReturn = std::async(std::launch::async, [this, &serverSock] {
        auto read = serverSock->read(4);
        return read.length() != 0;
    });
    ASSERT_FALSE(readReturn.get());
    readReturn = std::async(std::launch::async, [this, &serverSock] {
        auto read = serverSock->read(4, std::chrono::milliseconds(100));
        return read.length() != 0;
    });
    ASSERT_FALSE(readReturn.get());

    ASSERT_TRUE(server_->close());
}

TEST_P(SocketsShould, closeServerClosesConnectedClients) {
    ASSERT_TRUE(server_->open());

    auto waitForConnection = std::async(std::launch::async, [this] {
        return server_->acceptConnection();
    });
    ASSERT_TRUE(client1_->open());
    auto serverSocketOpt = waitForConnection.get();
    ASSERT_TRUE(serverSocketOpt);
    auto serverSock = serverSocketOpt.get();

    auto readReturn = std::async(std::launch::async, [this] {
        auto read = client1_->read(4);
        return read.length() != 0;
    });

    ASSERT_TRUE(server_->close());
    ASSERT_FALSE(readReturn.get());
    readReturn = std::async(std::launch::async, [this, &serverSock] {
        auto read = serverSock->read(4);
        return read.length() != 0;
    });
    ASSERT_FALSE(readReturn.get());
    readReturn = std::async(std::launch::async, [this, &serverSock] {
        auto read = serverSock->read(4, std::chrono::milliseconds(100));
        return read.length() != 0;
    });
    ASSERT_FALSE(readReturn.get());

    ASSERT_TRUE(client1_->close());
}

TEST_P(SocketsShould, performanceTest) {
    ASSERT_TRUE(server_->open());

    auto waitForConnection = std::async(std::launch::async, [this] {
        return server_->acceptConnection();
    });
    ASSERT_TRUE(client1_->open());
    auto serverSocketOpt = waitForConnection.get();

    std::chrono::time_point<std::chrono::high_resolution_clock>
        serverStartsWriting, clientStartsReading,
        serverStoppedWriting, clientStoppedReading;
    auto serverWrites = std::async(std::launch::async, [this, &serverSocketOpt, &serverStartsWriting, &serverStoppedWriting] { // NOLINT
        std::string buffer;
        buffer.resize(MB_100);
        serverStartsWriting = std::chrono::high_resolution_clock::now();
        serverSocketOpt.get()->write(buffer);
        serverStoppedWriting  = std::chrono::high_resolution_clock::now();
        return true;
    });

    auto clientReads = std::async(std::launch::async, [this, &clientStartsReading, &clientStoppedReading] { // NOLINT
        clientStartsReading = std::chrono::high_resolution_clock::now();
        std::string buffer = client1_->read(MB_100);
        clientStoppedReading = std::chrono::high_resolution_clock::now();
        return buffer.length() == MB_100;
    });

    ASSERT_TRUE(serverWrites.get());
    ASSERT_TRUE(clientReads.get());

    logger_->info("Server took {} us to write {} MB",
        std::chrono::duration_cast<std::chrono::microseconds>(serverStoppedWriting-serverStartsWriting).count(),  // NOLINT
        MB_100/1e6);
    logger_->info("Client took {} us to read {} MB",
        std::chrono::duration_cast<std::chrono::microseconds>(clientStoppedReading-clientStartsReading).count(),  // NOLINT
        MB_100/1e6);
    ASSERT_TRUE(client1_->close());
    ASSERT_TRUE(serverSocketOpt.get()->close());
    ASSERT_TRUE(server_->close());
}
