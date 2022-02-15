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
#include <condition_variable>
#include <optional>

#include <nlohmann/json.hpp>

#include "DeviceManager/DeviceManagerCommunicationPoint/PriorityAccessCommunicationPoint.hpp"
#include "DeviceManager/DeviceManagerWithPriorityAccess/DeviceManagerWithPriorityAccess.hpp"
#include "CommonInterfaces/IInitializable.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"

#include "CommonInterfaces/InitializableMock.hpp"
#include "Sockets/SocketMock.hpp"

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

class PriorityAccessCommunicationPointShould : public ::testing::Test {
 protected:
    PriorityAccessCommunicationPointShould() :
        logger_("PriorityAccessCommunicationPointShould"),
        deviceMock_(new NiceMock<crf::utility::commoninterfaces::InitializableMock>),
        manager_(new crf::utility::devicemanager::DeviceManagerWithPriorityAccess(deviceMock_)),
        socketMock_(new NiceMock<crf::communication::sockets::SocketMock>),
        packetSocket_(new crf::communication::datapacketsocket::PacketSocket(socketMock_)),
        initializeResult_(false),
        deinitializeResult_(true),
        isSocketOpen_(false),
        readMutex_(),
        readCv_(),
        bytesToRead_(),
        writeMutex_(),
        writeCv_(),
        dataWritten_(false),
        bytesWritten_() {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~PriorityAccessCommunicationPointShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        configureDevice();
        configureSocket();
    }

    void configureDevice() {
        ON_CALL(*deviceMock_, initialize()).WillByDefault(Invoke([this](){
            return initializeResult_;
        }));
        ON_CALL(*deviceMock_, deinitialize()).WillByDefault(Invoke([this](){
            return deinitializeResult_;
        }));
    }

    void configureSocket() {
        bytesWritten_.clear();
        bytesToRead_.clear();
        ON_CALL(*socketMock_, isOpen()).WillByDefault(Invoke([this]() {
            return isSocketOpen_;
        }));
        ON_CALL(*socketMock_, open()).WillByDefault(Invoke([this]() {
            isSocketOpen_ = true;
            return true;
        }));
        ON_CALL(*socketMock_, close()).WillByDefault(Invoke([this]() {
            std::unique_lock<std::mutex> lock(readMutex_);
            isSocketOpen_ = false;
            readCv_.notify_all();
            return true;
        }));
        ON_CALL(*socketMock_, read(_)).WillByDefault(Invoke([this](int length) {
            std::unique_lock<std::mutex> lock(readMutex_);
            while (bytesToRead_.length() < static_cast<size_t>(length)) {
                readCv_.wait_for(lock, std::chrono::milliseconds(10));
                if (!isSocketOpen_) {
                    return std::string();
                }
            }
            std::string bytes = bytesToRead_.substr(0, length);
            bytesToRead_ = bytesToRead_.substr(length, bytesToRead_.length());
            return bytes;
        }));
        ON_CALL(*socketMock_, write(_, _)).WillByDefault(
            Invoke([this](std::string buffer, bool ack) {
                std::unique_lock<std::mutex> lock(writeMutex_);
                if (!isSocketOpen_) {
                    return false;
                }
                bytesWritten_.append(buffer);
                dataWritten_ = true;
                writeCv_.notify_all();
                return true;
        }));
    }

    void writePacket(const crf::communication::datapackets::IPacket& packet) {
        std::unique_lock<std::mutex> lock(readMutex_);
        bytesToRead_.append("~~~~~~");
        bytesToRead_.append(packet.getHeader().serialize());
        bytesToRead_.append(packet.serialize());
        readCv_.notify_all();
    }

    std::optional<crf::communication::datapackets::JSONPacket> readJsonPacket() {
        {
            std::unique_lock<std::mutex> lock(writeMutex_);
            dataWritten_ = false;
            if (bytesWritten_.length() == 0) {
                if (!writeCv_.wait_for(lock, std::chrono::seconds(2), [this] {
                    return dataWritten_; })) {
                        return std::nullopt;
                }
            }
        }

        std::string sync = bytesWritten_.substr(0, 6);
        if (sync != "~~~~~~") {
            logger_->warn("Missed sync");
            return std::nullopt;
        }

        crf::communication::datapackets::PacketHeader header;
        std::string headerBytes = bytesWritten_.substr(6, header.size());
        if (!header.deserialize(headerBytes)) {
            logger_->warn("Failed to deserialize header");
            return std::nullopt;
        }
        crf::communication::datapackets::JSONPacket json;
        std::string jsonBytes = bytesWritten_.substr(6 + header.size(), header.length());
        if (!json.deserialize(jsonBytes)) {
            logger_->warn("Failed to deserialize packet: {}", jsonBytes);
            return std::nullopt;
        }
        bytesWritten_ = bytesWritten_.erase(0, 6 + header.size() + header.length());
        return json;
    }

    std::unique_ptr<crf::utility::devicemanager::PriorityAccessCommunicationPoint> sut_;
    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<NiceMock<crf::utility::commoninterfaces::InitializableMock>> deviceMock_;
    std::shared_ptr<crf::utility::devicemanager::DeviceManagerWithPriorityAccess> manager_;
    std::shared_ptr<NiceMock<crf::communication::sockets::SocketMock>> socketMock_;
    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> packetSocket_;

    bool initializeResult_;
    bool deinitializeResult_;
    bool isSocketOpen_;
    std::mutex readMutex_;
    std::condition_variable readCv_;
    std::string bytesToRead_;
    std::mutex writeMutex_;
    std::condition_variable writeCv_;
    bool dataWritten_;
    std::string bytesWritten_;
};

TEST_F(PriorityAccessCommunicationPointShould, initializeDeinitializeSequence) {
    sut_.reset(new crf::utility::devicemanager::PriorityAccessCommunicationPoint(
        packetSocket_, manager_));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(PriorityAccessCommunicationPointShould, correctlyLockControl) {
    sut_.reset(new crf::utility::devicemanager::PriorityAccessCommunicationPoint(
        packetSocket_, manager_));

    initializeResult_ = true;
    deinitializeResult_ = true;

    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "lockControl";
    json.data["priority"] = 1;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "lockControl");
    ASSERT_EQ(response.value().data["message"], true);

    json.data["priority"] = 10;

    writePacket(json);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "lockControl");
    ASSERT_EQ(response.value().data["message"], false);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(PriorityAccessCommunicationPointShould, invalidParametersToLockControl) {
    sut_.reset(new crf::utility::devicemanager::PriorityAccessCommunicationPoint(
        packetSocket_, manager_));

    initializeResult_ = true;
    deinitializeResult_ = true;

    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "lockControl";
    json.data["priodvs"] = 1;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    logger_->info("response {}", response.value().data["message"]);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Wrong parameters");

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(PriorityAccessCommunicationPointShould, invalidPriorityToLockControl) {
    sut_.reset(new crf::utility::devicemanager::PriorityAccessCommunicationPoint(
        packetSocket_, manager_));

    initializeResult_ = true;
    deinitializeResult_ = true;

    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "lockControl";
    json.data["priority"] = 0;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Priority not valid");

    json.data["priority"] = -10;

    writePacket(json);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Priority not valid");

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(PriorityAccessCommunicationPointShould, correctlyLockUnlockControl) {
    sut_.reset(new crf::utility::devicemanager::PriorityAccessCommunicationPoint(
        packetSocket_, manager_));

    initializeResult_ = true;
    deinitializeResult_ = true;

    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "lockControl";
    json.data["priority"] = 1;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "lockControl");
    ASSERT_EQ(response.value().data["message"], true);

    json.data["command"] = "unlockControl";
    json.data["priority"] = 12;

    writePacket(json);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "unlockControl");
    ASSERT_EQ(response.value().data["message"], false);

    json.data["priority"] = 1;

    writePacket(json);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "unlockControl");
    ASSERT_EQ(response.value().data["message"], true);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(PriorityAccessCommunicationPointShould, invalidParametersToUnlockControl) {
    sut_.reset(new crf::utility::devicemanager::PriorityAccessCommunicationPoint(
        packetSocket_, manager_));

    initializeResult_ = true;
    deinitializeResult_ = true;

    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "unlockControl";
    json.data["prio"] = 1;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Wrong parameters");

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(PriorityAccessCommunicationPointShould, invalidPriorityToUnlockControl) {
    sut_.reset(new crf::utility::devicemanager::PriorityAccessCommunicationPoint(
        packetSocket_, manager_));

    initializeResult_ = true;
    deinitializeResult_ = true;

    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "unlockControl";
    json.data["priority"] = 0;

    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Priority not valid");

    json.data["priority"] = -10;

    writePacket(json);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Priority not valid");

    ASSERT_TRUE(sut_->deinitialize());
}
