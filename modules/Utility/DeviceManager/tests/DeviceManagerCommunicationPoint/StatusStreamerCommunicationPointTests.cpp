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

#include "DeviceManager/DeviceManagerCommunicationPoint/StatusStreamerCommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"

#include "DeviceManager/DeviceManagerMock.hpp"
#include "Sockets/SocketMock.hpp"

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

class StatusStreamerCommunicationPointShould : public ::testing::Test {
 protected:
    StatusStreamerCommunicationPointShould() :
        logger_("StatusStreamerCommunicationPointShould"),
        deviceMock_(new NiceMock<crf::utility::devicemanager::DeviceManagerMock>),
        socketMock_(new NiceMock<crf::communication::sockets::SocketMock>),
        packetSocket_(new crf::communication::datapacketsocket::PacketSocket(socketMock_)),
        statusJSON_(),
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

    ~StatusStreamerCommunicationPointShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        configureDevice();
        configureSocket();
    }

    void configureDevice() {
        ON_CALL(*deviceMock_, getStatus()).WillByDefault(Invoke([this](){
            return statusJSON_;
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

    std::unique_ptr<crf::utility::devicemanager::StatusStreamerCommunicationPoint> sut_;
    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<NiceMock<crf::utility::devicemanager::DeviceManagerMock>> deviceMock_;
    std::shared_ptr<NiceMock<crf::communication::sockets::SocketMock>> socketMock_;
    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> packetSocket_;

    nlohmann::json statusJSON_;
    bool isSocketOpen_;
    std::mutex readMutex_;
    std::condition_variable readCv_;
    std::string bytesToRead_;
    std::mutex writeMutex_;
    std::condition_variable writeCv_;
    bool dataWritten_;
    std::string bytesWritten_;
};

TEST_F(StatusStreamerCommunicationPointShould, initializeDeinitializeSequence) {
    sut_.reset(new crf::utility::devicemanager::StatusStreamerCommunicationPoint(
        packetSocket_, deviceMock_));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(StatusStreamerCommunicationPointShould, opensTheSocketIfItWasNotOpen) {
    sut_.reset(new crf::utility::devicemanager::StatusStreamerCommunicationPoint(
        packetSocket_, deviceMock_));

    isSocketOpen_ = false;
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(isSocketOpen_);
    ASSERT_TRUE(sut_->deinitialize());
}

// TEST_F(StatusStreamerCommunicationPointShould, sendErrorOnWrongPacket) {
//     sut_.reset(new crf::utility::devicemanager::StatusStreamerCommunicationPoint(
//         packetSocket_, deviceMock_));
// }

TEST_F(StatusStreamerCommunicationPointShould, sendErrorIfMissingCommand) {
    sut_.reset(new crf::utility::devicemanager::StatusStreamerCommunicationPoint(
        packetSocket_, deviceMock_));

    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["comnd"] = "getStatus";
    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Cannot get command field from received json");

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(StatusStreamerCommunicationPointShould, sendErrorIfUnknownCommand) {
    sut_.reset(new crf::utility::devicemanager::StatusStreamerCommunicationPoint(
        packetSocket_, deviceMock_));

    ASSERT_TRUE(sut_->initialize());

    std::string unknownCommand = "In di dibinindi...Iiin...Indi...Indibinigui...";
    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = unknownCommand;
    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Unknown command: " + unknownCommand);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(StatusStreamerCommunicationPointShould, correctlyGetStatus) {
    sut_.reset(new crf::utility::devicemanager::StatusStreamerCommunicationPoint(
        packetSocket_, deviceMock_));

    ASSERT_TRUE(sut_->initialize());

    statusJSON_["status"] = "test1";

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "getStatus";
    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "getStatus");
    ASSERT_EQ(response.value().data["message"], statusJSON_);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(StatusStreamerCommunicationPointShould, sendErrorIfWrongStreamParameters) {
    sut_.reset(new crf::utility::devicemanager::StatusStreamerCommunicationPoint(
        packetSocket_, deviceMock_));

    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "startStreamStatus";
    json.data["frequency"] = "50";
    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Wrong parameters");

    json.data["frequency"] = 0;
    writePacket(json);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "error");
    ASSERT_EQ(response.value().data["message"], "Frequency not valid");

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(StatusStreamerCommunicationPointShould, correctlyStartStopStatusStream) {
    sut_.reset(new crf::utility::devicemanager::StatusStreamerCommunicationPoint(
        packetSocket_, deviceMock_));

    ASSERT_TRUE(sut_->initialize());

    statusJSON_["status"] = "test1";

    crf::communication::datapackets::JSONPacket startStreamJSON;
    startStreamJSON.data["command"] = "startStreamStatus";
    startStreamJSON.data["frequency"] = 100.0f;

    crf::communication::datapackets::JSONPacket stopStreamJSON;
    stopStreamJSON.data["command"] = "stopStreamStatus";

    writePacket(startStreamJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "streamStatus");
    ASSERT_EQ(response.value().data["message"], statusJSON_);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "streamStatus");
    ASSERT_EQ(response.value().data["message"], statusJSON_);

    writePacket(stopStreamJSON);

    ASSERT_FALSE(readJsonPacket());

    writePacket(startStreamJSON);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "streamStatus");
    ASSERT_EQ(response.value().data["message"], statusJSON_);

    ASSERT_TRUE(sut_->deinitialize());

    ASSERT_FALSE(readJsonPacket());
}

TEST_F(StatusStreamerCommunicationPointShould, correctlyStartStatusStreamWithDesiredFrequency) {
    sut_.reset(new crf::utility::devicemanager::StatusStreamerCommunicationPoint(
        packetSocket_, deviceMock_));

    ASSERT_TRUE(sut_->initialize());

    statusJSON_["status"] = "test1";

    crf::communication::datapackets::JSONPacket startStreamJSON;
    startStreamJSON.data["command"] = "startStreamStatus";
    startStreamJSON.data["frequency"] = 25.0f;

    writePacket(startStreamJSON);

    const int numberOfPacketsToRead = 51;
    auto start = std::chrono::high_resolution_clock::now();
    for (int i=0; i < numberOfPacketsToRead; i++) {
        auto response = readJsonPacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.value().data["command"], "reply");
        ASSERT_EQ(response.value().data["replyCommand"], "streamStatus");
        ASSERT_EQ(response.value().data["message"], statusJSON_);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    EXPECT_GE(duration, std::chrono::milliseconds(2000).count());

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(StatusStreamerCommunicationPointShould, stopStreamIfSocketCloses) {
    sut_.reset(new crf::utility::devicemanager::StatusStreamerCommunicationPoint(
        packetSocket_, deviceMock_));

    ASSERT_TRUE(sut_->initialize());

    statusJSON_["status"] = "test1";

    crf::communication::datapackets::JSONPacket startStreamJSON;
    startStreamJSON.data["command"] = "startStreamStatus";
    startStreamJSON.data["frequency"] = 100.0f;

    writePacket(startStreamJSON);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.value().data["command"], "reply");
    ASSERT_EQ(response.value().data["replyCommand"], "streamStatus");
    ASSERT_EQ(response.value().data["message"], statusJSON_);

    isSocketOpen_ = false;
    ASSERT_FALSE(readJsonPacket());

    ASSERT_TRUE(sut_->deinitialize());
}
