/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */


#include <condition_variable>
#include <future>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

#include "Cameras/CameraCommunicationPoint/CameraManager.hpp"
#include "Cameras/CameraCommunicationPoint/CameraCommunicationPoint.hpp"
#include "DataPackets/FramePacket/FramePacket.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "../ICameraSimulator.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Cameras/CameraMock.hpp"
#include "Sockets/SocketMock.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::sensors::cameras::CameraMock;
using crf::sensors::cameras::CameraManager;
using crf::sensors::cameras::CameraCommunicationPoint;
using crf::communication::datapackets::FramePacket;
using crf::communication::datapacketsocket::PacketSocket;
using crf::communication::sockets::SocketMock;

class CameraCommunicationPointShould : public ::testing::Test {
 protected:
    CameraCommunicationPointShould() :
        logger_("CameraCommunicationPointShould"),
        isSocketOpen_(true),
        readMutex_(),
        readCv_(),
        bytesToRead_(),
        cameraMock_(new NiceMock<CameraMock>),
        socketMock_(new NiceMock<SocketMock>),
        simulator_(new ICameraSimulator(cameraMock_)),
        manager_(new CameraManager(cameraMock_, std::chrono::seconds(0))),
        packetSocket_(new PacketSocket(socketMock_)) {
            logger_->info("{0} BEGIN",
                testing::UnitTest::GetInstance()->current_test_info()->name());
            simulator_->initializeDelay = std::chrono::milliseconds(1);
    }

    void SetUp() override {
        bytesWritten_.clear();
        bytesToRead_.clear();

        ON_CALL(*socketMock_, isOpen()).WillByDefault(Invoke([this]() { return isSocketOpen_; }));
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

    ~CameraCommunicationPointShould() {
        logger_->info("{0} END with {1}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void writePacket(const crf::communication::datapackets::IPacket& packet) {
        std::unique_lock<std::mutex> lock(readMutex_);
        bytesToRead_.append("~~~~~~");
        bytesToRead_.append(packet.getHeader().serialize());
        bytesToRead_.append(packet.serialize());
        readCv_.notify_all();
    }

    boost::optional<crf::communication::datapackets::JSONPacket> readJsonPacket() {
        {
            std::unique_lock<std::mutex> lock(writeMutex_);
            dataWritten_ = false;
            if (bytesWritten_.length() == 0) {
                if (!writeCv_.wait_for(lock, std::chrono::seconds(2), [this] {
                    return dataWritten_; })) {
                        return boost::none;
                }
            }
        }

        std::string sync = bytesWritten_.substr(0, 6);
        if (sync != "~~~~~~") {
            logger_->warn("Missed sync");
            return boost::none;
        }

        crf::communication::datapackets::PacketHeader header;
        std::string headerBytes = bytesWritten_.substr(6, header.size());
        if (!header.deserialize(headerBytes)) {
            logger_->warn("Failed to deserialize header");
            return boost::none;
        }

        crf::communication::datapackets::JSONPacket json;
        std::string jsonBytes = bytesWritten_.substr(6 + header.size(), header.length());
        if (!json.deserialize(jsonBytes)) {
            logger_->warn("Failed to deserialize packet: {}", jsonBytes);
            return boost::none;
        }
        bytesWritten_ = bytesWritten_.erase(0, 6 + header.size() + header.length());
        return json;
    }

    boost::optional<FramePacket> readFramePacket() {
        std::unique_lock<std::mutex> lock(writeMutex_);
        dataWritten_ = false;
        if (bytesWritten_.length() == 0) {
            writeCv_.wait(lock);
        }


        std::string sync = bytesWritten_.substr(0, 6);
        if (sync != "~~~~~~") {
            logger_->warn("Missed sync");
            return boost::none;
        }

        crf::communication::datapackets::PacketHeader header;
        std::string headerBytes = bytesWritten_.substr(6, header.size());
        if (!header.deserialize(headerBytes)) {
            logger_->warn("Failed to deserialize header");
            return boost::none;
        }

        FramePacket frame;
        std::string frameBytes = bytesWritten_.substr(6 + header.size(), header.length());
        if (!frame.deserialize(frameBytes)) {
            logger_->warn("Failed to deserialize packet");
        }
        bytesWritten_ = bytesWritten_.erase(0, 6 + header.size() + header.length());

        return frame;
    }

    crf::utility::logger::EventLogger logger_;

    bool isSocketOpen_;
    std::mutex readMutex_;
    std::condition_variable readCv_;
    std::string bytesToRead_;
    std::mutex writeMutex_;
    std::condition_variable writeCv_;
    bool dataWritten_;
    std::string bytesWritten_;

    std::shared_ptr<CameraMock> cameraMock_;
    std::shared_ptr<SocketMock> socketMock_;
    std::unique_ptr<ICameraSimulator> simulator_;
    std::shared_ptr<CameraManager> manager_;
    std::shared_ptr<PacketSocket> packetSocket_;

    std::unique_ptr<CameraCommunicationPoint> sut_;
};

TEST_F(CameraCommunicationPointShould, initializeDeinitializeSequence) {
    sut_.reset(new CameraCommunicationPoint(packetSocket_, manager_));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(sut_->deinitialize());

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CameraCommunicationPointShould, opensTheSocketIfItWasNotOpen) {
    sut_.reset(new CameraCommunicationPoint(packetSocket_, manager_));
    isSocketOpen_ = false;
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(isSocketOpen_);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CameraCommunicationPointShould, sendErrorOnWrongPacket) {
    sut_.reset(new CameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());

    FramePacket frame(std::string("ciao"),
        crf::communication::datapackets::FramePacket::Encoding::JPEG);
    writePacket(frame);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["command"], "reply");
    ASSERT_EQ(response.get().data["replyCommand"], "error");

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CameraCommunicationPointShould, sendErrorIfMissingCmd) {
    sut_.reset(new CameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["cmda"] = "Ciao";
    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["command"], "reply");
    ASSERT_EQ(response.get().data["replyCommand"], "error");

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CameraCommunicationPointShould, sendErrorIfUnknownCommand) {
    sut_.reset(new CameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "Ciao";
    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["command"], "reply");
    ASSERT_EQ(response.get().data["replyCommand"], "error");

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CameraCommunicationPointShould, DISABLED_correctlySetAndGetCameraParameters) {
    // This test is wrongly designed.
    // The next incoming message can be a frame or a status message
    // It is a "data race"
    EXPECT_CALL(*cameraMock_, setZoom(1.0f)).Times(1);
    sut_.reset(new CameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());
    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "startFrameStream";
    json.data["resolution"] = std::vector<uint32_t>({100, 100});
    json.data["encoding_format"] = "jpeg";
    json.data["encoding_quality"] = 5;
    json.data["camera_framerate"] = 10;
    json.data["stream_framerate"] = 10;
    writePacket(json);

    json.data["command"] = "setStatus";
    json.data["zoom"] = 1.0f;
    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["command"], "reply");
    ASSERT_EQ(response.get().data["replyCommand"], "setStatus");

    json.data["command"] = "getStatus";
    writePacket(json);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["command"], "reply");
    ASSERT_EQ(response.get().data["replyCommand"], "getStatus");
    ASSERT_NEAR(response.get().data["message"]["zoom"].get<float>(), 1.0, 1e-3);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CameraCommunicationPointShould, getErrorOnNotActiveStopStream) {
    sut_.reset(new CameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "stopFrameStream";
    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["replyCommand"], "error");

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CameraCommunicationPointShould, getErrorOnWrongFromOfStartStream) {
    sut_.reset(new CameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "startFrameStream";
    json.data["resolution"] = "getstream";
    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["replyCommand"].get<std::string>(), "error");

    std::vector<uint32_t> resolution;
    resolution.push_back(1024);
    json.data["resolution"] = resolution;
    writePacket(json);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["replyCommand"].get<std::string>(), "error");
    resolution.push_back(768);
    json.data["resolution"] = resolution;
    writePacket(json);
    response = readJsonPacket();

    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["replyCommand"].get<std::string>(), "error");

    json.data["encoding_format"] = "ciao";
    writePacket(json);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["replyCommand"].get<std::string>(), "error");

    json.data["encoding_format"] = "jpeg";
    writePacket(json);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["replyCommand"].get<std::string>(), "error");

    json.data["encoding_quality"] = 100;
    writePacket(json);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["replyCommand"].get<std::string>(), "error");

    json.data["encoding_quality"] = 5;
    writePacket(json);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["replyCommand"].get<std::string>(), "error");

    ASSERT_TRUE(sut_->deinitialize());
}

/** This test is disabled because it will fail with valgrind and stress,
 * but it works in normal operation **/
TEST_F(CameraCommunicationPointShould, DISABLED_correctlyReceiveFramesWith30Fps) {
    sut_.reset(new CameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["cmd"] = "getstream";
    json.data["resolution"] = std::vector<uint32_t>({100, 100});
    json.data["format"] = "jpeg";
    json.data["quality"] = 5;
    json.data["fps"] = 30;
    writePacket(json);
    const int numberOfFramesToRead = 10;
    auto start = std::chrono::high_resolution_clock::now();
    for (int i=0; i < numberOfFramesToRead; i++) {
        auto response = readFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.get().getEncodingType(),
            crf::communication::datapackets::FramePacket::Encoding::JPEG);
        ASSERT_NE(response.get().getBytes().length(), 0);
        auto bytes = response.get().getBytes();
        auto init = bytes.begin()+4;
        std::vector<char> buffer_vec(init, bytes.end());
        auto mat = cv::imdecode(buffer_vec, cv::IMREAD_COLOR);
        ASSERT_EQ(mat.cols, 100);
        ASSERT_EQ(mat.rows, 100);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    ASSERT_GT(duration, (1000/json.data["fps"].get<int>())*(numberOfFramesToRead-1));
    ASSERT_LT(duration, (1000/json.data["fps"].get<int>())*(numberOfFramesToRead+1));
}

/** This test is disabled because it will fail with valgrind and stress,
 * but it works in normal operation **/
TEST_F(CameraCommunicationPointShould, DISABLED_correctlyReceiveFramesWith5Fps) {
    sut_.reset(new CameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["cmd"] = "getstream";
    json.data["resolution"] = std::vector<uint32_t>({100, 100});
    json.data["format"] = "jpeg";
    json.data["quality"] = 5;
    json.data["fps"] = 5;
    writePacket(json);
    const int numberOfFramesToRead = 10;
    auto start = std::chrono::high_resolution_clock::now();
    for (int i=0; i < numberOfFramesToRead; i++) {
        auto response = readFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.get().getEncodingType(),
            crf::communication::datapackets::FramePacket::Encoding::JPEG);
        ASSERT_NE(response.get().getBytes().length(), 0);
        auto bytes = response.get().getBytes();
        auto init = bytes.begin()+4;
        std::vector<char> buffer_vec(init, bytes.end());
        auto mat = cv::imdecode(buffer_vec, cv::IMREAD_COLOR);
        ASSERT_EQ(mat.cols, 100);
        ASSERT_EQ(mat.rows, 100);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    ASSERT_GT(duration, (1000/json.data["fps"].get<int>())*(numberOfFramesToRead-1));
    ASSERT_LT(duration, (1000/json.data["fps"].get<int>())*(numberOfFramesToRead+1));
}

TEST_F(CameraCommunicationPointShould, startStreamsJpegAndCorrectlyReceiveFrames) {
    sut_.reset(new CameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "startFrameStream";
    json.data["resolution"] = std::vector<uint32_t>({100, 100});
    json.data["encoding_format"] = "jpeg";
    json.data["encoding_quality"] = 5;
    json.data["stream_framerate"] = 30;
    writePacket(json);
    const int numberOfFramesToRead = 10;
    for (int i=0; i < numberOfFramesToRead; i++) {
        auto response = readFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.get().getEncodingType(),
            crf::communication::datapackets::FramePacket::Encoding::JPEG);
        ASSERT_NE(response.get().getBytes().length(), 0);
        auto bytes = response.get().getBytes();
        auto init = bytes.begin()+8;
        std::vector<char> buffer_vec(init, bytes.end());
        auto mat = cv::imdecode(buffer_vec, cv::IMREAD_COLOR);
        ASSERT_EQ(mat.cols, 100);
        ASSERT_EQ(mat.rows, 100);
    }

    json.data["resolution"] = std::vector<uint32_t>({150, 150});
    writePacket(json);

    bool resolutionChanged = false;

    do {
        auto response = readFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.get().getEncodingType(),
            crf::communication::datapackets::FramePacket::Encoding::JPEG);
        ASSERT_NE(response.get().getBytes().length(), 0);

        auto bytes = response.get().getBytes();
        auto init = bytes.begin()+8;
        std::vector<char> buffer_vec(init, bytes.end());
        auto mat = cv::imdecode(buffer_vec, cv::IMREAD_COLOR);
        if (mat.cols == 100) {
            ASSERT_EQ(mat.cols, 100);
            ASSERT_EQ(mat.rows, 100);
        } else {
            ASSERT_EQ(mat.cols, 150);
            ASSERT_EQ(mat.rows, 150);
            resolutionChanged = true;
        }
    } while (!resolutionChanged);

    json.data["command"] = "stopFrameStream";
    writePacket(json);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CameraCommunicationPointShould, startStreamsx264AndCorrectlyReceiveFrames) {
    sut_.reset(new CameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "startFrameStream";
    json.data["resolution"] = std::vector<uint32_t>({100, 100});
    json.data["encoding_format"] = "x264";
    json.data["encoding_quality"] = 5;
    json.data["stream_framerate"] = 10;
    writePacket(json);

    for (int i=0; i < 5; i++) {
        auto response = readFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.get().getEncodingType(),
            crf::communication::datapackets::FramePacket::Encoding::X264);
        ASSERT_NE(response.get().getBytes().length(), 0);
    }

    json.data["resolution"] = std::vector<uint32_t>({150, 150});
    writePacket(json);

    for (int i=0; i < 5; i++) {
        auto response = readFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.get().getEncodingType(),
            crf::communication::datapackets::FramePacket::Encoding::X264);
        ASSERT_NE(response.get().getBytes().length(), 0);
    }

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CameraCommunicationPointShould, correctlyChangeCompressionMethod) {
    sut_.reset(new CameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::JSONPacket json;
    json.data["command"] = "startFrameStream";
    json.data["resolution"] = std::vector<uint32_t>({100, 100});
    json.data["encoding_format"] = "jpeg";
    json.data["encoding_quality"] = 5;
    json.data["stream_framerate"] = 30;
    writePacket(json);
    const int numberOfFramesToRead = 10;
    for (int i=0; i < numberOfFramesToRead; i++) {
        auto response = readFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.get().getEncodingType(),
            crf::communication::datapackets::FramePacket::Encoding::JPEG);
        ASSERT_NE(response.get().getBytes().length(), 0);
        auto bytes = response.get().getBytes();
        auto init = bytes.begin()+8;
        std::vector<char> buffer_vec(init, bytes.end());
        auto mat = cv::imdecode(buffer_vec, cv::IMREAD_COLOR);
        ASSERT_EQ(mat.cols, 100);
        ASSERT_EQ(mat.rows, 100);
    }

    json.data["encoding_format"] = "cvmat";
    writePacket(json);

    bool resolutionChanged = false;
    do {
        auto response = readFramePacket();
        ASSERT_TRUE(response);
        if (response.get().getEncodingType() !=
            crf::communication::datapackets::FramePacket::Encoding::JPEG) {
            ASSERT_EQ(response.get().getEncodingType(),
                crf::communication::datapackets::FramePacket::Encoding::CV_MAT);
            ASSERT_NE(response.get().getBytes().length(), 0);
            response.get().getEncodingType();
            resolutionChanged = true;
        }
    } while (!resolutionChanged);

    json.data["command"] = "stopFrameStream";
    writePacket(json);

    ASSERT_TRUE(sut_->deinitialize());
}
