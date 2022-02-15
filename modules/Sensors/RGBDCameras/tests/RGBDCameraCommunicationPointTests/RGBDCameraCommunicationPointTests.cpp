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

#include "RGBDCameras/RGBDCameraCommunicationPoint/RGBDCameraManager.hpp"
#include "RGBDCameras/RGBDCameraCommunicationPoint/RGBDCameraCommunicationPoint.hpp"
#include "DataPackets/RGBDFramePacket/RGBDFramePacket.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "../IRGBDCameraSimulator.hpp"
#include "EventLogger/EventLogger.hpp"
#include "RGBDCameras/RGBDCameraMock.hpp"
#include "Sockets/SocketMock.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::sensors::rgbdcameras::RGBDCameraMock;
using crf::sensors::rgbdcameras::RGBDCameraManager;
using crf::sensors::rgbdcameras::RGBDCameraCommunicationPoint;
using crf::communication::datapackets::RGBDFramePacket;
using crf::communication::datapacketsocket::PacketSocket;
using crf::communication::datapackets::JSONPacket;
using crf::communication::sockets::SocketMock;

class RGBDCameraCommunicationPointShould : public ::testing::Test {
 protected:
    RGBDCameraCommunicationPointShould() :
        logger_("RGBDCameraCommunicationPointShould"),
        isSocketOpen_(true),
        readMutex_(),
        readCv_(),
        bytesToRead_(),
        cameraMock_(new NiceMock<RGBDCameraMock>),
        socketMock_(new NiceMock<SocketMock>),
        simulator_(new IRGBDCameraSimulator(cameraMock_)),
        manager_(new RGBDCameraManager(cameraMock_, std::chrono::milliseconds(0))),
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

    ~RGBDCameraCommunicationPointShould() {
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

    boost::optional<JSONPacket> readJsonPacket() {
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

        JSONPacket json;
        std::string jsonBytes = bytesWritten_.substr(6 + header.size(), header.length());
        if (!json.deserialize(jsonBytes)) {
            logger_->warn("Failed to deserialize packet: {}", jsonBytes);
            return boost::none;
        }

        bytesWritten_ = bytesWritten_.erase(0, 6 + header.size() + header.length());

        return json;
    }

    boost::optional<crf::communication::datapackets::RGBDFramePacket> readRGBDFramePacket() {
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

        crf::communication::datapackets::RGBDFramePacket frame;
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

    std::shared_ptr<RGBDCameraMock> cameraMock_;
    std::shared_ptr<SocketMock> socketMock_;
    std::unique_ptr<IRGBDCameraSimulator> simulator_;
    std::shared_ptr<RGBDCameraManager> manager_;
    std::shared_ptr<PacketSocket> packetSocket_;

    std::unique_ptr<RGBDCameraCommunicationPoint> sut_;
};

TEST_F(RGBDCameraCommunicationPointShould, initializeDeinitializeSequence) {
    sut_.reset(new RGBDCameraCommunicationPoint(packetSocket_, manager_));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(RGBDCameraCommunicationPointShould, opensTheSocketIfItWasNotOpen) {
    sut_.reset(new RGBDCameraCommunicationPoint(packetSocket_, manager_));
    isSocketOpen_ = false;
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(isSocketOpen_);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RGBDCameraCommunicationPointShould, sendErrorOnWrongPacket) {
    sut_.reset(new RGBDCameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());

    crf::communication::datapackets::RGBDFramePacket frame;
    writePacket(frame);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["status"], "error");

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RGBDCameraCommunicationPointShould, sendErrorIfMissingCmd) {
    sut_.reset(new RGBDCameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());

    JSONPacket json;
    json.data["cmda"] = "Ciao";
    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["status"], "error");

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RGBDCameraCommunicationPointShould, sendErrorIfUnknownCommand) {
    sut_.reset(new RGBDCameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());

    JSONPacket json;
    json.data["cmd"] = "Ciao";
    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["status"], "error");

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RGBDCameraCommunicationPointShould, correctlySetAndGetCameraParameters) {
    EXPECT_CALL(*cameraMock_, setZoom(1.0f)).Times(1);
    sut_.reset(new RGBDCameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());

    JSONPacket json;
    json.data["cmd"] = "setparam";
    json.data["zoom"] = 1.0f;
    writePacket(json);

    json.data["cmd"] = "getparam";
    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["cmd"], "response");
    ASSERT_EQ(response.get().data["response_cmd"], "getparam");
    ASSERT_NEAR(response.get().data["params"]["zoom"].get<float>(), 0.0, 1e-3);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RGBDCameraCommunicationPointShould, getErrorOnNotActiveStopStream) {
    sut_.reset(new RGBDCameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());

    JSONPacket json;
    json.data["cmd"] = "stopstream";
    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["status"], "error");

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RGBDCameraCommunicationPointShould, getErrorOnWrongFromOfStartStream) {
    sut_.reset(new RGBDCameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());

    JSONPacket json;
    json.data["cmd"] = "getstream";
    json.data["resolution"] = "getstream";
    writePacket(json);

    auto response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["status"].get<std::string>(), "error");

    std::vector<uint32_t> resolution;
    resolution.push_back(1024);
    json.data["resolution"] = resolution;
    writePacket(json);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["status"].get<std::string>(), "error");
    resolution.push_back(768);
    json.data["resolution"] = resolution;
    writePacket(json);
    response = readJsonPacket();

    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["status"].get<std::string>(), "error");

    json.data["format"] = "ciao";
    writePacket(json);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["status"].get<std::string>(), "error");

    json.data["format"] = "jpeg";
    writePacket(json);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["status"].get<std::string>(), "error");

    json.data["quality"] = 100;
    writePacket(json);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["status"].get<std::string>(), "error");

    json.data["quality"] = 5;
    writePacket(json);

    response = readJsonPacket();
    ASSERT_TRUE(response);
    ASSERT_EQ(response.get().data["status"].get<std::string>(), "error");

    ASSERT_TRUE(sut_->deinitialize());
}

/** This test is disabled because it will fail with valgrind and stress,
 * but it works in normal operation **/
TEST_F(RGBDCameraCommunicationPointShould, DISABLED_correctlyReceiveFramesWith30Fps) {
    sut_.reset(new RGBDCameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());

    JSONPacket json;
    json.data["cmd"] = "getstream";
    json.data["resolution"] = std::vector<uint32_t>({100, 100});
    json.data["format"] = "jpeg";
    json.data["quality"] = 5;
    json.data["fps"] = 30;
    writePacket(json);
    const int numberOfFramesToRead = 10;
    auto start = std::chrono::high_resolution_clock::now();
    for (int i=0; i < numberOfFramesToRead; i++) {
        auto response = readRGBDFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.get().getRGBEncoding(),
            crf::communication::datapackets::RGBDFramePacket::RGBEncoding::JPEG);
        ASSERT_NE(response.get().getRGBBytes().length(), 0);
        auto bytes = response.get().getRGBBytes();
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
TEST_F(RGBDCameraCommunicationPointShould, DISABLED_correctlyReceiveFramesWith5Fps) {
    sut_.reset(new RGBDCameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());

    JSONPacket json;
    json.data["cmd"] = "getstream";
    json.data["resolution"] = std::vector<uint32_t>({100, 100});
    json.data["format"] = "jpeg";
    json.data["quality"] = 5;
    json.data["fps"] = 5;
    writePacket(json);
    const int numberOfFramesToRead = 10;
    auto start = std::chrono::high_resolution_clock::now();
    for (int i=0; i < numberOfFramesToRead; i++) {
        auto response = readRGBDFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.get().getRGBEncoding(),
            crf::communication::datapackets::RGBDFramePacket::RGBEncoding::JPEG);
        ASSERT_NE(response.get().getRGBBytes().length(), 0);
        auto bytes = response.get().getRGBBytes();
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

TEST_F(RGBDCameraCommunicationPointShould, startStreamsJpegAndCorrectlyReceiveFrames) {
    sut_.reset(new RGBDCameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());

    JSONPacket json;
    json.data["cmd"] = "getstream";
    json.data["resolution"] = std::vector<uint32_t>({100, 100});
    json.data["format"] = "jpeg";
    json.data["quality"] = 5;
    json.data["fps"] = 30;
    writePacket(json);
    const int numberOfFramesToRead = 10;
    for (int i=0; i < numberOfFramesToRead; i++) {
        auto response = readRGBDFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.get().getRGBEncoding(),
            crf::communication::datapackets::RGBDFramePacket::RGBEncoding::JPEG);
        ASSERT_NE(response.get().getRGBBytes().length(), 0);
        auto bytes = response.get().getRGBBytes();
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
        auto response = readRGBDFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.get().getRGBEncoding(),
            crf::communication::datapackets::RGBDFramePacket::RGBEncoding::JPEG);
        ASSERT_NE(response.get().getRGBBytes().length(), 0);

        auto bytes = response.get().getRGBBytes();
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

    json.data["cmd"] = "stopstream";
    writePacket(json);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RGBDCameraCommunicationPointShould, startStreamsx264AndCorrectlyReceiveFrames) {
    sut_.reset(new RGBDCameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());

    JSONPacket json;
    json.data["cmd"] = "getstream";
    json.data["resolution"] = std::vector<uint32_t>({100, 100});
    json.data["format"] = "x264";
    json.data["quality"] = 5;
    json.data["fps"] = 10;
    writePacket(json);

    for (int i=0; i < 5; i++) {
        auto response = readRGBDFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.get().getRGBEncoding(),
            crf::communication::datapackets::RGBDFramePacket::RGBEncoding::X264);
        ASSERT_NE(response.get().getRGBBytes().length(), 0);
    }

    json.data["resolution"] = std::vector<uint32_t>({150, 150});
    writePacket(json);

    for (int i=0; i < 5; i++) {
        auto response = readRGBDFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.get().getRGBEncoding(),
            crf::communication::datapackets::RGBDFramePacket::RGBEncoding::X264);
        ASSERT_NE(response.get().getRGBBytes().length(), 0);
    }

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RGBDCameraCommunicationPointShould, correctlyChangeCompressionMethod) {
    sut_.reset(new RGBDCameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());

    JSONPacket json;
    json.data["cmd"] = "getstream";
    json.data["resolution"] = std::vector<uint32_t>({100, 100});
    json.data["format"] = "jpeg";
    json.data["quality"] = 5;
    json.data["fps"] = 30;
    writePacket(json);
    const int numberOfFramesToRead = 10;
    for (int i=0; i < numberOfFramesToRead; i++) {
        auto response = readRGBDFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.get().getRGBEncoding(),
            crf::communication::datapackets::RGBDFramePacket::RGBEncoding::JPEG);
        ASSERT_NE(response.get().getRGBBytes().length(), 0);
        auto bytes = response.get().getRGBBytes();
        auto init = bytes.begin()+8;
        std::vector<char> buffer_vec(init, bytes.end());
        auto mat = cv::imdecode(buffer_vec, cv::IMREAD_COLOR);
        ASSERT_EQ(mat.cols, 100);
        ASSERT_EQ(mat.rows, 100);
    }

    json.data["format"] = "cvmat";
    writePacket(json);

    bool resolutionChanged = false;
    do {
        auto response = readRGBDFramePacket();
        ASSERT_TRUE(response);
        if (response.get().getRGBEncoding() !=
            crf::communication::datapackets::RGBDFramePacket::RGBEncoding::JPEG) {
            ASSERT_EQ(response.get().getRGBEncoding(),
                crf::communication::datapackets::RGBDFramePacket::RGBEncoding::CV_MAT);
            ASSERT_NE(response.get().getRGBBytes().length(), 0);
            response.get().getRGBEncoding();
            resolutionChanged = true;
        }
    } while (!resolutionChanged);

    json.data["cmd"] = "stopstream";
    writePacket(json);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RGBDCameraCommunicationPointShould, correctlyChangeDepthCompressionMethod) {
    sut_.reset(new RGBDCameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());

    JSONPacket json;
    json.data["cmd"] = "getstream";
    json.data["resolution"] = std::vector<uint32_t>({100, 100});
    json.data["depth_format"] = "cvmat";
    json.data["fps"] = 30;
    writePacket(json);
    const int numberOfFramesToRead = 10;
    for (int i=0; i < numberOfFramesToRead; i++) {
        auto response = readRGBDFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.get().getDepthEncoding(),
            crf::communication::datapackets::RGBDFramePacket::DepthEncoding::CV_MAT);
        ASSERT_NE(response.get().getDepthBytes().length(), 0);
    }

    json.data["depth_format"] = "lz4";
    writePacket(json);

    bool resolutionChanged = false;
    do {
        auto response = readRGBDFramePacket();
        ASSERT_TRUE(response);
        if (response.get().getDepthEncoding() !=
            crf::communication::datapackets::RGBDFramePacket::DepthEncoding::CV_MAT) {
            ASSERT_EQ(response.get().getDepthEncoding(),
                crf::communication::datapackets::RGBDFramePacket::DepthEncoding::LZ4);
            ASSERT_NE(response.get().getDepthBytes().length(), 0);
            resolutionChanged = true;
        }
    } while (!resolutionChanged);

    json.data["cmd"] = "stopstream";
    writePacket(json);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RGBDCameraCommunicationPointShould, requestBothDepthAndColorAligned) {
    sut_.reset(new RGBDCameraCommunicationPoint(packetSocket_, manager_));
    ASSERT_TRUE(sut_->initialize());

    JSONPacket json;
    json.data["cmd"] = "getstream";
    json.data["resolution"] = std::vector<uint32_t>({100, 100});
    json.data["format"] = "jpeg";
    json.data["quality"] = 6;
    json.data["depth_format"] = "lz4";
    json.data["fps"] = 30;
    json.data["align_frames"] = true;
    writePacket(json);
    const int numberOfFramesToRead = 10;
    for (int i=0; i < numberOfFramesToRead; i++) {
        auto response = readRGBDFramePacket();
        ASSERT_TRUE(response);
        ASSERT_EQ(response.get().getRGBEncoding(),
            crf::communication::datapackets::RGBDFramePacket::RGBEncoding::JPEG);
        ASSERT_NE(response.get().getRGBBytes().length(), 0);
        ASSERT_EQ(response.get().getDepthEncoding(),
            crf::communication::datapackets::RGBDFramePacket::DepthEncoding::LZ4);
        ASSERT_NE(response.get().getDepthBytes().length(), 0);
    }

    json.data["cmd"] = "stopstream";
    writePacket(json);

    ASSERT_TRUE(sut_->deinitialize());
}

