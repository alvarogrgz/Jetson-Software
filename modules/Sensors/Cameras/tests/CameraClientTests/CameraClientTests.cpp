/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#include <future>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "../ICameraSimulator.hpp"
#include "Cameras/CameraClient/CameraClient.hpp"
#include "DataPackets/FramePacket/FramePacket.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Sockets/SocketMock.hpp"
#include "Cameras/CameraMock.hpp"
#include "VideoCodecs/cvMatVideoCodec/cvMatVideoEncoder.hpp"
#include "VideoCodecs/JPEGVideoCodec/JPEGVideoEncoder.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::algorithms::videocodecs::IVideoEncoder;
using crf::algorithms::videocodecs::JPEGVideoEncoder;
using crf::algorithms::videocodecs::cvMatVideoEncoder;
using crf::sensors::cameras::CameraMock;
using crf::sensors::cameras::CameraClient;
using crf::communication::datapackets::FramePacket;
using crf::communication::sockets::SocketMock;
using crf::communication::datapacketsocket::PacketSocket;

class CameraClientShould : public ::testing::Test {
 protected:
    CameraClientShould() :
        logger_("CameraClientShould"),
        mock_(new NiceMock<CameraMock>),
        simulator_(new ICameraSimulator(mock_)),
        sut_(),
        socketMock_(new NiceMock<SocketMock>),
        socket_(new PacketSocket(socketMock_)),
        isSocketOpen_(false),
        readMutex_(),
        readCv_(),
        bytesToRead_(),
        writeMutex_(),
        writeCv_(),
        bytesWritten_(),
        writeTimeout_(std::chrono::seconds(2)) {
            logger_->info("{0} BEGIN",
                testing::UnitTest::GetInstance()->current_test_info()->name());
            simulator_->initializeDelay = std::chrono::milliseconds(1);
    }

    void SetUp() override {
        mock_->initialize();
        mock_->setResolution(cv::Size(640, 480));

        failToOpenSocket_ = false;
        isSocketOpen_ = false;
        ON_CALL(*socketMock_, isOpen()).WillByDefault(Invoke([this]() {
            return isSocketOpen_.load();
        }));

        ON_CALL(*socketMock_, open()).WillByDefault(Invoke([this]() {
            if (failToOpenSocket_) return false;
            if (isSocketOpen_) return false;
            isSocketOpen_ = true;
            return true;
        }));

        ON_CALL(*socketMock_, close()).WillByDefault(Invoke([this]() {
            std::unique_lock<std::mutex> lock(readMutex_);
            if (failToOpenSocket_) return false;
            if (!isSocketOpen_) return false;
            readCv_.notify_all();
            isSocketOpen_ = false;
            return true;
        }));

        ON_CALL(*socketMock_, read(_)).WillByDefault(Invoke([this](int length) {
            while (bytesToRead_.length() < static_cast<size_t>(length)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                if (!isSocketOpen_) {
                    return std::string();
                }
            }
            std::unique_lock<std::mutex> lock(readMutex_);
            std::string bytes = bytesToRead_.substr(0, length);
            bytesToRead_ = bytesToRead_.substr(length, bytesToRead_.length());
            return bytes;
        }));

        ON_CALL(*socketMock_, read(_, _)).WillByDefault(Invoke([this](int length, const std::chrono::milliseconds& timeout) { // NOLINT
            std::unique_lock<std::mutex> lock(readMutex_);
            if (readCv_.wait_for(lock, timeout) == std::cv_status::timeout) {
                return std::string();
            }

            std::string bytes = bytesToRead_.substr(0, length);
            bytesToRead_ = bytesToRead_.substr(length, bytesToRead_.length());
            return bytes;
        }));

        ON_CALL(*socketMock_, write(_, _)).WillByDefault(
            Invoke([this](std::string buffer, bool ack) {
                if (!isSocketOpen_) {
                    return false;
                }

                std::unique_lock<std::mutex> lock(writeMutex_);
                bytesWritten_.append(buffer);
                dataWritten_ = true;
                writeCv_.notify_all();
                return true;
        }));
    }

    ~CameraClientShould() {
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

    void sendGetParam() {
        auto pack = readJsonPacket();
        if (!pack) {
            logger_->info("Not received");
            return;
        }

        if (!isSocketOpen_) {
            return;
        }
        ASSERT_TRUE(pack);
        ASSERT_EQ(pack.get().data["cmd"], "getparam");
        nlohmann::json json;
        auto resolution = mock_->getResolution();
        if (resolution) {
            json["resolution"]["width"] = resolution.get().width;
            json["resolution"]["height"] = resolution.get().height;
        }

        auto fps = mock_->getFramerate();
        if (fps) {
            json["fps"] = fps.get();
        }

        auto zoom = mock_->getZoom();
        if (zoom) {
            json["zoom"] = zoom.get();
        }

        auto pan = mock_->getPan();
        auto tilt = mock_->getTilt();

        if (pan && tilt) {
            std::vector<float> position({
                pan.get(),
                tilt.get()
            });
            json["position"] = position;
        }

        auto exposure = mock_->getExposure();
        if (exposure) {
            json["exposure"] = exposure.get();
        }

        auto shutter = mock_->getShutterSpeed();
        if (shutter) {
            json["shutter"] = shutter.get();
        }

        auto focusmode = mock_->getFocusMode();
        if (focusmode) {
            json["focusmode"] = focusmode.get() ==
                crf::sensors::cameras::ICamera::FocusModes::Auto ? "auto" : "manual";
        }

        auto focus = mock_->getFocus();
        if (focus) {
            json["focus"] = focus.get();
        }

        auto iso = mock_->getISO();
        if (iso) {
            json["iso"] = iso.get();
        }

        auto availableResolutions = mock_->getAvailableResolutions();
        std::vector<std::pair <int, int> > jsonable;
        for (auto &res : availableResolutions) {
            jsonable.push_back({ res.width, res.height });
        }
        json["resolutions"] = jsonable;

        crf::communication::datapackets::JSONPacket packet;
        packet.data["cmd"] = "response";
        packet.data["response_cmd"] = "getparam";
        packet.data["params"] = json;
        writePacket(packet);
    }

    boost::optional<crf::communication::datapackets::JSONPacket> readJsonPacket() {
        {
            std::unique_lock<std::mutex> lock(writeMutex_);
            dataWritten_ = false;
            if (bytesWritten_.length() == 0) {
                if (!writeCv_.wait_for(lock, writeTimeout_, [this] {
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

    crf::utility::logger::EventLogger logger_;

    std::shared_ptr<CameraMock> mock_;
    std::unique_ptr<ICameraSimulator> simulator_;
    std::unique_ptr<CameraClient> sut_;

    std::shared_ptr<SocketMock> socketMock_;
    std::shared_ptr<PacketSocket> socket_;

    bool failToOpenSocket_;
    std::atomic<bool> isSocketOpen_;

    std::mutex readMutex_;
    std::condition_variable readCv_;
    std::string bytesToRead_;
    std::mutex writeMutex_;
    std::condition_variable writeCv_;
    bool dataWritten_;
    std::string bytesWritten_;
    std::chrono::milliseconds writeTimeout_;
};

TEST_F(CameraClientShould, failsToInitializeIfCantOpenSocket) {
    sut_.reset(new CameraClient(socket_, std::chrono::milliseconds(10)));
    failToOpenSocket_ = true;
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(CameraClientShould, failsToInitializeIfCantStartStream) {
    sut_.reset(new CameraClient(socket_, std::chrono::milliseconds(10)));

    auto retval = std::async(std::launch::async, [this]() {
        return sut_->initialize();
    });

    auto pack = readJsonPacket();

    ASSERT_TRUE(pack);
    ASSERT_EQ(pack.get().data["command"], "startFrameStream");

    ASSERT_FALSE(retval.get());
}

TEST_F(CameraClientShould, failsToInitializeIfCantGetCameraParameters) {
    sut_.reset(new CameraClient(socket_, std::chrono::milliseconds(10)));

    auto retval = std::async(std::launch::async, [this]() {
        return sut_->initialize();
    });

    auto pack = readJsonPacket();
    ASSERT_TRUE(pack);
    ASSERT_EQ(pack.get().data["command"], "startFrameStream");
    JPEGVideoEncoder encoder(crf::algorithms::videocodecs::CompressionQuality::Ultrafast);
    encoder.addFrame(mock_->getFrame());
    FramePacket frame(encoder.getBytes(),
        crf::communication::datapackets::FramePacket::Encoding::JPEG);
    writePacket(frame);

    ASSERT_FALSE(retval.get());
}

TEST_F(CameraClientShould, operationsFailsIfNotInitialized) {
    sut_.reset(new CameraClient(socket_, std::chrono::milliseconds(10)));

    ASSERT_FALSE(sut_->setZoom(0));
    ASSERT_FALSE(sut_->setPosition(0, 0));
    ASSERT_FALSE(sut_->setExposure(0));
    ASSERT_FALSE(sut_->setShutterSpeed(0));
    ASSERT_FALSE(sut_->setFocus(0));
    ASSERT_FALSE(sut_->setISO(0));
    ASSERT_FALSE(sut_->setFocusMode(
        crf::sensors::cameras::ICamera::FocusModes::Auto));

    ASSERT_FALSE(sut_->getResolution());
    ASSERT_FALSE(sut_->getFramerate());
    ASSERT_FALSE(sut_->getZoom());
    ASSERT_FALSE(sut_->getPan());
    ASSERT_FALSE(sut_->getTilt());
    ASSERT_FALSE(sut_->getExposure());
    ASSERT_FALSE(sut_->getShutterSpeed());
    ASSERT_FALSE(sut_->getFocusMode());
    ASSERT_FALSE(sut_->getFocus());

    ASSERT_TRUE(sut_->getFrame().empty());

    ASSERT_EQ(sut_->getAvailableResolutions().size(), 0);
}

// Test disabled due to a constant file. Needs to be checked
TEST_F(CameraClientShould, DISABLED_correctlySetAndGetCameraParameters) {
    sut_.reset(new CameraClient(socket_, std::chrono::milliseconds(100)));
    writeTimeout_ = std::chrono::milliseconds(1000);

    JPEGVideoEncoder encoder(crf::algorithms::videocodecs::CompressionQuality::Ultrafast);
    encoder.addFrame(mock_->getFrame());
    FramePacket frame(encoder.getBytes(),
        crf::communication::datapackets::FramePacket::Encoding::JPEG);
    writePacket(frame);

    auto retval = std::async(std::launch::async, [this]() {
        return sut_->initialize();
    });

    auto pack = readJsonPacket();
    ASSERT_TRUE(pack);
    ASSERT_EQ(pack.get().data["command"], "startFrameStream");
    sendGetParam();

    ASSERT_TRUE(retval.get());
    ASSERT_FALSE(sut_->initialize());

    ASSERT_FALSE(sut_->setFramerate(0));
    ASSERT_FALSE(sut_->getZoom());
    ASSERT_FALSE(sut_->getPan());
    ASSERT_FALSE(sut_->getTilt());
    ASSERT_FALSE(sut_->getExposure());
    ASSERT_FALSE(sut_->getShutterSpeed());
    ASSERT_FALSE(sut_->getFocusMode());
    ASSERT_FALSE(sut_->getFocus());
    ASSERT_FALSE(sut_->getISO());

    bool stop = false;
    retval = std::async(std::launch::async, [this, &stop]() {
        while (!stop) {
            sendGetParam();
        }
        return true;
    });
    ASSERT_TRUE(sut_->getZoom());
    ASSERT_EQ(sut_->getZoom().get(), 0);

    ASSERT_TRUE(sut_->getPan());
    ASSERT_EQ(sut_->getPan().get(), 0);

    ASSERT_TRUE(sut_->getTilt());
    ASSERT_EQ(sut_->getTilt().get(), 0);

    ASSERT_TRUE(sut_->getExposure());
    ASSERT_EQ(sut_->getExposure().get(), 0);

    ASSERT_TRUE(sut_->getShutterSpeed());
    ASSERT_EQ(sut_->getShutterSpeed().get(), 0);

    ASSERT_TRUE(sut_->getFocusMode());
    ASSERT_EQ(sut_->getFocusMode().get(), crf::sensors::cameras::ICamera::FocusModes::Auto);

    ASSERT_TRUE(sut_->getFocus());
    ASSERT_EQ(sut_->getFocus().get(), 0);

    ASSERT_TRUE(sut_->getISO());
    ASSERT_EQ(sut_->getISO().get(), 1200);

    auto resolutions = sut_->getAvailableResolutions();
    ASSERT_EQ(resolutions.size(), 6);

    stop = true;
    ASSERT_TRUE(retval.get());

    ASSERT_TRUE(sut_->setZoom(25));
    do {
        pack = readJsonPacket();
    } while (pack.get().data["command"] != "setStatus");
    ASSERT_TRUE(pack);
    ASSERT_EQ(pack.get().data["zoom"], 25);

    ASSERT_TRUE(sut_->setPosition(10, 10));
    pack = readJsonPacket();
    ASSERT_TRUE(pack);
    ASSERT_EQ(pack.get().data["position"][0], 10);
    ASSERT_EQ(pack.get().data["position"][1], 10);

    ASSERT_TRUE(sut_->setExposure(5));
    pack = readJsonPacket();
    ASSERT_TRUE(pack);
    ASSERT_EQ(pack.get().data["exposure"], 5);

    ASSERT_TRUE(sut_->setShutterSpeed(8));
    pack = readJsonPacket();
    ASSERT_TRUE(pack);
    ASSERT_EQ(pack.get().data["shutter"], 8);

    ASSERT_TRUE(sut_->setFocusMode(crf::sensors::cameras::ICamera::FocusModes::Manual));
    pack = readJsonPacket();
    ASSERT_TRUE(pack);
    ASSERT_EQ(pack.get().data["focusmode"], "manual");

    ASSERT_TRUE(sut_->setFocus(10));
    pack = readJsonPacket();
    ASSERT_TRUE(pack);
    ASSERT_EQ(pack.get().data["focus"], 10);

    ASSERT_TRUE(sut_->setISO(100));
    pack = readJsonPacket();
    ASSERT_TRUE(pack);
    ASSERT_EQ(pack.get().data["iso"], 100);
    ASSERT_TRUE(sut_->deinitialize());
}
