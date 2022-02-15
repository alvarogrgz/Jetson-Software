/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <nlohmann/json.hpp>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "Cameras/ICamera.hpp"
#include "DataPackets/FramePacket/FramePacket.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"
#include "VideoCodecs/IVideoEncoder.hpp"
#include "VideoCodecs/IVideoDecoder.hpp"

namespace crf {
namespace sensors {
namespace cameras {

class CameraClient : public ICamera {
 public:
    CameraClient() = delete;
    CameraClient(std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
        const std::chrono::milliseconds& serverResponseTimeout = std::chrono::milliseconds(1000));
    ~CameraClient() override;

    bool initialize() override;
    bool deinitialize() override;

    bool setResolution(const cv::Size&) override;
    bool setFramerate(int fps) override;
    bool setZoom(float zoom) override;
    bool setPosition(float pan, float tilt) override;
    bool setExposure(float exposure) override;
    bool setShutterSpeed(float) override;
    bool setFocusMode(FocusModes) override;
    bool setFocus(float) override;
    bool setISO(int) override;

    boost::optional<cv::Size> getResolution() override;
    boost::optional<int> getFramerate() override;
    boost::optional<float> getZoom() override;
    boost::optional<float> getPan() override;
    boost::optional<float> getTilt() override;
    boost::optional<float> getExposure() override;
    boost::optional<float> getShutterSpeed() override;
    boost::optional<FocusModes> getFocusMode() override;
    boost::optional<float> getFocus() override;
    boost::optional<int> getISO() override;
    cv::Mat getFrame() override;

    std::vector<cv::Size> getAvailableResolutions() override;
    std::vector<int> getAvailableFramerates(const cv::Size& resolution) override;

    bool setDeviceSpecificParameters(const nlohmann::json& configData) override;

    bool setEncoding(communication::datapackets::FramePacket::Encoding encoding,
        algorithms::videocodecs::CompressionQuality quality);

 private:
    utility::logger::EventLogger logger_;

    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket_;
    std::shared_ptr<algorithms::videocodecs::IVideoDecoder> activeDecoder_;

    std::chrono::milliseconds serverResponseTimeout_;

    cv::Size requestedResolution_;
    int requestedFramerate_;
    communication::datapackets::FramePacket::Encoding encoding_;
    algorithms::videocodecs::CompressionQuality quality_;

    std::mutex jsonMutex_;
    std::condition_variable jsonCv_;
    nlohmann::json parameters_;

    bool initialized_;

    std::atomic<bool> stopThread_;
    std::mutex frameMutex_;
    std::condition_variable frameCv_;
    cv::Mat latestFrame_;
    std::thread grabberThread_;
    void grabber();

    bool reqGetParameters();
    bool startFrameStream();
    bool stopFrameStream();
};

}  // namespace cameras
}  // namespace sensors
}  // namespace crf
