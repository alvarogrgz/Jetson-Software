/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "Cameras/CameraCommunicationPoint/CameraCommunicationPoint.hpp"
#include "RGBDCameras/RGBDCameraCommunicationPoint/RGBDCameraManager.hpp"
#include "DataPackets/RGBDFramePacket/RGBDFramePacket.hpp"
#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "EventLogger/EventLogger.hpp"
#include "VideoCodecs/IVideoEncoder.hpp"

namespace crf {
namespace sensors {
namespace rgbdcameras {

class RGBDCameraCommunicationPoint : public cameras::CameraCommunicationPoint {
 public:
    RGBDCameraCommunicationPoint() = delete;
    RGBDCameraCommunicationPoint(
        std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
        std::shared_ptr<RGBDCameraManager> manager);
    ~RGBDCameraCommunicationPoint() override;

 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<RGBDCameraManager> manager_;

    communication::datapackets::RGBDFramePacket::RGBEncoding currentRgbEncoding_;
    communication::datapackets::RGBDFramePacket::DepthEncoding currentDepthEncoding_;
    communication::datapackets::RGBDFramePacket::PointCloudEncoding currentPointCloudEncoding_;

    bool rgbRequested_;
    bool depthRequested_;
    bool pointCloudRequested_;
    bool alignFramesRequest_;

    cv::Mat extrinsics_;
    cv::Mat depthIntrinsics_;
    cv::Mat depthDistortion_;
    cv::Mat colorIntrinsics_;
    cv::Mat colorDistortion_;
    float subsampling_;

    void frameStreamer(int streamId) override;

    void startFrameStreamRequestHandler(const communication::datapackets::JSONPacket&) override;

    std::string compressLZ4(const std::string& bytes);
    void updateCameraParameters();
};

}  // namespace rgbdcameras
}  // namespace sensors
}  // namespace crf
