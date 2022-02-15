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

#include "DeviceManager/DeviceManagerCommunicationPoint/StatusStreamerCommunicationPoint.hpp"
#include "Cameras/CameraCommunicationPoint/CameraManager.hpp"
#include "DataPackets/FramePacket/FramePacket.hpp"
#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "EventLogger/EventLogger.hpp"
#include "VideoCodecs/IVideoEncoder.hpp"

namespace crf {
namespace sensors {
namespace cameras {

class CameraCommunicationPoint :
    public utility::devicemanager::StatusStreamerCommunicationPoint {
 public:
    CameraCommunicationPoint() = delete;
    CameraCommunicationPoint(std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
        std::shared_ptr<CameraManager> manager);
    ~CameraCommunicationPoint() override;
    bool deinitialize() override;


 protected:
    static std::vector<bool> availableStreams_;
    int StreamID_;
    int requestedStreamFramerate_;
    std::atomic<bool> frameStreamActive_, stopFrameStream_;
    std::thread frameStreamerThread_;
    std::mutex encoderMutex_;
    std::shared_ptr<algorithms::videocodecs::IVideoEncoder> encoder_;
    communication::datapackets::FramePacket::Encoding encodingType_;
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket_;
    
    virtual void startFrameStreamRequestHandler(const communication::datapackets::JSONPacket&);
    virtual void frameStreamer(int streamId);

 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<CameraManager> manager_;

    void setStatusRequestHandler(const communication::datapackets::JSONPacket&);
    void getParametersRequestHandler(const communication::datapackets::JSONPacket&);
    void getFrameRequestHandler(const communication::datapackets::JSONPacket&);
    void stopFrameStreamRequestHandler(const communication::datapackets::JSONPacket&);
};

}  // namespace cameras
}  // namespace sensors
}  // namespace crf
