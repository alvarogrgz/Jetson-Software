/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <map>
#include <string>
#include <functional>
#include <atomic>
#include <thread>
#include <set>
#include <condition_variable>
#include <mutex>

#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "DeviceManager/IDeviceManager.hpp"
#include "EventLogger/EventLogger.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"

namespace crf {
namespace utility {
namespace devicemanager {

/*
 * @brief Communication Point that can send once or stream given a desired frequency, the status of
 *        a device.
 */
class StatusStreamerCommunicationPoint :
    public communication::communicationpointserver::ICommunicationPoint {
 public:
    StatusStreamerCommunicationPoint() = delete;
    StatusStreamerCommunicationPoint(
        std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
        std::shared_ptr<crf::utility::devicemanager::IDeviceManager> manager);
    ~StatusStreamerCommunicationPoint() override;

    bool initialize() override;
    bool deinitialize() override;

 protected:
    crf::utility::logger::EventLogger logger_;
    std::map<uint16_t, std::function<void(const std::string&)>> packetHandlers_;
    std::map<std::string,
        std::function<void(const communication::datapackets::JSONPacket&)>> jsonCommandHandlers_;
    std::atomic<bool> streamActive_;
    std::atomic<bool> stopStream_;
    std::atomic<bool> stopThreads_;
    bool initialized_;


    template<typename T>
    void sendJSONPacket(const std::string& replyCommand, T message, bool requiresAck) {
        communication::datapackets::JSONPacket responseJSON;
        responseJSON.data["command"] = "reply";
        responseJSON.data["replyCommand"] = replyCommand;
        responseJSON.data["message"] = message;

        if (replyCommand == "error") {
            logger_->warn(message);
        }
        socket_->write(responseJSON, responseJSON.getHeader(), requiresAck);
    }

 private:
    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket_;
    std::shared_ptr<crf::utility::devicemanager::IDeviceManager> manager_;
    std::thread receiverThread_;
    float frequency_;
    std::thread statusStreamerThread_;

    std::mutex frequencyMtx_;
    std::condition_variable frequencyCV_;

    void getStatusRequestHandler(const communication::datapackets::JSONPacket& packet);
    void startStreamStatusRequestHandler(const communication::datapackets::JSONPacket& packet);
    void stopStreamStatusRequestHandler(const communication::datapackets::JSONPacket& packet);

    void receiver();
    void statusStreamer();

    void parseJSONPacket(const std::string& buffer);
};

}  // namespace devicemanager
}  // namespace utility
}  // namespace crf
