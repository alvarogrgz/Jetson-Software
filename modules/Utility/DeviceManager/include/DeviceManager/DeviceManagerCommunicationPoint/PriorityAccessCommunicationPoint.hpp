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

#include "DeviceManager/DeviceManagerCommunicationPoint/StatusStreamerCommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "DeviceManager/DeviceManagerWithPriorityAccess/DeviceManagerWithPriorityAccess.hpp"
#include "EventLogger/EventLogger.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"

namespace crf {
namespace utility {
namespace devicemanager {

/*
 * @brief Communication Point that can send once or stream given a desired frequency, the status of
 *        a device. It can also give the control of the device according the priority.
 */
class PriorityAccessCommunicationPoint : public StatusStreamerCommunicationPoint {
 public:
    PriorityAccessCommunicationPoint() = delete;
    PriorityAccessCommunicationPoint(
        std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
        std::shared_ptr<crf::utility::devicemanager::DeviceManagerWithPriorityAccess> manager);
    ~PriorityAccessCommunicationPoint() override;

 private:
    std::shared_ptr<DeviceManagerWithPriorityAccess> manager_;

    void lockControlRequestHandler(const communication::datapackets::JSONPacket& packet);
    void unlockControlRequestHandler(const communication::datapackets::JSONPacket& packet);
};

}  // namespace devicemanager
}  // namespace utility
}  // namespace crf
