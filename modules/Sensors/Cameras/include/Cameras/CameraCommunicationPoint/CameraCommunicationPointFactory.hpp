/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>

#include "Cameras/CameraCommunicationPoint/CameraManager.hpp"
#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "CommunicationPointServer/ICommunicationPointFactory.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace cameras {

class CameraCommunicationPointFactory :
    public communication::communicationpointserver::ICommunicationPointFactory {
 public:
    CameraCommunicationPointFactory() = delete;
    explicit CameraCommunicationPointFactory(std::shared_ptr<CameraManager> manager);
    CameraCommunicationPointFactory(const CameraCommunicationPointFactory&) = default;
    CameraCommunicationPointFactory(CameraCommunicationPointFactory&&) = default;
    ~CameraCommunicationPointFactory() override = default;

    boost::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
        create(std::shared_ptr<communication::datapacketsocket::PacketSocket>) override;

 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<CameraManager> manager_;
};

}  // namespace cameras
}  // namespace sensors
}  // namespace crf
