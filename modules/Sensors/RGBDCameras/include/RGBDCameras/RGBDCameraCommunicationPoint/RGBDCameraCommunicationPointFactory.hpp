/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>

#include "RGBDCameras/RGBDCameraCommunicationPoint/RGBDCameraManager.hpp"
#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "CommunicationPointServer/ICommunicationPointFactory.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace rgbdcameras {

class RGBDCameraCommunicationPointFactory :
    public communication::communicationpointserver::ICommunicationPointFactory {
 public:
    RGBDCameraCommunicationPointFactory() = delete;
    explicit RGBDCameraCommunicationPointFactory(std::shared_ptr<RGBDCameraManager> manager);
    RGBDCameraCommunicationPointFactory(const RGBDCameraCommunicationPointFactory&) = default;
    RGBDCameraCommunicationPointFactory(RGBDCameraCommunicationPointFactory&&) = default;
    ~RGBDCameraCommunicationPointFactory() override = default;

    boost::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
        create(std::shared_ptr<communication::datapacketsocket::PacketSocket>) override;

 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<RGBDCameraManager> manager_;
};

}  // namespace rgbdcameras
}  // namespace sensors
}  // namespace crf
