/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#include <memory>

#include "RGBDCameras/RGBDCameraCommunicationPoint/RGBDCameraCommunicationPoint.hpp"
#include "RGBDCameras/RGBDCameraCommunicationPoint/RGBDCameraCommunicationPointFactory.hpp"

namespace crf {
namespace sensors {
namespace rgbdcameras {

RGBDCameraCommunicationPointFactory::RGBDCameraCommunicationPointFactory(
    std::shared_ptr<RGBDCameraManager> manager) :
        logger_("RGBDCameraCommunicationPointFactory"),
        manager_(manager) {
            logger_->debug("CTor");
}

boost::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint> >
    RGBDCameraCommunicationPointFactory::create(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket) {
        std::shared_ptr<communication::communicationpointserver::ICommunicationPoint> commpoint =
            std::make_shared<RGBDCameraCommunicationPoint>(socket, manager_);
        return commpoint;
    }

}  // namespace rgbdcameras
}  // namespace sensors
}  // namespace crf
