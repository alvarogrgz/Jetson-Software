/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>

#include "Cameras/CameraCommunicationPoint/CameraCommunicationPoint.hpp"
#include "Cameras/CameraCommunicationPoint/CameraCommunicationPointFactory.hpp"

namespace crf {
namespace sensors {
namespace cameras {

CameraCommunicationPointFactory::CameraCommunicationPointFactory(
    std::shared_ptr<CameraManager> manager) :
        logger_("CameraCommunicationPointFactory"),
        manager_(manager) {
            logger_->debug("CTor");
}

boost::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
    CameraCommunicationPointFactory::create(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket) {
        std::shared_ptr<communication::communicationpointserver::ICommunicationPoint> commpoint =
            std::make_shared<CameraCommunicationPoint>(socket, manager_);
        return commpoint;
    }

}   // namespace cameras
}   // namespace sensors
}   // namespace crf
