/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>

#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"

namespace crf {
namespace communication {
namespace communicationpointserver {

class ICommunicationPointFactory {
 public:
    virtual ~ICommunicationPointFactory() = default;

    virtual boost::optional<std::shared_ptr<ICommunicationPoint>> create(
        std::shared_ptr<datapacketsocket::PacketSocket>) = 0;
};

}  // namespace communicationpointserver
}  // namespace communication
}  // namespace crf
