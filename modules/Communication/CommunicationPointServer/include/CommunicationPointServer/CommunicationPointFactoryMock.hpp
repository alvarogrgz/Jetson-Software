/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <boost/optional.hpp>
#include <gmock/gmock.h>
#include <memory>

#include "CommunicationPointServer/ICommunicationPointFactory.hpp"

namespace crf {
namespace communication {
namespace communicationpointserver {

class CommunicationPointFactoryMock : public ICommunicationPointFactory {
 public:
    MOCK_METHOD1(create, boost::optional<std::shared_ptr<ICommunicationPoint>>(
        std::shared_ptr<datapacketsocket::PacketSocket>));
};

}  // namespace communicationpointserver
}  // namespace communication
}  // namespace crf
