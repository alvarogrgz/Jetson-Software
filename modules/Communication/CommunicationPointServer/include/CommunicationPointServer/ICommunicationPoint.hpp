/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>

#include "CommonInterfaces/IInitializable.hpp"

namespace crf {
namespace communication {
namespace communicationpointserver {

class ICommunicationPoint : utility::commoninterfaces::IInitializable {
 public:
    virtual ~ICommunicationPoint() = default;

    bool initialize() override = 0;
    bool deinitialize() override = 0;
};

}  // namespace communicationpointserver
}  // namespace communication
}  // namespace crf
