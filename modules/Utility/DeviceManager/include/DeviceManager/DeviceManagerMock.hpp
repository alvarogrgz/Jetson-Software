/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include "DeviceManager/IDeviceManager.hpp"

namespace crf {
namespace utility {
namespace devicemanager {

class DeviceManagerMock : public IDeviceManager {
 public:
    MOCK_METHOD0(getStatus, nlohmann::json());
};

}  // namespace devicemanager
}  // namespace utility
}  // namespace crf
