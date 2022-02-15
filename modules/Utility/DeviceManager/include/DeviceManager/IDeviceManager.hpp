/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *  ==================================================================================================
 */

#pragma once

#include <nlohmann/json.hpp>

namespace crf {
namespace utility {
namespace devicemanager {

class IDeviceManager {
 public:
    virtual ~IDeviceManager() = default;

    /*
     * @brief For each device you can get all the parameters and current status of the device. This
     *        method is thread safe.
     * @return Parameters in json format.
     */
    virtual nlohmann::json getStatus() = 0;
};

}  // namespace devicemanager
}  // namespace utility
}  // namespace crf
