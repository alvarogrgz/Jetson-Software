/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <thread>
#include <chrono>

#include <nlohmann/json.hpp>

#include "DeviceManager/DeviceManagerWithAutoInitialization/DeviceManagerWithAutoInitialization.hpp"
#include "CommonInterfaces/IInitializable.hpp"
#include "EventLogger/EventLogger.hpp"
#include "ComponentAccessControl/SimpleAccessControl/SimpleAccessControl.hpp"

namespace crf {
namespace utility {
namespace devicemanager {

/*
 * @brief This thread safe device manager automatically deinitialized the device if no one has
 *        requested any information in a time equal to the initialization timeout. It also allows
 *        to know if with a given priority you are the highest one requesting access.
 */
class DeviceManagerWithPriorityAccess : public DeviceManagerWithAutoInitialization {
 public:
    DeviceManagerWithPriorityAccess(
        const std::shared_ptr<crf::utility::commoninterfaces::IInitializable>& device,
        const std::chrono::milliseconds& initializationTimeout = std::chrono::seconds(60),
        const std::chrono::milliseconds& controlAccessTimeout = std::chrono::seconds(15));
    DeviceManagerWithPriorityAccess(const DeviceManagerWithPriorityAccess& other) = delete;
    DeviceManagerWithPriorityAccess(DeviceManagerWithPriorityAccess&& other) = delete;
    DeviceManagerWithPriorityAccess() = delete;
    ~DeviceManagerWithPriorityAccess();

    nlohmann::json getStatus() override;

    /*
     * @brief Requests the control of the device given the priority. If a higher priority is
     *        given, the control over the rest will be revoked.
     * @param priority the priority of the system from 1 to 2^32, being the 1 the number with
     *        highest priority.
     * @return A shared pointer of the device if the access was granted.
     * @return nullptr if there is someone else with a higher priority and you cant control the
     *         device.
     */
    bool lockControl(const uint32_t &priority);
    /*
     * @brief Notifies that the control done by the client with the given priority stops
     *        controlling the arm.
     * @param priority the priority of the system from 1 to 2^32, being the 1 the number with
     *        highest priority.
     * @return True if the unlock of the device control was successful.
     * @return False if the unlock of the device control failed.
     */
    bool unlockControl(const uint32_t &priority);

 protected:
    crf::communication::componentaccesscontrol::SimpleAccessControl simpleAccessControl_;

    /*
     * @brief Checks if the given priority is the highest taking into account the previous requests
     *        and the control access timeout.
     * @return True if it is the highest.
     */
    bool checkCommandPriority(const uint32_t &priority);
};

}  // namespace devicemanager
}  // namespace utility
}  // namespace crf
