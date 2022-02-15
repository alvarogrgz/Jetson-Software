/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *         Jorge Playán Garai CERN BE/CEM/MRO 2021
 *  ==================================================================================================
 */

#include <memory>
#include <thread>
#include <chrono>

#include <nlohmann/json.hpp>

#include "DeviceManager/DeviceManagerWithPriorityAccess/DeviceManagerWithPriorityAccess.hpp"
#include "CommonInterfaces/IInitializable.hpp"
#include "EventLogger/EventLogger.hpp"
#include "ComponentAccessControl/SimpleAccessControl/SimpleAccessControl.hpp"

namespace crf {
namespace utility {
namespace devicemanager {

DeviceManagerWithPriorityAccess::DeviceManagerWithPriorityAccess(
    const std::shared_ptr<crf::utility::commoninterfaces::IInitializable>& device,
    const std::chrono::milliseconds& initializationTimeout,
    const std::chrono::milliseconds& controlAccessTimeout) :
    DeviceManagerWithAutoInitialization(device, initializationTimeout),
    simpleAccessControl_(controlAccessTimeout) {
    logger_ = crf::utility::logger::EventLogger("DeviceManagerWithPriorityAccess");
    if (controlAccessTimeout == std::chrono::milliseconds(0)) {
        throw std::runtime_error("The control access timeout is not valid");
    }
}

DeviceManagerWithPriorityAccess::~DeviceManagerWithPriorityAccess() {
}

nlohmann::json DeviceManagerWithPriorityAccess::getStatus() {
    logger_->debug("getStatus");
    std::scoped_lock<std::mutex> lock(accessMutex_);
    nlohmann::json statusJSON;
    lastRequestTime_ = std::chrono::high_resolution_clock::now();

    if (initializeDevice()) {
        statusJSON["status"] = "initialize";
    } else {
        statusJSON["status"] = "deinitialize";
    }

    statusJSON["priorityUnderControl"] = simpleAccessControl_.getHighestPriority();

    return statusJSON;
}

bool DeviceManagerWithPriorityAccess::lockControl(const uint32_t &priority) {
    logger_->debug("lockControl");
    std::scoped_lock<std::mutex> lock(accessMutex_);
    if (!simpleAccessControl_.requestAccess(priority)) {
        logger_->warn("A higher priority communication point holds the access");
        return false;
    }
    lastRequestTime_ = std::chrono::high_resolution_clock::now();
    if (!initializeDevice()) {
        return false;
    }
    return true;
}

bool DeviceManagerWithPriorityAccess::unlockControl(const uint32_t &priority) {
    logger_->debug("unlockControl");
    std::scoped_lock<std::mutex> lock(accessMutex_);
    if (!simpleAccessControl_.releaseAccess(priority)) {
        logger_->warn("Failed to remove the priority number from the list");
        return false;
    }
    return true;
}

bool DeviceManagerWithPriorityAccess::checkCommandPriority(const uint32_t &priority) {
    if (simpleAccessControl_.getHighestPriority() < priority &&
        simpleAccessControl_.getHighestPriority() != 0) {
        logger_->warn("You don't have the control of the TIM");
        return false;
    }
    if (!simpleAccessControl_.requestAccess(priority)) {
        logger_->warn("Failed to refresh priority access time");
        return false;
    }
    lastRequestTime_ = std::chrono::high_resolution_clock::now();
    return true;
}

}  // namespace devicemanager
}  // namespace utility
}  // namespace crf
