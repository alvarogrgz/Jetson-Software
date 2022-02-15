/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *         Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <limits>

#include "ComponentAccessControl/SimpleAccessControl/SimpleAccessControl.hpp"

namespace crf {
namespace communication {
namespace componentaccesscontrol {

SimpleAccessControl::SimpleAccessControl(const std::chrono::milliseconds expireTime) :
    logger_("SimpleAccessControl"),
    accessMutex_(),
    requestedAccesses_(),
    accessExpireTime_(expireTime) {
    logger_->debug("CTor");
}

bool SimpleAccessControl::requestAccess(uint32_t priority) {
    logger_->debug("requestAccess");
    if (priority == 0) {
        logger_->error("The priority can not be 0");
        return false;
    }
    std::lock_guard<std::mutex> lock(accessMutex_);
    if (accessExpireTime_.count() != 0) {
        expiredCleanup();
    }
    if (!containsPriority(priority)) {
        Access acc = {priority, std::chrono::high_resolution_clock::now()};
        requestedAccesses_.push_back(acc);
    } else {
        for (auto it = requestedAccesses_.begin(); it != requestedAccesses_.end(); it++) {
            if (it->priority_ == priority) {
                it->lastAccessTime_ = std::chrono::high_resolution_clock::now();
            }
        }
    }
    return (getHighestPriority() == priority);
}

bool SimpleAccessControl::releaseAccess(uint32_t priority) {
    logger_->debug("releaseAccess");
    if (priority == 0) {
        logger_->error("The priority can not be 0");
        return false;
    }
    std::lock_guard<std::mutex> lock(accessMutex_);
    if (accessExpireTime_.count() != 0) {
        expiredCleanup();
    }
    if (!containsPriority(priority)) {
        return false;
    }
    for (auto it = requestedAccesses_.begin(); it != requestedAccesses_.end(); it++) {
        if (it->priority_ == priority) {
            requestedAccesses_.erase(it);
            break;
        }
    }
    return true;
}

uint32_t SimpleAccessControl::getHighestPriority() {
    logger_->debug("getHighestPriority");
    if (accessExpireTime_.count() != 0) {
        expiredCleanup();
    }
    if (requestedAccesses_.size() == 0) {
        logger_->warn("No accesses requested");
        return 0;
    }
    uint32_t max = std::numeric_limits<uint32_t>::max();
    for (auto it = requestedAccesses_.begin(); it != requestedAccesses_.end(); it++) {
        max = it->priority_ < max ? it->priority_ : max;
    }
    return max;
}

void SimpleAccessControl::expiredCleanup() {
    logger_->debug("expiredCleanup");
    auto now = std::chrono::high_resolution_clock::now();
    for (auto it = requestedAccesses_.begin(); it != requestedAccesses_.end(); it++) {
        if ((now - it->lastAccessTime_) > accessExpireTime_) {
            requestedAccesses_.erase(it--);
        }
    }
}

bool SimpleAccessControl::containsPriority(uint32_t priority) {
    logger_->debug("containsPriority");
    for (auto it = requestedAccesses_.begin(); it != requestedAccesses_.end(); it++) {
        if (it->priority_ == priority) return true;
    }
    return false;
}

}  // namespace componentaccesscontrol
}  // namespace communication
}  // namespace crf
