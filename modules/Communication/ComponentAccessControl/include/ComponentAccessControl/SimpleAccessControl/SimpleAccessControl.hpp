/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *         Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <chrono>
#include <mutex>
#include <vector>

#include "ComponentAccessControl/IComponentAccessControl.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace communication {
namespace componentaccesscontrol {

/*
 * @brief Manages the access depending on the priority number, that can go from 1 to 2^32, being
 * the 1 the number with highest priority. If a priority number does not request the access for a
 * specific period of time its access is revoqued.
 */
class SimpleAccessControl : public IComponentAccessControl {
 public:
    explicit SimpleAccessControl(
        const std::chrono::milliseconds expireTime = std::chrono::milliseconds(0));

    bool requestAccess(uint32_t priority) override;
    bool releaseAccess(uint32_t priority) override;
    uint32_t getHighestPriority() override;

 private:
    typedef struct  {
        uint32_t priority_;
        std::chrono::time_point<std::chrono::high_resolution_clock> lastAccessTime_;
    } Access;

    crf::utility::logger::EventLogger logger_;
    std::mutex accessMutex_;
    std::vector<Access> requestedAccesses_;
    std::chrono::milliseconds accessExpireTime_;

    /*
     * @brief Revoques the access of a priority number is this has not accessed for a determined
     * period of time, deleting it from the list.
     */
    void expiredCleanup();
    /* 
     * @return True if the priority number is in the list
     * @return False if the priority number is ot in the list
     */
    bool containsPriority(uint32_t priority);
};

}  // namespace componentaccesscontrol
}  // namespace communication
}  // namespace crf
