/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <stdint.h>

namespace crf {
namespace communication {
namespace componentaccesscontrol {

class IComponentAccessControl {
 public:
    virtual ~IComponentAccessControl() = default;
    /* 
     * @return True if the given priority number has access.
     * @return False if the given priority number doesn't have access.
     */
    virtual bool requestAccess(uint32_t priority) = 0;
    /* 
     * @return True if the given priority number was correctly deleted from the list of number with
     *         access.
     * @return False if the given priority number wasn't correctly deleted from the list of number
     *         with access.
     */
    virtual bool releaseAccess(uint32_t priority) = 0;
    /*
     * @brief Returns the number in the list with the maximum priority.
     * @return the highest priority, if no one has request it it will return 0.
     */
    virtual uint32_t getHighestPriority() = 0;
};

}  // namespace componentaccesscontrol
}  // namespace communication
}  // namespace crf
