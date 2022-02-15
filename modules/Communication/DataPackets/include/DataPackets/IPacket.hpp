/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MNRO 2017
 *
 *  ==================================================================================================
 */

#pragma once

#include <cstring>
#include <memory>
#include <string>

#include "DataPackets/PacketHeader.hpp"

namespace crf {
namespace communication {
namespace datapackets {

class IPacket {
 public:
    virtual ~IPacket() = default;

    /*
     * @brief
     * @return
     */
    virtual std::string serialize() const = 0;
    /*
     * @brief
     * @param
     * @return
     */
    virtual bool deserialize(const std::string& buffer) = 0;
    /*
     * @brief
     * @return
     */
    virtual PacketHeader getHeader() const = 0;
    /*
     * @brief
     * @return
     */
    virtual std::string toJSONString() const = 0;
};

}  // namespace datapackets
}  // namespace communication
}  // namespace crf
