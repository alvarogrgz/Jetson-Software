/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO 2018
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <nlohmann/json.hpp>

#include "DataPackets/IPacket.hpp"
#include "DataPackets/PacketTypes.hpp"

namespace crf {
namespace communication {
namespace datapackets {

class JSONPacket : public IPacket {
 public:
    ~JSONPacket() override = default;

    std::string serialize() const override;
    bool deserialize(const std::string& buffer) override;
    PacketHeader getHeader() const override;
    std::string toJSONString() const override;

    nlohmann::json data;
};

}  // namespace datapackets
}  // namespace communication
}  // namespace crf
