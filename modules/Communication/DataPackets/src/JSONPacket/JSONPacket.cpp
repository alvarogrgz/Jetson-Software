/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO 2018
 *
 *  ==================================================================================================
 */

#include <exception>
#include <string>

#include "DataPackets/JSONPacket/JSONPacket.hpp"

namespace crf {
namespace communication {
namespace datapackets {

std::string JSONPacket::serialize() const {
    return data.dump();
}

bool JSONPacket::deserialize(const std::string& buffer) {
    try {
        data = nlohmann::json::parse(buffer);
    } catch (const std::exception&) {
        data.clear();
        return false;
    }
    return true;
}

PacketHeader JSONPacket::getHeader() const {
    PacketHeader header{};
    header.type(JSON_PACKET_TYPE);
    header.length(data.dump().size());
    return header;
}

std::string JSONPacket::toJSONString() const {
    return data.dump();
}

}  // namespace datapackets
}  // namespace communication
}  // namespace crf
