/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#include <string>

#include "DataPackets/FramePacket/FramePacket.hpp"

namespace crf {
namespace communication {
namespace datapackets {

FramePacket::FramePacket() :
    encoding_(Encoding::JPEG),
    bytes_() {
}

FramePacket::FramePacket(const std::string& bytes, Encoding encoding)  :
    encoding_(encoding),
    bytes_(bytes) {
}

std::string FramePacket::serialize() const {
    std::string buffer;
    auto enc = static_cast<uint8_t>(encoding_);
    buffer.append(std::string(reinterpret_cast<const char*>(&enc), 1));
    buffer.append(bytes_);
    return buffer;
}

bool FramePacket::deserialize(const std::string& buffer) {
    if (buffer.length() == 0) {
        return false;
    }
    encoding_ = static_cast<Encoding>(buffer[0]);
    if (buffer.length() > 1)
        bytes_ = buffer.substr(1, buffer.length() - 1);
    return true;
}

PacketHeader FramePacket::getHeader() const {
    PacketHeader header;
    header.type(FRAME_PACKET);
    header.length(bytes_.length() + 1);
    return header;
}

std::string FramePacket::toJSONString() const {
    return std::string();
}

std::string FramePacket::getBytes() const {
    return bytes_;
}

FramePacket::Encoding FramePacket::getEncodingType() const {
    return encoding_;
}

}  // namespace datapackets
}  // namespace communication
}  // namespace crf
