/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Camarero Vera CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <string>

#include "DataPackets/StreamPacket/StreamPacket.hpp"
#include "CommunicationUtility/StreamWriter.hpp"
#include "CommunicationUtility/StreamReader.hpp"

namespace crf {
namespace communication {
namespace datapackets {

std::string StreamPacket::serialize() const {
    crf::utility::communicationutility::StreamWriter writer;
    writer.write(format_);
    writer.write(quality_);
    writer.write(data_);
    return writer.toString();
}

bool StreamPacket::deserialize(const std::string& buffer) {
    crf::utility::communicationutility::StreamReader reader(buffer);
    reader.read(&format_);
    reader.read(&quality_);
    reader.read(&data_);
    return true;
}

PacketHeader StreamPacket::getHeader() const {
    PacketHeader header{};
    header.type(STREAM_PACKET_TYPE);
    header.length(data_.length());
    return header;
}

std::string StreamPacket::toJSONString() const {
    return std::string();
}

std::string StreamPacket::format() const {
    return format_;
}

void StreamPacket::format(const std::string &format) {
    format_ = format;
    return;
}

uint8_t StreamPacket::quality() const {
    return quality_;
}

void StreamPacket::quality(uint8_t quality) {
    quality_ = quality;
    return;
}

std::string StreamPacket::data() const {
    return data_;
}

void StreamPacket::data(const std::string &data) {
    data_ = data;
    return;
}

}  // namespace datapackets
}  // namespace communication
}  // namespace crf
