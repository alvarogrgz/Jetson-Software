/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MNRO 2017
 *
 *  ==================================================================================================
 */

#include <cstring>
#include <memory>
#include <string>

#include "DataPackets/PacketHeader.hpp"

namespace crf {
namespace communication {
namespace datapackets {

PacketHeader::PacketHeader() :
    type_(),
    length_(),
    timeStamp_(),
    writerID_() {
}

std::string PacketHeader::serialize() const {
    char buffer[size_]; // NOLINT
    std::memcpy(buffer, &type_, 2);
    std::memcpy(buffer+2, &length_, 4);
    std::memcpy(buffer+6, &timeStamp_, 8);
    std::memcpy(buffer+14, &writerID_, 1);
    buffer[size_-1] = calculateCrc();
    return std::string(buffer, size_);
}

bool PacketHeader::deserialize(const std::string& buffer) {
    if (buffer.length() != size_) {
        return false;
    }
    std::memcpy(&type_, buffer.c_str(), 2);
    std::memcpy(&length_, buffer.c_str()+2, 4);
    std::memcpy(&timeStamp_, buffer.c_str()+6, 8);
    std::memcpy(&writerID_, buffer.c_str()+14, 1);
    if (static_cast<uint8_t>(buffer.back()) != calculateCrc()) {
        return false;
    }
    return true;
}

std::string PacketHeader::toJSONString() const {
    std::string json = "{length: " + std::to_string(length_) + ", type: " + std::to_string(type_) +
        ", timestamp: " + std::to_string(timeStamp_) + "}";
    return json;
}

unsigned int PacketHeader::size() const {
    return size_;
}

uint16_t PacketHeader::type() const {
    return type_;
}

void PacketHeader::type(uint16_t type) {
    type_ = type;
    return;
}

uint32_t PacketHeader::length() const {
    return length_;
}

void PacketHeader::length(uint32_t length) {
    length_ = length;
    return;
}

int64_t PacketHeader::timeStamp() const {
    return timeStamp_;
}

void PacketHeader::timeStamp(int64_t timeStamp) {
    timeStamp_ = timeStamp;
    return;
}

uint8_t PacketHeader::writerID() const {
    return writerID_;
}

void PacketHeader::writerID(uint8_t writerID) {
    writerID_ = writerID;
    return;
}

uint8_t PacketHeader::calculateCrc() const {
    /*
     * TODO: I imagine you might wish to have some more sophisticated algorithm
     */
    return (type_ + length_ + timeStamp_ + writerID_) % 255;
}

}  // namespace datapackets
}  // namespace communication
}  // namespace crf
