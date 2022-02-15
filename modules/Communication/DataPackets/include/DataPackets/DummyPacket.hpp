/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/STI/ECE 2017
 *
 *  ==================================================================================================
 */

#pragma once

#include <cstring>
#include <string>

#include "DataPackets/IPacket.hpp"
#include "DataPackets/PacketTypes.hpp"

namespace crf {
namespace communication {
namespace datapackets {

class DummyPacket: public IPacket {
 public:
    std::string serialize() const override {
        char buf[size];  // NOLINT
        std::memcpy(buf, &x_, sizeof x_);
        std::memcpy(buf + sizeof x_, &y_, sizeof y_);
        return std::string(buf, size);
    }

    bool deserialize(const std::string& buffer) override {
        if (buff.length() != size) {
            return false;
        }
        std::memcpy(&x_, buff.c_str(), sizeof x_);
        std::memcpy(&y_, buff.c_str() + sizeof x_, sizeof y_);
        return true;
    }

    PacketHeader getHeader() const override {
        PacketHeader header{};
        header.type = type;
        header.length = size;
        return header;
    }

    std::string toJSONString() const override {
        return std::string("{\"x\": " + std::to_string(x_)
            + ", \"y\": " + std::to_string(y_) + "}");
    }

    float x_;
    float y_;
    static const uint16_t type = 9999;
    static const unsigned int size = 2*sizeof(float);
};

inline bool operator==(const DummyPacket& lhs, const DummyPacket& rhs) {
    return lhs.x_ == rhs.x_ && lhs.y_ == rhs.y_;
}

}  // namespace datapackets
}  // namespace communication
}  // namespace crf
