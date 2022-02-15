/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Camarero Vera CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>

#include "DataPackets/IPacket.hpp"
#include "DataPackets/PacketTypes.hpp"

namespace crf {
namespace communication {
namespace datapackets {

class StreamPacket: public IPacket {
 public:
    ~StreamPacket() override = default;

    std::string serialize() const override;
    bool deserialize(const std::string& buffer) override;
    PacketHeader getHeader() const override;
    std::string toJSONString() const override;

    std::string format() const;
    void format(const std::string &format);
    uint8_t quality() const;
    void quality(uint8_t quality);
    std::string data() const;
    void data(const std::string &data);

 private:
    std::string format_;
    uint8_t quality_;
    std::string data_;
};

}  // namespace datapackets
}  // namespace communication
}  // namespace crf
