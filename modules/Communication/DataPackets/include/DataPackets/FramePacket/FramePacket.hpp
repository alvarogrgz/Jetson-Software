/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
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

class FramePacket : public IPacket {
 public:
    enum class Encoding { JPEG = 1, CV_MAT = 2, X264 = 3 };

    FramePacket();
    FramePacket(const std::string& bytes, Encoding encoding);
    ~FramePacket() override = default;

    std::string serialize() const override;
    bool deserialize(const std::string& buffer) override;
    PacketHeader getHeader() const override;
    std::string toJSONString() const override;

    std::string getBytes() const;
    Encoding getEncodingType() const;

 private:
    Encoding encoding_;
    std::string bytes_;
};

}  // namespace datapackets
}  // namespace communication
}  // namespace crf
