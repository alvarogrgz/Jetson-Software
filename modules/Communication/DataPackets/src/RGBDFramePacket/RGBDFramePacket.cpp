/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#include <bitset>
#include <string>

#include "DataPackets/RGBDFramePacket/RGBDFramePacket.hpp"
#include "CommunicationUtility/StreamWriter.hpp"
#include "CommunicationUtility/StreamReader.hpp"

namespace crf {
namespace communication {
namespace datapackets {

RGBDFramePacket::RGBDFramePacket() :
    contains_(0),
    rgbEncoding_(RGBDFramePacket::RGBEncoding::CV_MAT),
    rgbBytes_(),
    depthEncoding_(RGBDFramePacket::DepthEncoding::CV_MAT),
    depthBytes_(),
    pointCloudEncoding_(RGBDFramePacket::PointCloudEncoding::PLY),
    pointCloudBytes_() {
}

std::string RGBDFramePacket::serialize() const {
    crf::utility::communicationutility::StreamWriter writer;

    writer.write(static_cast<uint8_t>(contains_.to_ulong()));
    if (contains_[0]) {
        writer.write(static_cast<uint8_t>(rgbEncoding_));
        writer.write(rgbBytes_);
    }
    if (contains_[1]) {
        writer.write(static_cast<uint8_t>(depthEncoding_));
        writer.write(depthBytes_);
    }
    if (contains_[2]) {
        writer.write(static_cast<uint8_t>(pointCloudEncoding_));
        writer.write(pointCloudBytes_);
    }
    return writer.toString();
}

bool RGBDFramePacket::deserialize(const std::string& buffer) {
    crf::utility::communicationutility::StreamReader reader(buffer);

    uint8_t containsBits;
    if (!reader.read(&containsBits)) {
        return false;
    }

    contains_ = std::bitset<3>(containsBits);
    if (contains_[0]) {
        if (!reader.read(reinterpret_cast<uint8_t*>(&rgbEncoding_))) {
            return false;
        }
        if (!reader.read(&rgbBytes_)) {
            return false;
        }
    }
    if (contains_[1]) {
        if (!reader.read(reinterpret_cast<uint8_t*>(&depthEncoding_))) {
            return false;
        }
        if (!reader.read(&depthBytes_)) {
            return false;
        }
    }
    if (contains_[2]) {
        if (!reader.read(reinterpret_cast<uint8_t*>(&pointCloudEncoding_))) {
            return false;
        }
        if (!reader.read(&pointCloudBytes_)) {
            return false;
        }
    }
    return true;
}

PacketHeader RGBDFramePacket::getHeader() const {
    PacketHeader header;
    header.type(RGBD_FRAME_PACKET);
    uint32_t length = 1;  // contains_ bytes
    // RGB bytes
    length += contains_[0] ? sizeof(uint8_t) + sizeof(uint32_t) + rgbBytes_.length() : 0;
    // Depth bytes
    length += contains_[1] ? sizeof(uint8_t) + sizeof(uint32_t) + depthBytes_.length() : 0;
    // Point Cloud bytes
    length += contains_[2] ? sizeof(uint8_t) + sizeof(uint32_t) + pointCloudBytes_.length() : 0;
    header.length(length);
    return header;
}

std::string RGBDFramePacket::toJSONString() const {
    return std::string();
}

bool RGBDFramePacket::containsRGB() const {
    return contains_[0];
}

bool RGBDFramePacket::containsDepth() const {
    return contains_[1];
}

bool RGBDFramePacket::containsPointCloud() const {
    return contains_[2];
}

RGBDFramePacket::RGBEncoding RGBDFramePacket::getRGBEncoding() const {
    return rgbEncoding_;
}

RGBDFramePacket::DepthEncoding RGBDFramePacket::getDepthEncoding() const {
    return depthEncoding_;
}

RGBDFramePacket::PointCloudEncoding RGBDFramePacket::getPointCloudEncoding() const {
    return pointCloudEncoding_;
}

std::string RGBDFramePacket::getRGBBytes() const {
    return rgbBytes_;
}

std::string RGBDFramePacket::getDepthBytes() const {
    return depthBytes_;
}

std::string RGBDFramePacket::getPointCloudBytes() const {
    return pointCloudBytes_;
}

void RGBDFramePacket::setRGBBytes(RGBDFramePacket::RGBEncoding econding,
    const std::string& bytes) {
    contains_[0] = 1;
    rgbEncoding_ = econding;
    rgbBytes_ = bytes;
    return;
}

void RGBDFramePacket::setDepthBytes(RGBDFramePacket::DepthEncoding encoding,
    const std::string& bytes) {
    contains_[1] = 1;
    depthEncoding_ = encoding;
    depthBytes_ = bytes;
    return;
}

void RGBDFramePacket::setPointCloudBytes(RGBDFramePacket::PointCloudEncoding encoding,
    const std::string& bytes) {
    contains_[2] = 1;
    pointCloudEncoding_ = encoding;
    pointCloudBytes_ = bytes;
    return;
}

}  // namespace datapackets
}  // namespace communication
}  // namespace crf
