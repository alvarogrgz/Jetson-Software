/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <bitset>
#include <string>

#include "DataPackets/IPacket.hpp"
#include "DataPackets/PacketTypes.hpp"

namespace crf {
namespace communication {
namespace datapackets {

class RGBDFramePacket : public IPacket {
 public:
    enum class RGBEncoding { JPEG = 1, CV_MAT = 2, X264 = 3 };
    enum class DepthEncoding { CV_MAT = 1, LZ4 = 2 };
    enum class PointCloudEncoding { PLY = 1, LZ4 = 2 };

    RGBDFramePacket();
    ~RGBDFramePacket() override = default;

    std::string serialize() const override;
    bool deserialize(const std::string& buffer) override;
    PacketHeader getHeader() const override;
    std::string toJSONString() const override;

    bool containsRGB() const;
    bool containsDepth() const;
    bool containsPointCloud() const;

    RGBEncoding getRGBEncoding() const;
    DepthEncoding getDepthEncoding() const;
    PointCloudEncoding getPointCloudEncoding() const;

    std::string getRGBBytes() const;
    std::string getDepthBytes() const;
    std::string getPointCloudBytes() const;

    void setRGBBytes(RGBEncoding econding, const std::string& bytes);
    void setDepthBytes(DepthEncoding encoding, const std::string& bytes);
    void setPointCloudBytes(PointCloudEncoding encoding, const std::string& bytes);

 private:
    std::bitset<3> contains_;

    RGBEncoding rgbEncoding_;
    std::string rgbBytes_;

    DepthEncoding depthEncoding_;
    std::string depthBytes_;

    PointCloudEncoding pointCloudEncoding_;
    std::string pointCloudBytes_;
};

}  // namespace datapackets
}  // namespace communication
}  // namespace crf
