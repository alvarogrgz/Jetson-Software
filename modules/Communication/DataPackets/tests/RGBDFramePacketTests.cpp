/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */


#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "DataPackets/RGBDFramePacket/RGBDFramePacket.hpp"
#include "EventLogger/EventLogger.hpp"


using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;

using crf::communication::datapackets::RGBDFramePacket;

class RGBDFramePacketShould : public ::testing::Test {
 protected:
    RGBDFramePacketShould() :
        logger_("RGBDFramePacket"),
        sut_() {
        logger_->info("{0} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~RGBDFramePacketShould() {
        logger_->info("{0} END with {1}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<RGBDFramePacket> sut_;
};

TEST_F(RGBDFramePacketShould, initiliazesEmpty) {
    sut_.reset(new RGBDFramePacket);

    ASSERT_FALSE(sut_->containsRGB());
    ASSERT_FALSE(sut_->containsDepth());
    ASSERT_FALSE(sut_->containsPointCloud());

    ASSERT_TRUE(sut_->getRGBBytes().empty());
    ASSERT_TRUE(sut_->getDepthBytes().empty());
    ASSERT_TRUE(sut_->getPointCloudBytes().empty());

    ASSERT_EQ(sut_->serialize().length(), 1);
    ASSERT_EQ(sut_->getHeader().length(), 1);
    ASSERT_EQ(sut_->getHeader().type(), crf::communication::datapackets::RGBD_FRAME_PACKET);
}

TEST_F(RGBDFramePacketShould, setCorrectlyRgbBytes) {
    sut_.reset(new RGBDFramePacket);

    std::string bytes;
    bytes.resize(10);
    sut_->setRGBBytes(RGBDFramePacket::RGBEncoding::JPEG, bytes);
    ASSERT_TRUE(sut_->containsRGB());
    auto serbytes = sut_->serialize();
    ASSERT_EQ(serbytes.length(), sut_->getHeader().length());

    RGBDFramePacket backFrame;
    ASSERT_TRUE(backFrame.deserialize(serbytes));
    ASSERT_TRUE(backFrame.containsRGB());
    ASSERT_FALSE(backFrame.containsDepth());
    ASSERT_FALSE(backFrame.containsPointCloud());
    ASSERT_EQ(backFrame.getRGBEncoding(), RGBDFramePacket::RGBEncoding::JPEG);
    ASSERT_EQ(backFrame.getRGBBytes().length(), bytes.length());
}


TEST_F(RGBDFramePacketShould, setCorrectlyDepthBytes) {
    sut_.reset(new RGBDFramePacket);

    std::string bytes;
    bytes.resize(10);
    sut_->setDepthBytes(RGBDFramePacket::DepthEncoding::CV_MAT, bytes);
    ASSERT_TRUE(sut_->containsDepth());
    auto serbytes = sut_->serialize();
    ASSERT_EQ(serbytes.length(), sut_->getHeader().length());

    RGBDFramePacket backFrame;
    ASSERT_TRUE(backFrame.deserialize(serbytes));
    ASSERT_TRUE(backFrame.containsDepth());
    ASSERT_FALSE(backFrame.containsRGB());
    ASSERT_FALSE(backFrame.containsPointCloud());
    ASSERT_EQ(backFrame.getDepthEncoding(), RGBDFramePacket::DepthEncoding::CV_MAT);
    ASSERT_EQ(backFrame.getDepthBytes().length(), bytes.length());
}

TEST_F(RGBDFramePacketShould, setCorrectlyPointCloudBytes) {
    sut_.reset(new RGBDFramePacket);

    std::string bytes;
    bytes.resize(10);
    sut_->setPointCloudBytes(RGBDFramePacket::PointCloudEncoding::PLY, bytes);
    ASSERT_TRUE(sut_->containsPointCloud());
    auto serbytes = sut_->serialize();
    ASSERT_EQ(serbytes.length(), sut_->getHeader().length());

    RGBDFramePacket backFrame;
    ASSERT_TRUE(backFrame.deserialize(serbytes));
    ASSERT_TRUE(backFrame.containsPointCloud());
    ASSERT_FALSE(backFrame.containsDepth());
    ASSERT_FALSE(backFrame.containsRGB());
    ASSERT_EQ(backFrame.getPointCloudEncoding(), RGBDFramePacket::PointCloudEncoding::PLY);
    ASSERT_EQ(backFrame.getPointCloudBytes().length(), bytes.length());
}
