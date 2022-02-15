/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <string>

#include "EventLogger/EventLogger.hpp"
#include "DataPackets/PacketHeader.hpp"

class PacketHeaderShould : public ::testing::Test {
 protected:
    PacketHeaderShould() :
        logger_("PacketHeaderShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~PacketHeaderShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
};

TEST(PacketHeaderShould, returnTrueWhenDeserializedFromCorrectlySerializedPacket) {
    crf::communication::datapackets::PacketHeader expectedHeader;
    crf::communication::datapackets::PacketHeader obtainedHeader;
    expectedHeader.type(1);
    expectedHeader.length(2);
    expectedHeader.timeStamp(3);
    expectedHeader.writerID(4);
    ASSERT_TRUE(obtainedHeader.deserialize(expectedHeader.serialize()));
    ASSERT_EQ(expectedHeader.type(), obtainedHeader.type());
    ASSERT_EQ(expectedHeader.length(), obtainedHeader.length());
    ASSERT_EQ(expectedHeader.timeStamp(), obtainedHeader.timeStamp());
    ASSERT_EQ(expectedHeader.writerID(), obtainedHeader.writerID());
}

TEST(PacketHeaderShould, returnFalseWhenDeserializedFromGarbage) {
    crf::communication::datapackets::PacketHeader obtainedHeader;
    std::string garbage(obtainedHeader.size(), '?');
    ASSERT_FALSE(obtainedHeader.deserialize(garbage));
}

TEST(PacketHeaderShould, returnTrueIfCrcIsCorrect) {
    /*
     * Data to reproduce a bug (comparison between signed and unsigned) found during the experiment
     * in the lab.
     */
    crf::communication::datapackets::PacketHeader expectedHeader;
    crf::communication::datapackets::PacketHeader obtainedHeader;
    expectedHeader.type(801);
    expectedHeader.length(155);
    expectedHeader.timeStamp(0);
    expectedHeader.writerID(0);
    ASSERT_TRUE(obtainedHeader.deserialize(expectedHeader.serialize()));
    ASSERT_EQ(expectedHeader.type(), obtainedHeader.type());
    ASSERT_EQ(expectedHeader.length(), obtainedHeader.length());
    ASSERT_EQ(expectedHeader.timeStamp(), obtainedHeader.timeStamp());
    ASSERT_EQ(expectedHeader.writerID(), obtainedHeader.writerID());
}
