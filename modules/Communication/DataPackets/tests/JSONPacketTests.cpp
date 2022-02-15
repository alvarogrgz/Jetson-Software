/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO 2017
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "EventLogger/EventLogger.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"

class JSONPacketShould : public ::testing::Test {
 protected:
    JSONPacketShould() :
        logger_("JSONPacketShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~JSONPacketShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    crf::communication::datapackets::JSONPacket sut_;
};

TEST_F(JSONPacketShould, returnFalseWhenDeserializeFromGarbage) {
    ASSERT_FALSE(sut_.deserialize("afdsgkjadfhjadgGARBAGE"));
}

TEST_F(JSONPacketShould, returnTrueWhenDeserializeFromJsonAndBeTheSameAfterMultipleSerialization) {
    std::string someJsonData("{\"key1\": 1234, \"key2\": \"someValue\"}");
    ASSERT_TRUE(sut_.deserialize(someJsonData));
    ASSERT_TRUE(sut_.deserialize(sut_.serialize()));
    nlohmann::json j = nlohmann::json::parse(sut_.serialize());
    ASSERT_EQ(1234, j.at("key1").get<int>());
    ASSERT_EQ("someValue", j.at("key2").get<std::string>());
}

TEST_F(JSONPacketShould, returnAppropriateHeader) {
    ASSERT_TRUE(sut_.deserialize("{\"k\": 0}"));
    crf::communication::datapackets::PacketHeader header = sut_.getHeader();
    ASSERT_EQ(crf::communication::datapackets::JSON_PACKET_TYPE, header.type());
    ASSERT_GT(sut_.getHeader().length(), 0);
}
