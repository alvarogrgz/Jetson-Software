/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "DataPacketSocket/PacketSocket.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Sockets/SocketMock.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::communication::datapacketsocket::PacketSocket;
using crf::communication::sockets::SocketMock;

class PacketSocketShould : public ::testing::Test {
 protected:
    PacketSocketShould() :
        logger_("PacketSocketShould"),
        sut_(),
        socket_(new NiceMock<SocketMock>),
        bufferToRead_(),
        writtenBuffer_() {
        logger_->info("{0} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    void SetUp() override {
        ON_CALL(*socket_, open()).WillByDefault(Return(true));
        ON_CALL(*socket_, close()).WillByDefault(Return(true));
        ON_CALL(*socket_, isOpen()).WillByDefault(Return(true));
        ON_CALL(*socket_, write(_, _)).WillByDefault(Invoke(
            [this](const std::string& buffer, bool reqAck){
                    writtenBuffer_ = buffer;
                    return true;
        }));

        ON_CALL(*socket_, read(_)).WillByDefault(Invoke([this](int length){
            auto str = bufferToRead_.substr(0, length);
            bufferToRead_.erase(0, length);
            return str;
        }));

        ON_CALL(*socket_, read(_, _)).WillByDefault(
            Invoke([this](int length, const std::chrono::milliseconds& timeout){
                if (bufferToRead_.length() < static_cast<size_t>(length)) {
                    return std::string();
                }

                auto str = bufferToRead_.substr(0, length);
                bufferToRead_.erase(0, length);
                return str;
        }));
    }

    ~PacketSocketShould() {
        logger_->info("{0} END with {1}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void putToReadPacket(const crf::communication::datapackets::IPacket& packet) {
        bufferToRead_.clear();
        bufferToRead_.append("~~~~~~");
        bufferToRead_.append(packet.getHeader().serialize());
        bufferToRead_.append(packet.serialize());
    }

    crf::utility::logger::EventLogger logger_;

    std::unique_ptr<PacketSocket> sut_;
    std::shared_ptr<SocketMock> socket_;

    std::string bufferToRead_;
    std::string writtenBuffer_;
};

TEST_F(PacketSocketShould, openCloseIsOpenWriteTest) {
    sut_.reset(new PacketSocket(socket_));

    ASSERT_TRUE(sut_->open());
    ASSERT_TRUE(sut_->isOpen());
    ASSERT_TRUE(sut_->close());
}

TEST_F(PacketSocketShould, writeCorrectBytes) {
    sut_.reset(new PacketSocket(socket_));
    crf::communication::datapackets::JSONPacket json;
    json.data["data"] = "tentative";

    ASSERT_TRUE(sut_->write(json, json.getHeader()));
    ASSERT_EQ(writtenBuffer_.length(), 6 + json.serialize().length() + json.getHeader().size());
}

TEST_F(PacketSocketShould, readCorrectBytes) {
    sut_.reset(new PacketSocket(socket_));
    crf::communication::datapackets::JSONPacket json;
    json.data["data"] = "tentative";

    putToReadPacket(json);
    auto retval = sut_->read();

    ASSERT_TRUE(retval);
    ASSERT_EQ(retval.get().first.length(), json.serialize().length());
}

TEST_F(PacketSocketShould, readCorrectBytesWithTimeout) {
    sut_.reset(new PacketSocket(socket_));
    crf::communication::datapackets::JSONPacket json;
    json.data["data"] = "tentative";

    putToReadPacket(json);
    auto retval = sut_->read(std::chrono::milliseconds(100));

    ASSERT_TRUE(retval);
    ASSERT_EQ(retval.get().first.length(), json.serialize().length());
}

TEST_F(PacketSocketShould, correctlyResyncAfterSomeBullshit) {
    sut_.reset(new PacketSocket(socket_));
    crf::communication::datapackets::JSONPacket json;
    json.data["data"] = "tentative";

    bufferToRead_.clear();
    bufferToRead_.append("~~~~~");
    bufferToRead_.append(json.getHeader().serialize());
    bufferToRead_.append(json.serialize());
    auto retval = sut_->read();
    ASSERT_FALSE(retval);

    bufferToRead_.append("~~~~~~");
    bufferToRead_.append(json.getHeader().serialize());
    bufferToRead_.append(json.serialize());

    retval = sut_->read();
    ASSERT_TRUE(retval);
    ASSERT_EQ(retval.get().first.length(), json.serialize().length());

    bufferToRead_.clear();
    bufferToRead_.append("~~~~~");
    bufferToRead_.append(std::string("asd"));
    bufferToRead_.append(json.serialize());
    retval = sut_->read();
    ASSERT_FALSE(retval);

    bufferToRead_.append("~~~~~~");
    bufferToRead_.append(json.getHeader().serialize());
    bufferToRead_.append(json.serialize());

    retval = sut_->read();
    ASSERT_TRUE(retval);
    ASSERT_EQ(retval.get().first.length(), json.serialize().length());

    bufferToRead_.clear();
    bufferToRead_.append("~~~~~");
    bufferToRead_.append(json.getHeader().serialize());
    bufferToRead_.append("asd");
    retval = sut_->read();
    ASSERT_FALSE(retval);

    bufferToRead_.append("~~~~~~");
    bufferToRead_.append(json.getHeader().serialize());
    bufferToRead_.append(json.serialize());

    retval = sut_->read();
    ASSERT_TRUE(retval);
    ASSERT_EQ(retval.get().first.length(), json.serialize().length());
}

TEST_F(PacketSocketShould, correctlyResyncAfterSomeBullshitNonBlocking) {
    sut_.reset(new PacketSocket(socket_));
    crf::communication::datapackets::JSONPacket json;
    json.data["data"] = "tentative";

    bufferToRead_.clear();
    bufferToRead_.append("~~~~~");
    bufferToRead_.append(json.getHeader().serialize());
    bufferToRead_.append(json.serialize());
    auto retval = sut_->read(std::chrono::milliseconds(100));
    ASSERT_FALSE(retval);

    bufferToRead_.append("~~~~~~");
    bufferToRead_.append(json.getHeader().serialize());
    bufferToRead_.append(json.serialize());

    retval = sut_->read();
    ASSERT_TRUE(retval);
    ASSERT_EQ(retval.get().first.length(), json.serialize().length());

    bufferToRead_.clear();
    bufferToRead_.append("~~~~~");
    bufferToRead_.append(std::string("asd"));
    bufferToRead_.append(json.serialize());
    retval = sut_->read(std::chrono::milliseconds(100));
    ASSERT_FALSE(retval);

    bufferToRead_.append("~~~~~~");
    bufferToRead_.append(json.getHeader().serialize());
    bufferToRead_.append(json.serialize());

    retval = sut_->read(std::chrono::milliseconds(100));
    ASSERT_TRUE(retval);
    ASSERT_EQ(retval.get().first.length(), json.serialize().length());

    bufferToRead_.clear();
    bufferToRead_.append("~~~~~");
    bufferToRead_.append(json.getHeader().serialize());
    bufferToRead_.append("asd");
    retval = sut_->read(std::chrono::milliseconds(100));
    ASSERT_FALSE(retval);

    bufferToRead_.append("~~~~~~");
    bufferToRead_.append(json.getHeader().serialize());
    bufferToRead_.append(json.serialize());

    retval = sut_->read();
    ASSERT_TRUE(retval);
    ASSERT_EQ(retval.get().first.length(), json.serialize().length());
}
