/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "DataPacketSocket/PacketSocket.hpp"

namespace crf {
namespace communication {
namespace datapacketsocket {

PacketSocket::PacketSocket(std::shared_ptr<crf::communication::sockets::ISocket> socket) :
    socket_(socket),
    syncLost_(false) {}

PacketSocket::~PacketSocket() {
    socket_->close();
}

bool PacketSocket::open() {
    return socket_->open();
}

bool PacketSocket::close() {
    return socket_->close();
}

bool PacketSocket::isOpen() {
    return socket_->isOpen();
}

bool PacketSocket::write(const datapackets::IPacket& packet,
    const datapackets::PacketHeader& header,
    bool requiresAck) {
        if (!isOpen()) {
            return false;
        }

        std::string uniqueBuffer;
        uniqueBuffer.append("~~~~~~");
        uniqueBuffer.append(header.serialize());
        uniqueBuffer.append(packet.serialize());
        return socket_->write(uniqueBuffer, requiresAck);
}

boost::optional<std::pair<std::string, datapackets::PacketHeader>> PacketSocket::read() {
    if (!isOpen()) {
        return boost::none;
    }

    bool wasSyncLost = syncLost_;
    if (syncLost_ && !resync(true, std::chrono::seconds(0))) {
        return boost::none;
    }

    if (!wasSyncLost) {
        std::string syncBytes = socket_->read(6);
        if (syncBytes != "~~~~~~") {
            syncLost_ = true;
            return boost::none;
        }
    }

    datapackets::PacketHeader header;
    std::string headerBytes = socket_->read(header.size());
    if (headerBytes.length() != header.size()) {
        syncLost_ = true;
        return boost::none;
    }

    if (!header.deserialize(headerBytes)) {
        syncLost_ = true;
        return boost::none;
    }

    std::string packetBytes = socket_->read(header.length());
    if (packetBytes.length() != header.length()) {
        syncLost_ = true;
        return boost::none;
    }

    return std::pair<std::string, datapackets::PacketHeader>({packetBytes, header});
}

boost::optional<std::pair<std::string, datapackets::PacketHeader>> PacketSocket::read(
    const std::chrono::milliseconds& timeout) {
    if (!isOpen()) {
        return boost::none;
    }

    bool wasSyncLost = syncLost_;
    if (syncLost_ && !resync(false, timeout)) {
        return boost::none;
    }

    if (!wasSyncLost) {
        std::string syncBytes = socket_->read(6);
        if (syncBytes != "~~~~~~") {
            syncLost_ = true;
            return boost::none;
        }
    }

    datapackets::PacketHeader header;
    std::string headerBytes = socket_->read(header.size(), timeout);
    if (headerBytes.length() != header.size()) {
        syncLost_ = true;
        return boost::none;
    }

    if (!header.deserialize(headerBytes)) {
        syncLost_ = true;
        return boost::none;
    }

    std::string packetBytes = socket_->read(header.length(), timeout);
    if (packetBytes.length() != header.length()) {
        syncLost_ = true;
        return boost::none;
    }

    return std::pair<std::string, datapackets::PacketHeader>({packetBytes, header});
}

bool PacketSocket::resync(bool blocking, const std::chrono::milliseconds& timeout) {
    std::string bf;
    do {
        if (blocking) {
            bf = socket_->read(1);
        } else {
            bf = socket_->read(1, timeout);
        }

        if (bf.length() == 0) {
            return false;
        }
    } while (bf[0] != '~');

    if (blocking) {
        bf = socket_->read(5);
    } else {
        bf = socket_->read(5, timeout);
    }

    if (bf != "~~~~~") {
        return false;
    }

    syncLost_ = false;
    return true;
}

}  // namespace datapacketsocket
}  // namespace communication
}  // namespace crf
