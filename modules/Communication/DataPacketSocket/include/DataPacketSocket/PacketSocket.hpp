/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <boost/optional.hpp>
#include <memory>
#include <string>
#include <utility>

#include "DataPackets/IPacket.hpp"
#include "Sockets/ISocket.hpp"

namespace crf {
namespace communication {
namespace datapacketsocket {

class PacketSocket {
 public:
    PacketSocket() = delete;
    explicit PacketSocket(std::shared_ptr<sockets::ISocket> socket);
    PacketSocket(const PacketSocket&);
    PacketSocket(PacketSocket&&);
    ~PacketSocket();

    bool open();
    bool close();

    bool isOpen();

    bool write(const datapackets::IPacket& packet, const datapackets::PacketHeader& header,
        bool requiresAck = true);

    boost::optional<std::pair<std::string, datapackets::PacketHeader>> read();
    boost::optional<std::pair<std::string, datapackets::PacketHeader>> read(
        const std::chrono::milliseconds& timeout);

 private:
    std::shared_ptr<crf::communication::sockets::ISocket> socket_;

    bool syncLost_;
    bool resync(bool blocking, const std::chrono::milliseconds& timeout);
};

}  // namespace datapacketsocket
}  // namespace communication
}  // namespace crf
