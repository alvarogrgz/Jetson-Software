/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "Sockets/ISocketServer.hpp"
#include "Sockets/IPC/IPCSocket.hpp"
#include "Sockets/IPC/internal/SocketMmap.hpp"

namespace crf {
namespace communication {
namespace sockets {

class IPCServer : public ISocketServer {
 public:
    IPCServer() = delete;
    explicit IPCServer(const std::string& name);
    ~IPCServer() override;

    bool open() override;
    bool close() override;

    bool isOpen() override;

    boost::optional<std::shared_ptr<ISocket> > acceptConnection() override;

 private:
    utility::logger::EventLogger logger_;

    std::string name_;
    std::unique_ptr<internal::SocketMmap> socket_;

    bool isOpen_;
    std::vector<std::shared_ptr<ISocket> > connectedClients_;
};

}  // namespace sockets
}  // namespace communication
}  // namespace crf
