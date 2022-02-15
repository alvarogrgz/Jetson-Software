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

#include "EventLogger/EventLogger.hpp"
#include "Sockets/ISocket.hpp"
#include "Sockets/IPC/IPCServer.hpp"
#include "Sockets/IPC/internal/SocketMmap.hpp"

namespace crf {
namespace communication {
namespace sockets {

class IPCSocket : public ISocket {
 public:
    IPCSocket() = delete;
    IPCSocket(const std::string& name, uint32_t size);
    ~IPCSocket() override;

    bool open() override;
    bool close() override;

    bool isOpen() override;

    bool write(const std::string&, bool requiresAck) override;
    std::string read(int length,
        const std::chrono::milliseconds& timeout) override;

 private:
    friend class IPCServer;
    IPCSocket(std::shared_ptr<internal::SocketMmap> socket,
        const std::string& name,
        uint16_t port);

    enum Roles { Client, Server };

    utility::logger::EventLogger logger_;

    std::shared_ptr<internal::SocketMmap> socket_;
    Roles role_;
    bool isOpen_;
    std::string name_;
    uint16_t port_;
    uint32_t size_;
};

}  // namespace sockets
}  // namespace communication
}  // namespace crf
