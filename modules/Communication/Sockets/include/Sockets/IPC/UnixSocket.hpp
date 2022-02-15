/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <chrono>
#include <string>

#include "EventLogger/EventLogger.hpp"
#include "Sockets/ISocket.hpp"
#include "Sockets/IPC/UnixSocketServer.hpp"

namespace crf {
namespace communication {
namespace sockets {

class UnixSocket : public ISocket {
 public:
    UnixSocket() = delete;
    explicit UnixSocket(const std::string& name);
    UnixSocket(const UnixSocket&) = delete;
    UnixSocket(UnixSocket&&) = delete;
    ~UnixSocket() override;

    bool open() override;
    bool close() override;

    bool isOpen() override;

    bool write(const std::string&, bool requiresAck) override;
    std::string read(int length) override;
    std::string read(int length, const std::chrono::milliseconds& timeout) override;

 private:
    friend class UnixSocketServer;
    UnixSocket(int socketFd, const std::string& name);

    bool setNonBlockingFlag(bool value);
    std::string nonBlockingRead(int length, const std::chrono::milliseconds& timeout);
    std::string blockingRead(int length);
    int getSocketError();

    enum Roles { Client, Server };

    utility::logger::EventLogger logger_;

    int socketFd_;
    Roles role_;
    bool isOpen_;

    bool isBlocking_;

    std::string name_;
};

}  // namespace sockets
}  // namespace communication
}  // namespace crf
