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
#include "Sockets/TCP/TCPServer.hpp"

namespace crf {
namespace communication {
namespace sockets {

class TCPSocket : public ISocket {
 public:
    TCPSocket() = delete;
    TCPSocket(const std::string& hostname, int port);
    TCPSocket(const TCPSocket&) = delete;
    TCPSocket(TCPSocket&&) = delete;
    ~TCPSocket() override;

    bool open() override;
    bool close() override;

    bool isOpen() override;

    bool write(const std::string&, bool requiresAck) override;
    std::string read(int length) override;
    std::string read(int length, const std::chrono::milliseconds& timeout) override;

 private:
    friend class TCPServer;
    TCPSocket(int socketFd, const std::string& hostname, int port);

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

    std::string hostname_;
    int port_;
};

}  // namespace sockets
}  // namespace communication
}  // namespace crf
