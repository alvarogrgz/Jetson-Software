/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#define MAX_PENDING_CONNECTIONS 5

#include <arpa/inet.h>
#include <fcntl.h>
#include <memory>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdlib.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "Sockets/TCP/TCPServer.hpp"
#include "Sockets/TCP/TCPSocket.hpp"

namespace crf {
namespace communication {
namespace sockets {

TCPServer::TCPServer(uint16_t port) :
    logger_("TCPServer_" + std::to_string(port)),
    port_(port),
    serverFd_(0),
    isOpen_(false),
    connectedClients_() {
        logger_->debug("CTor");
}

TCPServer::~TCPServer() {
    logger_->debug("DTor");
    ::close(serverFd_);
}

bool TCPServer::open() {
    logger_->debug("open()");
    if (isOpen_) {
        return false;
    }

    if ((serverFd_ = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        logger_->warn("Could not create socket: {}", strerror(errno));
        return false;
    }

    struct sockaddr_in servaddr;
    std::memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = 0;
    servaddr.sin_port = htons(port_);

    int one = 1;
    if (setsockopt(serverFd_, SOL_TCP, TCP_NODELAY, &one, sizeof(one)) != 0) {
        logger_->warn("Could not set TCP_NODELAY flag: {}", strerror(errno));
        return false;
    }

    struct linger sl;
    sl.l_onoff = 1;
    sl.l_linger = 0;
    if (setsockopt(serverFd_, SOL_SOCKET, SO_LINGER, &sl, sizeof(sl)) != 0) {
        logger_->warn("Could not set SO_LINGER flag: {}", strerror(errno));
        return false;
    }

    if (bind(serverFd_, reinterpret_cast<sockaddr*>(&servaddr), sizeof(servaddr)) == -1) {
        logger_->warn("Failed to bind socket: {}", strerror(errno));
        return false;
    }

    if (listen(serverFd_, MAX_PENDING_CONNECTIONS) == -1) {
        logger_->warn("Failed to listen on socket: {}", strerror(errno));
        return false;
    }

    isOpen_ = true;
    return true;
}

bool TCPServer::close() {
    logger_->debug("close()");
    if (!isOpen_) {
        return false;
    }

    for (size_t i=0; i < connectedClients_.size(); i++) {
        connectedClients_[i]->close();
    }

    connectedClients_.clear();
    getSocketError();
    if (::shutdown(serverFd_, SHUT_RDWR) < 0) {
        if (errno != ENOTCONN && errno != EINVAL)
            logger_->warn("Could not make shutdown: {}", strerror(errno));
    }
    if (::close(serverFd_) < 0)
        logger_->warn("Could not close server: {}", strerror(errno));

    isOpen_ = false;
    return true;
}

bool TCPServer::isOpen() {
    logger_->debug("isOpen");

    return isOpen_;
}

boost::optional<std::shared_ptr<ISocket> > TCPServer::acceptConnection() {
    logger_->debug("acceptConnection()");
    if (!isOpen_) {
        logger_->warn("Socket is not open");
        return boost::none;
    }
    struct sockaddr_in client;
    uint32_t len = sizeof(client);
    std::memset(&client, 0, sizeof(client));
    int socket = accept(serverFd_, reinterpret_cast<sockaddr*>(&client), &len);

    if (socket == -1) {
        logger_->warn("Could not accept socket: {}", strerror(errno));
        return boost::none;
    }

    char str[INET_ADDRSTRLEN];
    if (inet_ntop(AF_INET, &(client.sin_addr), str, INET_ADDRSTRLEN) == NULL) {
        logger_->warn("Could not convert address from binary to text: {}", strerror(errno));
        return boost::none;
    }

    uint16_t client_port = static_cast<uint16_t>(client.sin_port);
    std::shared_ptr<ISocket> sockPtr(new TCPSocket(socket,
        std::string(str, INET_ADDRSTRLEN), client_port));
    logger_->info("Client connected from {} on port {}", client_port, port_);
    connectedClients_.push_back(sockPtr);
    return sockPtr;
}

int TCPServer::getSocketError() {
    int err = 1;
    socklen_t len = sizeof err;
    if (-1 == getsockopt(serverFd_, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&err), &len))
        logger_->error("Could not get socket error: {}", strerror(errno));
    if (err)
        errno = err;              // set errno to the socket SO_ERROR
    return err;
}

}  // namespace sockets
}  // namespace communication
}  // namespace crf
