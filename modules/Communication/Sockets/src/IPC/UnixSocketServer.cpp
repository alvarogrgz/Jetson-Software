/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#define MAX_PENDING_CONNECTIONS 5

#include <arpa/inet.h>
#include <dirent.h>
#include <fcntl.h>
#include <memory>
#include <netdb.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>
#include <sys/stat.h>

#include "Sockets/IPC/UnixSocketServer.hpp"
#include "Sockets/IPC/UnixSocket.hpp"

namespace crf {
namespace communication {
namespace sockets {

UnixSocketServer::UnixSocketServer(const std::string& name) :
    logger_("UnixSocketServer_" + name),
    name_("/tmp/ipc_unixdomain/"+name),
    serverFd_(0),
    isOpen_(false),
    connectedClients_() {
        logger_->debug("CTor");
}

UnixSocketServer::~UnixSocketServer() {
    logger_->debug("DTor");
    ::close(serverFd_);
}

bool UnixSocketServer::open() {
    logger_->debug("open()");
    if (isOpen_) {
        return false;
    }

    DIR* dir = opendir("/tmp/ipc_unixdomain/");
    if (dir) {
        if (chmod("/tmp/ipc_unixdomain", 0777) != 0) {
            logger_->warn("Could not change folder permissions: {}", strerror(errno));
            return false;
        }
        closedir(dir);
    } else if (errno == ENOENT) {
        if (mkdir("/tmp/ipc_unixdomain/", 0777) != 0) {
            logger_->warn("Could not create ipc folder: {}", strerror(errno));
            return false;
        }
    } else {
        logger_->warn("Could not open ipc directory");
        return false;
    }

    if ((serverFd_ = socket(AF_UNIX, SOCK_STREAM, 0)) == -1) {
        logger_->warn("Could not create socket: {}", strerror(errno));
        return false;
    }

    struct sockaddr_un servaddr;
    std::memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sun_family = AF_UNIX;
    strncpy(servaddr.sun_path, name_.c_str(), name_.length());
    if (bind(serverFd_, reinterpret_cast<sockaddr*>(&servaddr), sizeof(servaddr)) == -1) {
        logger_->warn("Failed to bind socket: {}", strerror(errno));
        return false;
    }

    if (listen(serverFd_, MAX_PENDING_CONNECTIONS) == -1) {
        logger_->warn("Failed to listen on socket: {}", strerror(errno));
        return false;
    }

    if (chmod(servaddr.sun_path, 0777) != 0) {
        logger_->warn("Could not change folder permissions: {}", strerror(errno));
        return false;
    }

    isOpen_ = true;
    return true;
}

bool UnixSocketServer::close() {
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

    unlink(name_.c_str());
    isOpen_ = false;
    return true;
}

bool UnixSocketServer::isOpen() {
    logger_->debug("isOpen");

    return isOpen_;
}

boost::optional<std::shared_ptr<ISocket> > UnixSocketServer::acceptConnection() {
    logger_->debug("acceptConnection()");
    if (!isOpen_) {
        logger_->warn("Socket is not open");
        return boost::none;
    }
    struct sockaddr_un client;
    std::memset(&client, 0, sizeof(client));
    uint32_t len = sizeof(client);
    int socket = accept(serverFd_, reinterpret_cast<sockaddr*>(&client), &len);

    if (socket == -1) {
        logger_->warn("Could not accept socket: {}", strerror(errno));
        return boost::none;
    }

    logger_->info("Client connected on resource {}", name_);
    std::shared_ptr<ISocket> sockPtr(new UnixSocket(socket, name_));
    connectedClients_.push_back(sockPtr);
    return sockPtr;
}

int UnixSocketServer::getSocketError() {
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
