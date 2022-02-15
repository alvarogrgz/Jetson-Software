/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#define CONNECTION_ESTABLISH_TIMEOUT 1

#include <dirent.h>
#include <fcntl.h>
#include <memory>
#include <nlohmann/json.hpp>
#include <stdlib.h>
#include <string>
#include <sys/types.h>
#include <unistd.h>

#include "Sockets/IPC/IPCServer.hpp"

namespace crf {
namespace communication {
namespace sockets {

IPCServer::IPCServer(const std::string& name) :
    logger_("IPCServer:"+name),
    name_(name),
    socket_(),
    isOpen_(false),
    connectedClients_() {
        logger_->debug("CTor");
}

IPCServer::~IPCServer() {
    logger_->debug("DTor");
    close();
}

bool IPCServer::open() {
    logger_->debug("open()");
    if (isOpen_) {
        logger_->warn("Already open");
        return false;
    }

    DIR* dir = opendir("/tmp/ipc_mmap/");
    if (dir) {
        if (chmod("/tmp/ipc_mmap/", 0777) != 0) {
            logger_->warn("Could not change folder permissions: {}", strerror(errno));
            return false;
        }
        closedir(dir);
    } else if (ENOENT == errno) {
        if (mkdir("/tmp/ipc_mmap/", 0777) != 0) {
            logger_->warn("Could not create ipc folder: {}", strerror(errno));
            return false;
        }
    } else {
        logger_->warn("Could not open ipc directory");
        return false;
    }

    socket_.reset(new internal::SocketMmap("/tmp/ipc_mmap/"+name_,
        internal::SocketMmap::Roles::Server, 4096, true));

    if (!socket_->open()) {
        return false;
    }

    logger_->info("Server open on channel {}", name_);
    isOpen_ = true;
    return true;
}

bool IPCServer::close() {
    logger_->debug("close()");
    if (!isOpen_) {
        logger_->warn("Not open");
        return false;
    }

    for (size_t i = 0; i < connectedClients_.size(); i++) {
        connectedClients_[i]->close();
    }

    if (!socket_->close()) {
        return false;
    }

    isOpen_ = false;
    return true;
}

bool IPCServer::isOpen() {
    return isOpen_;
}

boost::optional<std::shared_ptr<ISocket> > IPCServer::acceptConnection() {
    logger_->debug("acceptConnection()");
    if (!isOpen_) {
        logger_->warn("Server is not open");
        return boost::none;
    }

    std::string request;
    std::string byte;
    do {
        byte = socket_->read(1);
        if (byte.length() == 0) {
            return boost::none;
        }
        request.append(byte);
    } while (byte[0] != '}');

    nlohmann::json jsonRequest;
    uint16_t port;
    int size;
    try {
        jsonRequest = nlohmann::json::parse(request);
        port = jsonRequest["port"].get<uint16_t>();
        size = jsonRequest["size"].get<int>();
    } catch (const std::exception& e) {
        logger_->warn("Invalid request: {}", e.what());
        return boost::none;
    }

    std::cout << "Received request to open " << jsonRequest.dump() << std::endl;
    std::shared_ptr<internal::SocketMmap> clientMmap(
        new internal::SocketMmap("/tmp/ipc_mmap/"+name_+std::to_string(port),
        internal::SocketMmap::Roles::Client, size < 4096 ? 4096 : size));

    if (!clientMmap->open()) {
        logger_->warn("Failed to open requested socket on port {}", port);
        return boost::none;
    }

    if (!clientMmap->write("OK")) {
        logger_->warn("Failed to establish socket connection");
        return boost::none;
    }

    std::string ackOk = clientMmap->read(2, std::chrono::seconds(CONNECTION_ESTABLISH_TIMEOUT));
    if (ackOk != "OK") {
        logger_->warn("Failed to establish socket connection");
        return boost::none;
    }

    std::shared_ptr<ISocket> clientSocket(new IPCSocket(clientMmap, name_, port));
    logger_->info("Client connected on socket {}:{}", name_, port);
    return clientSocket;
}

}  // namespace sockets
}  // namespace communication
}  // namespace crf
