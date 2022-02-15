/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#define CONNECTION_ESTABLISH_TIMEOUT 1

#include <limits>
#include <memory>
#include <nlohmann/json.hpp>
#include <random>
#include <string>

#include "Sockets/IPC/IPCSocket.hpp"

namespace crf {
namespace communication {
namespace sockets {

IPCSocket::IPCSocket(const std::string& name, uint32_t size) :
    logger_("IPCSocket:"+name),
    socket_(),
    role_(Roles::Client),
    isOpen_(false),
    name_(name),
    port_(0),
    size_(size) {
        logger_->debug("CTor");
}

IPCSocket::IPCSocket(std::shared_ptr<internal::SocketMmap> socket,
    const std::string& name,
    uint16_t port) :
        logger_("IPCSocket_"+name),
        socket_(socket),
        role_(Roles::Server),
        isOpen_(true),
        name_(name),
        port_(port),
        size_(0) {
            logger_->debug("CTor");
}

IPCSocket::~IPCSocket() {
    logger_->debug("DTor");
    close();
}

bool IPCSocket::open() {
    logger_->debug("open()");
    if (isOpen_) {
        logger_->warn("Already open");
        return false;
    }

    internal::SocketMmap serverSocket("/tmp/ipc_mmap/"+name_,
        internal::SocketMmap::Client, 4096);
    if (!serverSocket.open()) {
        logger_->warn("Requested server is not open");
        return false;
    }

    std::string filename;
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> rand(
        1, std::numeric_limits<uint16_t>::max());
    do {
        port_ = rand(rng);
        filename = "/tmp/ipc_mmap/"+name_+std::to_string(port_);
    } while (access(filename.c_str(), F_OK) != -1);

    socket_.reset(new internal::SocketMmap(filename,
        internal::SocketMmap::Roles::Server, size_));
    if (!socket_->open()) {
        logger_->warn("Could not open client socket");
        return false;
    }

    nlohmann::json requestOpen;
    requestOpen["port"] = port_;
    requestOpen["size"] = size_;
    std::cout << "Request to open " << requestOpen.dump() << std::endl;
    if (!serverSocket.write(requestOpen.dump())) {
        logger_->warn("Could not write open request");
        return false;
    }

    std::string reqResponse = socket_->read(2, std::chrono::milliseconds(1000));
    if (reqResponse != "OK") {
        logger_->warn("Failed to receive ack from server");
        return false;
    }

    if (!socket_->write("OK")) {
        logger_->warn("Failed to ack opening from server");
        return false;
    }

    logger_->info("Client open on socket {}:{}", name_, port_);
    isOpen_ = true;
    return true;
}

bool IPCSocket::close() {
    logger_->debug("close()");
    if (!isOpen_) {
        logger_->warn("Not open");
        return false;
    }

    socket_->close();

    isOpen_ = false;
    return true;
}

bool IPCSocket::isOpen() {
    return isOpen_;
}

bool IPCSocket::write(const std::string& buffer, bool requiresAck) {
    if (!isOpen_) {
        logger_->warn("Not open");
        return false;
    }

    return socket_->write(buffer);
}

std::string IPCSocket::read(int length,
    const std::chrono::milliseconds& timeout = std::chrono::milliseconds(0)) {
        if (!isOpen_) {
            logger_->warn("Not open");
            return std::string();
        }
        if (timeout.count() == 0) {
            return socket_->read(length);
        }
        return socket_->read(length, timeout);
}

}  // namespace sockets
}  // namespace communication
}  // namespace crf
