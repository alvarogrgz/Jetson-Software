/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#include <memory>

#include "CommunicationPointServer/CommunicationPointServer.hpp"

namespace crf {
namespace communication {
namespace communicationpointserver {

CommunicationPointServer::CommunicationPointServer(std::shared_ptr<sockets::ISocketServer> server,
    std::shared_ptr<ICommunicationPointFactory> factory) :
    logger_("CommunicationPointServer"),
    server_(server),
    factory_(factory),
    initialized_(false),
    stopThread_(false),
    acceptThread_(),
    cleanupThread_(),
    mapMutex_(),
    connectedClients_() {
    logger_->debug("CTor");
}

CommunicationPointServer::~CommunicationPointServer() {
    logger_->debug("DTor");
    if (initialized_) {
        deinitialize();
    }
}

bool CommunicationPointServer::initialize() {
    logger_->debug("initialize()");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }
    if (!server_->open()) {
        logger_->warn("Could not open server");
        return false;
    }
    stopThread_ = false;
    acceptThread_ = std::thread(&CommunicationPointServer::acceptLoop, this);
    initialized_ = true;
    return true;
}

bool CommunicationPointServer::deinitialize() {
    logger_->debug("deinitialize()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    for (auto it : connectedClients_) {
        it.second->deinitialize();
    }
    if (!server_->close()) {
        logger_->warn("Could not close server");
        return false;
    }
    stopThread_ = true;
    acceptThread_.join();
    if (cleanupThread_.joinable()) {
        cleanupThread_.join();
    }
    initialized_ = false;
    return true;
}

void CommunicationPointServer::acceptLoop() {
    logger_->debug("acceptLoop()");
    while (!stopThread_) {
        auto socket = server_->acceptConnection();
        if (!socket) {
            logger_->warn("Failed to accept client connection");
            continue;
        }
        std::shared_ptr<datapacketsocket::PacketSocket> packetSocket(
            new datapacketsocket::PacketSocket(socket.get()));
        auto commPoint = factory_->create(packetSocket);
        if (!commPoint) {
            logger_->warn("Could not create communication point");
            continue;
        }
        if (!commPoint.get()->initialize()) {
            logger_->warn("Failed to initialize communication point");
            continue;
        }
        std::unique_lock<std::mutex> lock(mapMutex_);
        connectedClients_.insert({socket.get(), commPoint.get()});
    }
}

}  // namespace communicationpointserver
}  // namespace communication
}  // namespace crf
