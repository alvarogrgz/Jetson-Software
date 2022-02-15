/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "CommunicationPointServer/ICommunicationPointFactory.hpp"
#include "CommonInterfaces/IInitializable.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Sockets/ISocketServer.hpp"

namespace crf {
namespace communication {
namespace communicationpointserver {

class CommunicationPointServer : utility::commoninterfaces::IInitializable {
 public:
    CommunicationPointServer() = delete;
    CommunicationPointServer(std::shared_ptr<sockets::ISocketServer> server,
        std::shared_ptr<ICommunicationPointFactory> factory);
    CommunicationPointServer(const CommunicationPointServer&);
    CommunicationPointServer(CommunicationPointServer&&);
    ~CommunicationPointServer() override;

    bool initialize() override;
    bool deinitialize() override;

 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<sockets::ISocketServer> server_;
    std::shared_ptr<ICommunicationPointFactory> factory_;

    bool initialized_;

    std::atomic<bool> stopThread_;
    std::thread acceptThread_;
    void acceptLoop();

    std::thread cleanupThread_;
    void cleanup();

    std::mutex mapMutex_;
    std::map<std::shared_ptr<sockets::ISocket>,
      std::shared_ptr<ICommunicationPoint>> connectedClients_;
};

}  // namespace communicationpointserver
}  // namespace communication
}  // namespace crf
