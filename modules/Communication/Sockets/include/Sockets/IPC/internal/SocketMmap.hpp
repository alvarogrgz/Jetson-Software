/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <atomic>
#include <chrono>
#include <shared_mutex>
#include <string>
#include <thread>

#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace communication {
namespace sockets {
namespace internal {

class SocketMmap  {
 public:
    enum Roles { Client, Server };
    enum Status { NotConnected, Active, Closed };

    SocketMmap() = delete;
    SocketMmap(const std::string& name, Roles role, uint32_t size, bool keepListening = false);
    ~SocketMmap();

    bool open();
    bool close();

    bool isOpen();

    bool write(const std::string&);
    std::string read(uint32_t length);
    std::string read(uint32_t length, const std::chrono::milliseconds& timeout);

    bool isPartnerActive();
    Status getPartnerStatus();

 private:
    typedef struct MmapBufferHeader {
        Status status;
        pthread_mutex_t bufferMutex;
        pthread_cond_t bufferCv;
        uint64_t aliveTimestamp;
        uint32_t size;
        uint32_t readPosition;
        uint32_t writePosition;
        uint32_t remainingSpace;
        bool alwaysListening;
    } MmapBufferHeader;

    utility::logger::EventLogger logger_;

    void* completeMmap_;
    MmapBufferHeader* firstBufferHeader_;
    uintptr_t* firstBuffer_;
    MmapBufferHeader* secondBufferHeader_;
    uintptr_t* secondBuffer_;

    const Roles role_;
    const std::string name_;
    const uint32_t bufferSize_;
    const uint32_t totalMmapSize_;
    int mmapFd_;
    bool isOpen_;
    bool keepListening_;

    int partnerSize_;

    bool initMmap();
    bool notifyNewData();

    bool serverSetup();
    bool clientSetup();

    void initHeaderStruct(MmapBufferHeader* ptr);

    uint64_t timestamp();

    std::shared_timed_mutex localMmapMutex_;
    bool mmaped_;
    std::atomic<bool> stopHeartbeat_;
    std::thread heartbeatThread_;
    void heartbeat();

    void autoresize(uint32_t reqWriteSize);
};

}  // namespace internal
}  // namespace sockets
}  // namespace communication
}  // namespace crf
