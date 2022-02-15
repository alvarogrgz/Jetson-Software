/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#define PARTNER_TIMEOUT_MS 5000

#include <algorithm>
#include <chrono>
#include <errno.h>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>

#include "Sockets/Ipc/internal/SocketMmap.hpp"

namespace crf {
namespace communication {
namespace sockets {
namespace internal {

SocketMmap::SocketMmap(const std::string& name, Roles role, uint32_t size, bool keepListening) :
    logger_("SocketMmap"),
    completeMmap_(),
    firstBufferHeader_(),
    firstBuffer_(),
    secondBufferHeader_(),
    secondBuffer_(),
    role_(role),
    name_(name),
    bufferSize_(size),
    totalMmapSize_((sizeof(MmapBufferHeader)+bufferSize_)*2),
    mmapFd_(0),
    isOpen_(false),
    keepListening_(keepListening),
    localMmapMutex_(),
    mmaped_(false),
    stopHeartbeat_(false),
    heartbeatThread_() {
        logger_->debug("CTor: {} {}", name_, role_);
    }

SocketMmap::~SocketMmap() {
    logger_->debug("DTor:  {} {}", name_, role_);
    close();
}

bool SocketMmap::open() {
    logger_->debug("open()");
    if (isOpen_) {
        return false;
    }

    bool retval = false;
    if (role_ == Roles::Server) {
        retval = serverSetup();
    } else {
        retval = clientSetup();
    }

    if (!retval) {
        return false;
    }

    isOpen_ = true;
    stopHeartbeat_ = false;
    heartbeatThread_ = std::thread(&SocketMmap::heartbeat, this);
    return true;
}

bool SocketMmap::close() {
    logger_->debug("close()");
    if (!isOpen_) {
        return false;
    }
    if (role_ == Roles::Server) {
        firstBufferHeader_->status = Status::Closed;
        pthread_mutex_lock(&secondBufferHeader_->bufferMutex);
        pthread_cond_broadcast(&secondBufferHeader_->bufferCv);
        pthread_mutex_unlock(&secondBufferHeader_->bufferMutex);
    } else {
        secondBufferHeader_->status = Status::Closed;
        pthread_mutex_lock(&firstBufferHeader_->bufferMutex);
        pthread_cond_broadcast(&firstBufferHeader_->bufferCv);
        pthread_mutex_unlock(&firstBufferHeader_->bufferMutex);
    }

    std::lock_guard<std::shared_timed_mutex> guard(localMmapMutex_);
    stopHeartbeat_ = true;
    heartbeatThread_.join();

    bool deleteFile = !isPartnerActive();

    munmap(completeMmap_, totalMmapSize_);
    mmaped_ = false;
    ::close(mmapFd_);

    if (deleteFile)
        ::remove(name_.c_str());

    isOpen_ = false;
    return true;
}

bool SocketMmap::isOpen() {
    return isOpen_;
}

bool SocketMmap::write(const std::string& buffer) {
    if (!isOpen_) {
        return false;
    }

    std::shared_lock<std::shared_timed_mutex> guard(localMmapMutex_);
    if (!mmaped_) {
        return false;
    }

    if (!keepListening_ && !isPartnerActive()) {
        return false;
    }

    uintptr_t* mmapPtr = role_ == Roles::Server ? firstBuffer_ : secondBuffer_;
    MmapBufferHeader* header = role_ == Roles::Server ? firstBufferHeader_ : secondBufferHeader_;
    if (buffer.size() > header->remainingSpace) {
        logger_->error("No buffer space available");
        return false;
    }

    int newWritePos = header->writePosition;
    if (buffer.size() + header->writePosition > header->size) {
        int available = header->size - header->writePosition;
        std::memcpy(mmapPtr + header->writePosition,
            buffer.c_str(), available);
        std::memcpy(mmapPtr,
            buffer.c_str() + available, buffer.length()-available);
        newWritePos = buffer.length()-available;
    } else {
        std::memcpy(mmapPtr + header->writePosition,
            buffer.c_str(), buffer.length());
        newWritePos += buffer.length();
    }

    pthread_mutex_lock(&header->bufferMutex);
    header->writePosition = newWritePos;
    header->remainingSpace -= buffer.length();
    pthread_cond_broadcast(&header->bufferCv);
    pthread_mutex_unlock(&header->bufferMutex);

    return true;
}

std::string SocketMmap::read(uint32_t length) {
    if (!isOpen_) {
        return std::string();
    }

    std::shared_lock<std::shared_timed_mutex> guard(localMmapMutex_);
    if (!mmaped_) {
        return std::string();
    }

    if (!keepListening_ && !isPartnerActive()) {
        return std::string();
    }

    uintptr_t* mmapPtr = role_ == Roles::Server ? secondBuffer_ : firstBuffer_;
    MmapBufferHeader* header = role_ == Roles::Server ? secondBufferHeader_ : firstBufferHeader_;

    pthread_mutex_lock(&header->bufferMutex);
    uint32_t availableToRead = header->size - header->remainingSpace;
    pthread_mutex_unlock(&header->bufferMutex);

    while (availableToRead < length) {
        pthread_mutex_lock(&header->bufferMutex);
        pthread_cond_wait(&header->bufferCv, &header->bufferMutex);
        availableToRead = header->size - header->remainingSpace;
        pthread_mutex_unlock(&header->bufferMutex);

        if (!mmaped_) {
            return std::string();
        }
    }

    pthread_mutex_lock(&header->bufferMutex);
    uint32_t writePosition = header->writePosition;
    pthread_mutex_unlock(&header->bufferMutex);

    char* buf = new char[length];
    uint32_t newReadPosition = header->readPosition;
    if ((writePosition > header->readPosition) ||
        ((header->readPosition + length) < header->size)) {
            std::memcpy(buf, mmapPtr+header->readPosition, length);
            newReadPosition += length;
    } else {
        std::memcpy(buf, mmapPtr+header->readPosition, header->size - header->readPosition);
        std::memcpy(buf, mmapPtr, length - (header->size - header->readPosition));
        newReadPosition = length - (header->size - header->readPosition);
    }

    pthread_mutex_lock(&header->bufferMutex);
    header->readPosition = newReadPosition;
    header->remainingSpace += length;
    pthread_mutex_unlock(&header->bufferMutex);

    std::string retval(buf, length);
    delete[] buf;
    return retval;
}

std::string SocketMmap::read(uint32_t length, const std::chrono::milliseconds& timeout) {
    if (!isOpen_) {
        return std::string();
    }

    std::shared_lock<std::shared_timed_mutex> guard(localMmapMutex_);
    if (!mmaped_) {
        return std::string();
    }

    if (!keepListening_ && !isPartnerActive()) {
        return std::string();
    }

    uintptr_t* mmapPtr = role_ == Roles::Server ? secondBuffer_ : firstBuffer_;
    MmapBufferHeader* header = role_ == Roles::Server ? secondBufferHeader_ : firstBufferHeader_;

    pthread_mutex_lock(&header->bufferMutex);
    uint32_t availableToRead = header->size - header->remainingSpace;
    pthread_mutex_unlock(&header->bufferMutex);

    if ((timeout.count() == 0) && (availableToRead < length)) {
        return std::string();
    }

    timespec endTime;
    clock_gettime(CLOCK_REALTIME, &endTime);
    int seconds = timeout.count() / 1000;
    int milliseconds = timeout.count() % 1000;
    endTime.tv_sec += seconds;
    endTime.tv_nsec += milliseconds*1e6;
    while (availableToRead < length) {
        pthread_mutex_lock(&header->bufferMutex);
        int rc = pthread_cond_timedwait(&header->bufferCv, &header->bufferMutex, &endTime);
        availableToRead = header->size - header->remainingSpace;
        pthread_mutex_unlock(&header->bufferMutex);

        if (rc != 0) {
            return std::string();
        }

        if (!mmaped_) {
            return std::string();
        }
    }

    pthread_mutex_lock(&header->bufferMutex);
    uint32_t writePosition = header->writePosition;
    pthread_mutex_unlock(&header->bufferMutex);
    char* buf = new char[length];
    uint32_t newReadPosition = header->readPosition;
    if ((writePosition > header->readPosition) ||
        ((header->readPosition + length) < header->size)) {
            std::memcpy(buf, mmapPtr+header->readPosition, length);
            newReadPosition += length;
    } else {
        std::memcpy(buf, mmapPtr+header->readPosition, header->size - header->readPosition);
        std::memcpy(buf, mmapPtr, length - (header->size - header->readPosition));
        newReadPosition = length - (header->size - header->readPosition);
    }
    pthread_mutex_lock(&header->bufferMutex);
    header->readPosition = newReadPosition;
    header->remainingSpace += length;
    pthread_mutex_unlock(&header->bufferMutex);
    std::string retval(buf, length);
    delete[] buf;
    return retval;
}

bool SocketMmap::isPartnerActive() {
    if ((role_ == Roles::Client) &&
        firstBufferHeader_->alwaysListening && (firstBufferHeader_->status != Status::Closed)) {
            return true;
    }

    MmapBufferHeader* partnerHeader = role_ == Roles::Server ? secondBufferHeader_ :
        firstBufferHeader_;

    return (((partnerHeader->status == Status::Active) &&
        (timestamp() - partnerHeader->aliveTimestamp < PARTNER_TIMEOUT_MS)) ||
        (partnerHeader->status == Status::NotConnected));
}

bool SocketMmap::serverSetup() {
    if (access(name_.c_str(), F_OK) != -1) {
        /**
         * If the requested file already exists, this checks if it is actually in use
         * To be in use, the server side must be in Alive state and the timestamp updated
         */ 
        int fd = ::open(name_.c_str(), O_RDONLY, 0666);
        if (fd == -1) {
            logger_->error("Failed to open mmap file to check if server alive: {}",
                strerror(errno));
            return false;
        }

        struct stat buf;
        fstat(fd, &buf);
        if (static_cast<uint32_t>(buf.st_size) < sizeof(MmapBufferHeader)) {
            logger_->error("Requested file does not appear to be correct");
            return false;
        }

        void* mmap_tmp = mmap(
            NULL, sizeof(MmapBufferHeader), PROT_READ, MAP_SHARED, fd, 0);
        if (mmap_tmp == MAP_FAILED) {
            logger_->error("Failed to mmap file to check if server alive: {}",
                strerror(errno));
            return false;
        }
        MmapBufferHeader* serverHeaderTmp = reinterpret_cast<MmapBufferHeader*>(mmap_tmp);
        if ((serverHeaderTmp->status == Status::Active) &&
            (timestamp() - serverHeaderTmp->aliveTimestamp < PARTNER_TIMEOUT_MS)) {
                logger_->error("Requested mmap is busy");
                if (munmap(mmap_tmp, sizeof(MmapBufferHeader)) == -1) {
                    logger_->error("Failed to munmap: {}", strerror(errno));
                    return false;
                }
                if (::close(fd) == -1) {
                    logger_->error("Failed to close: {}", strerror(errno));
                    return false;
                }
                return false;
        }
        if (munmap(mmap_tmp, sizeof(MmapBufferHeader)) == -1) {
            logger_->error("Failed to munmap: {}", strerror(errno));
            return false;
        }
        if (::close(fd) == -1) {
            logger_->error("Failed to close: {}", strerror(errno));
            return false;
        }
    }

    mmapFd_ = ::open(name_.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0666);
    if (mmapFd_ == -1) {
        logger_->error("Failed to open mmap file: {}", strerror(errno));
        return false;
    }

    if (ftruncate(mmapFd_, totalMmapSize_) == -1) {
        logger_->error("Failed to ftruncate {}", strerror(errno));
        return false;
    }

    completeMmap_ = mmap(NULL, totalMmapSize_, PROT_READ | PROT_WRITE, MAP_SHARED, mmapFd_, 0);
    mmaped_ = true;
    if (completeMmap_ == MAP_FAILED) {
        logger_->error("Failed to mmap {}", strerror(errno));
        return false;
    }

    std::memset(completeMmap_, 0, totalMmapSize_);
    firstBufferHeader_ = reinterpret_cast<MmapBufferHeader*>(completeMmap_);
    secondBufferHeader_ = reinterpret_cast<MmapBufferHeader*>((char*)completeMmap_ + sizeof(MmapBufferHeader)); // NOLINT
    initHeaderStruct(firstBufferHeader_);
    initHeaderStruct(secondBufferHeader_);

    firstBufferHeader_->alwaysListening = keepListening_;

    firstBuffer_ = reinterpret_cast<uintptr_t*>(secondBufferHeader_ + sizeof(MmapBufferHeader)); // NOLINT
    secondBuffer_ = reinterpret_cast<uintptr_t*>(firstBuffer_ + bufferSize_); // NOLINT

    firstBufferHeader_->status = Status::Active;
    firstBufferHeader_->aliveTimestamp = timestamp();

    return true;
}

bool SocketMmap::clientSetup() {
    if (access(name_.c_str(), F_OK) == -1) {
        return false;
    }

    mmapFd_ = ::open(name_.c_str(), O_RDWR, 0666);
    if (mmapFd_ == -1) {
        return false;
    }

    completeMmap_ = mmap(NULL, totalMmapSize_, PROT_READ | PROT_WRITE, MAP_SHARED, mmapFd_, 0);
    mmaped_ = true;
    firstBufferHeader_ = reinterpret_cast<MmapBufferHeader*>(completeMmap_);
    secondBufferHeader_ = reinterpret_cast<MmapBufferHeader*>((char*)completeMmap_ + sizeof(MmapBufferHeader)); // NOLINT
    firstBuffer_ = reinterpret_cast<uintptr_t*>(secondBufferHeader_ + sizeof(MmapBufferHeader)); // NOLINT
    secondBuffer_ = reinterpret_cast<uintptr_t*>(firstBuffer_ + bufferSize_); // NOLINT

    if (!isPartnerActive()) {
        close();
        return false;
    }

    secondBufferHeader_->status = Status::Active;
    secondBufferHeader_->aliveTimestamp = timestamp();
    return true;
}

void SocketMmap::initHeaderStruct(MmapBufferHeader* ptr) {
    ptr->size = bufferSize_;
    ptr->readPosition = 0;
    ptr->writePosition = 0;
    ptr->remainingSpace = bufferSize_;
    ptr->status = Status::NotConnected;

    pthread_mutexattr_t mutex_attr;
    pthread_mutexattr_init(&mutex_attr);
    pthread_mutexattr_setpshared(&mutex_attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&ptr->bufferMutex, &mutex_attr);
    pthread_condattr_t cond_attr;
    pthread_condattr_init(&cond_attr);
    pthread_condattr_setpshared(&cond_attr, PTHREAD_PROCESS_SHARED);
    pthread_cond_init(&ptr->bufferCv, &cond_attr);
}

uint64_t SocketMmap::timestamp() {
    timeval tv;
    gettimeofday(&tv, NULL);
    return static_cast<uint64_t>(tv.tv_sec*1000 + tv.tv_usec/1000);
}

void SocketMmap::heartbeat() {
    while (!stopHeartbeat_) {
        if (role_ == Roles::Server) {
            firstBufferHeader_->aliveTimestamp = timestamp();
        } else {
            secondBufferHeader_->aliveTimestamp = timestamp();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void SocketMmap::autoresize(uint32_t reqWriteSize) { }

}  // namespace internal
}  // namespace sockets
}  // namespace communication
}  // namespace crf
