/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string>
#include <sys/un.h>
#include <unistd.h>

#include "Sockets/IPC/UnixSocket.hpp"

namespace crf {
namespace communication {
namespace sockets {

UnixSocket::UnixSocket(const std::string& name) :
    logger_("UnixSocketClient"),
    socketFd_(0),
    role_(Roles::Client),
    isOpen_(false),
    isBlocking_(true),
    name_("/tmp/ipc_unixdomain/"+name) {
        logger_->debug("CTor");
    }

UnixSocket::UnixSocket(int socketFd, const std::string& name)  :
    logger_("UnixSocketServer"),
    socketFd_(socketFd),
    role_(Roles::Server),
    isOpen_(true),
    isBlocking_(true),
    name_(name) {
        logger_->debug("CTor");
    }

UnixSocket::~UnixSocket() {
    logger_->debug("DTor");
    ::shutdown(socketFd_, SHUT_RDWR);
    ::close(socketFd_);
}

bool UnixSocket::open()  {
    logger_->debug("open()");
    if (isOpen_) {
        logger_->warn("Already open");
        return false;
    }

    socketFd_ = socket(AF_UNIX, SOCK_STREAM, 0);
    if (socketFd_ < 0) {
        logger_->warn("Could not open socket: {}", strerror(errno));
        return false;
    }

    struct sockaddr_un servaddr;
    std::memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sun_family = AF_UNIX;
    strncpy(servaddr.sun_path, name_.c_str(), name_.length());

    int retval;
    do {
        retval = connect(socketFd_, reinterpret_cast<sockaddr*>(&servaddr), sizeof(servaddr));
        if (retval == -1 && errno != EINPROGRESS) {
            logger_->warn("Could not connect to server: {}", strerror(errno));
            return false;
        }
    } while (retval != 0);

    isOpen_ = true;
    return true;
}

bool UnixSocket::close()  {
    logger_->debug("close()");
    if (!isOpen_) {
        logger_->warn("Socket not open");
        return false;
    }

    getSocketError();
    if (::shutdown(socketFd_, SHUT_RDWR) < 0) {
        if (errno != ENOTCONN && errno != EINVAL)
            logger_->warn("Could not make shutdown: {}", strerror(errno));
    }
    if (::close(socketFd_) < 0)
        logger_->warn("Could not close socket: {}", strerror(errno));
    isOpen_ = false;
    return true;
}

bool UnixSocket::isOpen()  {
    return isOpen_;
}

bool UnixSocket::write(const std::string& buffer, bool requiresAck)  {
    if (!isOpen_) {
        return false;
    }

    size_t sent = 0;
    do {
        int retval = ::send(socketFd_, buffer.c_str()+sent, buffer.length()-sent, MSG_NOSIGNAL);
        if (retval == -1) {
            if (errno == EAGAIN) {
                continue;
            } else {
                close();
                return false;
            }
        }
        sent += static_cast<size_t>(retval);
    } while (sent < buffer.length());

    return true;
}

std::string UnixSocket::read(int length) {
    if (!isOpen_) {
        return std::string();
    }

    if (!isBlocking_) {
        setNonBlockingFlag(false);
    }

    return blockingRead(length);
}

std::string UnixSocket::read(int length, const std::chrono::milliseconds& timeout) {
    if (!isOpen_) {
        return std::string();
    }

    if (isBlocking_) {
        setNonBlockingFlag(true);
    }
    return nonBlockingRead(length, timeout);
}

bool UnixSocket::setNonBlockingFlag(bool value) {
    logger_->debug("Changing blocking flag to {}", value);
    if (value) {
        int flags = fcntl(socketFd_, F_GETFL);
        if (fcntl(socketFd_, F_SETFL, flags | O_NONBLOCK) != 0) {
            logger_->warn("Could not set O_NONBLOCK flag: {}", strerror(errno));
            return false;
        }
        isBlocking_ = false;
    } else {
        int flags = fcntl(socketFd_, F_GETFL);
        if (fcntl(socketFd_, F_SETFL, flags & (~O_NONBLOCK)) != 0) {
            logger_->warn("Could not reset O_NONBLOCK flag: {}", strerror(errno));
            return false;
        }
        isBlocking_ = true;
    }

    return true;
}

std::string UnixSocket::nonBlockingRead(int length, const std::chrono::milliseconds& timeout) {
    int received = 0;
    std::string buffer;

    if (timeout.count() == 0) {  // fully non blocking read
        char* cbuf = new char[length];
        int retval = ::recv(socketFd_, cbuf, length-received, MSG_DONTWAIT);
        if (retval != -1) {
            buffer = std::string(cbuf, retval);
        }
        delete[] cbuf;
        return buffer;
    }

    auto start = std::chrono::high_resolution_clock::now();
    do {
        auto now = std::chrono::high_resolution_clock::now();
        if ((now - start) > timeout) {
            break;
        }

        if (!isOpen_) {
            break;
        }

        char* cbuf = new char[length];
        int retval = ::recv(socketFd_, cbuf, length-received, MSG_DONTWAIT);
        if (retval == -1) {
            if (errno == EAGAIN) {
                delete[] cbuf;
                continue;
            } else {
                delete[] cbuf;
                break;
            }
        }

        if (retval == 0) {
            logger_->warn("Connection was lost with partner socket");
            delete[] cbuf;
            break;
        }

        buffer.append(cbuf, retval);
        received += retval;
        delete[] cbuf;
    } while (received < length);

    return buffer;
}

std::string UnixSocket::blockingRead(int length) {
    char* cbuf = new char[length];
    int retval = ::recv(socketFd_, cbuf, length, MSG_WAITALL);
    if (retval <= 0) {
        if (retval == -1)
            logger_->warn("Error during blocking read: {}", strerror(errno));
        delete[] cbuf;
        return std::string();
    }

    std::string buffer(cbuf, retval);
    delete[] cbuf;
    return buffer;
}

int UnixSocket::getSocketError() {
    int err = 1;
    socklen_t len = sizeof err;
    if (-1 == getsockopt(socketFd_, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&err), &len))
        logger_->error("Could not get socket error: {}", strerror(errno));
    if (err)
        errno = err;              // set errno to the socket SO_ERROR
    return err;
}

}  // namespace sockets
}  // namespace communication
}  // namespace crf
