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
#include <netinet/tcp.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>

#include "Sockets/TCP/TCPSocket.hpp"

namespace crf {
namespace communication {
namespace sockets {

TCPSocket::TCPSocket(const std::string& hostname, int port) :
    logger_("TCPSocketClient"),
    socketFd_(0),
    role_(Roles::Client),
    isOpen_(false),
    isBlocking_(true),
    hostname_(hostname),
    port_(port) {
        logger_->debug("CTor");
    }

TCPSocket::TCPSocket(int socketFd, const std::string& hostname, int port)  :
    logger_("TCPSocketServer"),
    socketFd_(socketFd),
    role_(Roles::Server),
    isOpen_(true),
    isBlocking_(true),
    hostname_(hostname),
    port_(port) {
        logger_->debug("CTor");
    }

TCPSocket::~TCPSocket() {
    logger_->debug("DTor");
    ::shutdown(socketFd_, SHUT_RDWR);
    ::close(socketFd_);
}

bool TCPSocket::open()  {
    logger_->debug("open()");
    if (isOpen_) {
        logger_->warn("Already open");
        return false;
    }

    struct sockaddr_in servaddr;
    socketFd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socketFd_ < 0) {
        logger_->warn("Could not open socket: {}", strerror(errno));
        return false;
    }

    auto server = gethostbyname(hostname_.c_str());
    if (server == NULL) {
        logger_->warn("Could not get hostname: {}", strerror(errno));
        return false;
    }

    std::memset(reinterpret_cast<char*>(&servaddr), 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    bcopy(reinterpret_cast<char*>(server->h_addr),
        reinterpret_cast<char*>(&servaddr.sin_addr.s_addr),
        server->h_length);
    servaddr.sin_port = htons(port_);

    int one = 1;
    if (setsockopt(socketFd_, SOL_TCP, TCP_NODELAY, &one, sizeof(one)) != 0) {
        logger_->warn("Could not set TCP_NODELAY flag: {}", strerror(errno));
        return false;
    }

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

bool TCPSocket::close()  {
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

bool TCPSocket::isOpen()  {
    return isOpen_;
}

bool TCPSocket::write(const std::string& buffer, bool requiresAck)  {
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

std::string TCPSocket::read(int length) {
    if (!isOpen_) {
        return std::string();
    }

    if (!isBlocking_) {
        setNonBlockingFlag(false);
    }

    return blockingRead(length);
}

std::string TCPSocket::read(int length, const std::chrono::milliseconds& timeout) {
    if (!isOpen_) {
        return std::string();
    }

    if (isBlocking_) {
        setNonBlockingFlag(true);
    }
    return nonBlockingRead(length, timeout);
}

bool TCPSocket::setNonBlockingFlag(bool value) {
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

std::string TCPSocket::nonBlockingRead(int length, const std::chrono::milliseconds& timeout) {
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

std::string TCPSocket::blockingRead(int length) {
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

int TCPSocket::getSocketError() {
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
