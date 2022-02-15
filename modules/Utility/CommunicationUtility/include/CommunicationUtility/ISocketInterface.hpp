/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO 2017
 *
 *  ==================================================================================================
 */

#pragma once

#include <sys/socket.h>
#include <netdb.h>

namespace crf {
namespace utility {
namespace communicationutility {

class ISocketInterface {
 public:
    virtual ~ISocketInterface() = default;

    virtual int accept(int socket, struct sockaddr *address, socklen_t *address_len) = 0;
    virtual int bind(int socket, const struct sockaddr *address, socklen_t address_len) = 0;
    virtual int connect(int socket, const struct sockaddr *address, socklen_t address_len) = 0;
    virtual void freeaddrinfo(struct addrinfo *res) = 0;
    virtual int getaddrinfo(const char *node, const char *service, const struct addrinfo *hints,
        struct addrinfo **res) = 0;
    virtual int getpeername(int socket, struct sockaddr *address, socklen_t *address_len) = 0;
    virtual int getsockname(int socket, struct sockaddr *address, socklen_t *address_len) = 0;
    virtual int getsockopt(int socket, int level, int option_name, void *option_value,
        socklen_t *option_len) = 0;
    virtual int listen(int socket, int backlog) = 0;
    virtual ssize_t recv(int socket, void *buffer, size_t length, int flags) = 0;
    virtual ssize_t recvfrom(int socket, void *buffer, size_t length, int flags,
        struct sockaddr *address, socklen_t *address_len) = 0;
    virtual ssize_t recvmsg(int socket, struct msghdr *message, int flags) = 0;
    virtual ssize_t send(int socket, const void *message, size_t length, int flags) = 0;
    virtual ssize_t sendmsg(int socket, const struct msghdr *message, int flags) = 0;
    virtual ssize_t sendto(int socket, const void *message, size_t length, int flags,
        const struct sockaddr *dest_addr, socklen_t dest_len) = 0;
    virtual int setsockopt(int socket, int level, int option_name, const void *option_value,
        socklen_t option_len) = 0;
    virtual int shutdown(int socket, int how) = 0;
    virtual int socket(int domain, int type, int protocol) = 0;
    virtual int socketpair(int domain, int type, int protocol, int socket_vector[2]) = 0;
    virtual int close(int socket) = 0;
};

}  // namespace communicationutility
}  // namespace utility
}  // namespace crf
