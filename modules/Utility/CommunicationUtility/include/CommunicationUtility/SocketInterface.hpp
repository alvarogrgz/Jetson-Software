/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO 2017
 *
 *  ==================================================================================================
 */

#pragma once

#include <sys/socket.h>

#include "CommunicationUtility/ISocketInterface.hpp"

namespace crf {
namespace utility {
namespace communicationutility {

class SocketInterface: public ISocketInterface {
 public:
    ~SocketInterface() override = default;

    int accept(int socket, struct sockaddr *address, socklen_t *address_len) override;
    int bind(int socket, const struct sockaddr *address, socklen_t address_len) override;
    int connect(int socket, const struct sockaddr *address, socklen_t address_len) override;
    void freeaddrinfo(struct addrinfo *res) override;
    int getaddrinfo(const char *node, const char *service, const struct addrinfo *hints,
        struct addrinfo **res) override;
    int getpeername(int socket, struct sockaddr *address, socklen_t *address_len) override;
    int getsockname(int socket, struct sockaddr *address, socklen_t *address_len) override;
    int getsockopt(int socket, int level, int option_name, void *option_value,
        socklen_t *option_len) override;
    int listen(int socket, int backlog) override;
    ssize_t recv(int socket, void *buffer, size_t length, int flags) override;
    ssize_t recvfrom(int socket, void *buffer, size_t length, int flags,
        struct sockaddr *address, socklen_t *address_len) override;
    ssize_t recvmsg(int socket, struct msghdr *message, int flags) override;
    ssize_t send(int socket, const void *message, size_t length, int flags) override;
    ssize_t sendmsg(int socket, const struct msghdr *message, int flags) override;
    ssize_t sendto(int socket, const void *message, size_t length, int flags,
        const struct sockaddr *dest_addr, socklen_t dest_len) override;
    int setsockopt(int socket, int level, int option_name, const void *option_value,
        socklen_t option_len) override;
    int shutdown(int socket, int how) override;
    int socket(int domain, int type, int protocol) override;
    int socketpair(int domain, int type, int protocol, int socket_vector[2]) override;
    int close(int socket) override;
};

}  // namespace communicationutility
}  // namespace utility
}  // namespace crf
