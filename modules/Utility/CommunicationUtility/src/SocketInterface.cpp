/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO 2017
 *
 *  ==================================================================================================
 */

#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>

#include "CommunicationUtility/SocketInterface.hpp"

namespace crf {
namespace utility {
namespace communicationutility {

int SocketInterface::accept(int socket, struct sockaddr *address, socklen_t *address_len) {
    return ::accept(socket, address, address_len);
}

int SocketInterface::bind(int socket, const struct sockaddr *address, socklen_t address_len) {
    return ::bind(socket, address, address_len);
}

int SocketInterface::connect(int socket, const struct sockaddr *address, socklen_t address_len) {
    return ::connect(socket, address, address_len);
}

void SocketInterface::freeaddrinfo(struct addrinfo *res) {
    return ::freeaddrinfo(res);
}

int SocketInterface::getaddrinfo(const char *node, const char *service,
    const struct addrinfo *hints, struct addrinfo **res) {
    return ::getaddrinfo(node, service, hints, res);
}

int SocketInterface::getpeername(int socket, struct sockaddr *address, socklen_t *address_len) {
    return ::getpeername(socket, address, address_len);
}

int SocketInterface::getsockname(int socket, struct sockaddr *address, socklen_t *address_len) {
    return ::getsockname(socket, address, address_len);
}

int SocketInterface::getsockopt(int socket, int level, int option_name, void *option_value,
    socklen_t *option_len) {
    return ::getsockopt(socket, level, option_name, option_value, option_len);
}

int SocketInterface::listen(int socket, int backlog) {
    return ::listen(socket, backlog);
}

ssize_t SocketInterface::recv(int socket, void *buffer, size_t length, int flags) {
    return ::recv(socket, buffer, length, flags);
}

ssize_t SocketInterface::recvfrom(int socket, void *buffer, size_t length, int flags,
    struct sockaddr *address, socklen_t *address_len) {
    return ::recvfrom(socket, buffer, length, flags, address, address_len);
}

ssize_t SocketInterface::recvmsg(int socket, struct msghdr *message, int flags) {
    return ::recvmsg(socket, message, flags);
}

ssize_t SocketInterface::send(int socket, const void *message, size_t length, int flags) {
    return ::send(socket, message, length, flags);
}

ssize_t SocketInterface::sendmsg(int socket, const struct msghdr *message, int flags) {
    return ::sendmsg(socket, message, flags);
}

ssize_t SocketInterface::sendto(int socket, const void *message, size_t length, int flags,
    const struct sockaddr *dest_addr, socklen_t dest_len) {
    return ::sendto(socket, message, length, flags, dest_addr, dest_len);
}

int SocketInterface::setsockopt(int socket, int level, int option_name, const void *option_value,
    socklen_t option_len) {
    return ::setsockopt(socket, level, option_name, option_value, option_len);
}

int SocketInterface::shutdown(int socket, int how) {
    return ::shutdown(socket, how);
}

int SocketInterface::socket(int domain, int type, int protocol) {
    return ::socket(domain, type, protocol);
}

int SocketInterface::socketpair(int domain, int type, int protocol, int socket_vector[2]) {
    return ::socketpair(domain, type, protocol, socket_vector);
}

int SocketInterface::close(int socket) {
    return ::close(socket);
}

}  // namespace communicationutility
}  // namespace utility
}  // namespace crf
