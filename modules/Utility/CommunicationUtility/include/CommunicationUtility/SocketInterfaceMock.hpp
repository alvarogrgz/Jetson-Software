/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/STI/ 2017
 *
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>

#include "CommunicationUtility/ISocketInterface.hpp"

namespace crf {
namespace utility {
namespace commutility {

class SocketInterfaceMock : public ISocketInterface {
 public:
  MOCK_METHOD3(accept,
      int(int socket, struct sockaddr *address, socklen_t *address_len));
  MOCK_METHOD3(bind,
      int(int socket, const struct sockaddr *address, socklen_t address_len));
  MOCK_METHOD3(connect,
      int(int socket, const struct sockaddr *address, socklen_t address_len));
  MOCK_METHOD1(freeaddrinfo,
      void(struct addrinfo *res));
  MOCK_METHOD4(getaddrinfo,
      int(const char *node, const char *service, const struct addrinfo *hints,
        struct addrinfo **res));
  MOCK_METHOD3(getpeername,
      int(int socket, struct sockaddr *address, socklen_t *address_len));
  MOCK_METHOD3(getsockname,
      int(int socket, struct sockaddr *address, socklen_t *address_len));
  MOCK_METHOD5(getsockopt,
      int(int socket, int level, int option_name, void *option_value, socklen_t *option_len));
  MOCK_METHOD2(listen,
      int(int socket, int backlog));
  MOCK_METHOD4(recv,
      ssize_t(int socket, void *buffer, size_t length, int flags));
  MOCK_METHOD6(recvfrom,
      ssize_t(int socket, void *buffer, size_t length, int flags, struct sockaddr *address,
        socklen_t *address_len));
  MOCK_METHOD3(recvmsg,
      ssize_t(int socket, struct msghdr *message, int flags));
  MOCK_METHOD4(send,
      ssize_t(int socket, const void *message, size_t length, int flags));
  MOCK_METHOD3(sendmsg,
      ssize_t(int socket, const struct msghdr *message, int flags));
  MOCK_METHOD6(sendto,
      ssize_t(int socket, const void *message, size_t length, int flags,
        const struct sockaddr *dest_addr, socklen_t dest_len));
  MOCK_METHOD5(setsockopt,
      int(int socket, int level, int option_name, const void *option_value, socklen_t option_len));
  MOCK_METHOD2(shutdown,
      int(int socket, int how));
  MOCK_METHOD3(socket,
      int(int domain, int type, int protocol));
  MOCK_METHOD4(socketpair,
      int(int domain, int type, int protocol, int socket_vector[2]));
  MOCK_METHOD1(close,
      int(int socket));
};

}  // namespace commutility
}  // namespace utility
}  // namespace crf
