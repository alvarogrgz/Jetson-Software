#pragma once
/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <string>

#include "Sockets/ISocket.hpp"

namespace crf {
namespace communication {
namespace sockets {

class SocketMock : public ISocket {
 public:
  MOCK_METHOD0(open,
      bool());
  MOCK_METHOD0(close,
      bool());
  MOCK_METHOD0(isOpen,
      bool());
  MOCK_METHOD2(write,
      bool(const std::string&, bool));
  MOCK_METHOD1(read,
      std::string(int length));
  MOCK_METHOD2(read,
      std::string(int length, const std::chrono::milliseconds& timeout));
};

}  // namespace sockets
}  // namespace communication
}  // namespace crf
