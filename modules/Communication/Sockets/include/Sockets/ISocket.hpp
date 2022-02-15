/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <chrono>
#include <string>

namespace crf {
namespace communication {
namespace sockets {
/**
 * @brief Generic socket communication interface
 * Generic socket communication interface.
 * All the implementations of this interface must respect the same behaviour.
 * This behaviour is defined in the SocketTests file in the test folder.
 * For a complete description of the various behaviours, please refer to the README.md file for this
 * module.
 */
class ISocket {
 public:
    /**
     * @brief Destroy the ISocket object
     */
    virtual ~ISocket() = default;

    /**
     * @brief Open the socket
     * @return true if the socket was correctly open
     * @return false otherwise
     */
    virtual bool open() = 0;
    /**
     * @brief Close the socket
     * The partner socket must receive a notification that this socket was closed at the next
     * read/write operation
     * @return true if the socket was correctly closed
     * @return false otherwise
     */
    virtual bool close() = 0;

    /**
     * @brief Return if the socket is open
     * 
     * @return true if is open
     * @return false otherwise
     */
    virtual bool isOpen() = 0;

    /**
     * @brief Writes a buffer on the socket.
     * Writes a buffer on the socket. The entire buffer is written.
     * @param requiresAck flag that allows to send unreliable messages if the transport protocol allows it.
     * @return true wrote the entire buffer
     * @return false otherwise
     */
    virtual bool write(const std::string& , bool requiresAck = true) = 0;

    /**
     * @brief Read the specified number of bytes from the socket
     * Returns an empty string if something went wrong.
     * This operation is fully blocking.
     * Closing the socket from another thread interrupts the read operation.
     * @param length number of bytes to read
     * @return std::string the read bytes
     */
    virtual std::string read(int length) = 0;

    /**
     * @brief Read the specified number of bytes from the socket with a timeout
     * Returns an empty string if something went wrong.
     * This operation is non-blocking and returns when the timeout expired.
     * If the timeout expired, the received bytes so far are returned.
     * Closing the socket from another thread interrupts the read operation
     * @param length number of bytes to read
     * @param timeout timeout
     * @return std::string the read bytes
     */
    virtual std::string read(int length, const std::chrono::milliseconds& timeout) = 0;
};

}  // namespace sockets
}  // namespace communication
}  // namespace crf
