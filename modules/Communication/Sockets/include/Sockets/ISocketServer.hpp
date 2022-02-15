/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <boost/optional.hpp>
#include <chrono>
#include <memory>
#include <string>

#include "Sockets/ISocket.hpp"

namespace crf {
namespace communication {
namespace sockets {

/**
 * @brief Generic socket server implementation
 * Generic socket server implementation.
 * All the implementations of this interface must respect the same behaviour.
 * This behaviour is defined in the SocketTests file in the test folder.
 * For a complete description of the various behaviours, please refer to the README.md file for this
 * module.
 */
class ISocketServer {
 public:
    /**
     * @brief Destroy the ISocketServer object
     */
    virtual ~ISocketServer() = default;

    /**
     * @brief Opens the server
     * @return true the server is correctly open
     * @return false otherwise
     */
    virtual bool open() = 0;
    /**
     * @brief Closes the server
     * Closing the server will automatically close all the accepted ISocket
     * @return true the server is correctly closed
     * @return false otherwise
     */
    virtual bool close() = 0;

    /**
     * @brief Returns if the server is open
     * @return true the server is open
     * @return false otherwise
     */
    virtual bool isOpen() = 0;

    /**
     * @brief Accepts an incoming connection
     * Accepts an incoming connection. The operation is fully blocking.
     * Closing the server will force this method to return
     * @return std::shared_ptr<ISocket> if an incoming connection was correctly accepted. The socket is already open
     * @return boost::none otherwise
     */
    virtual boost::optional<std::shared_ptr<ISocket> > acceptConnection() = 0;
};

}  // namespace sockets
}  // namespace communication
}  // namespace crf
