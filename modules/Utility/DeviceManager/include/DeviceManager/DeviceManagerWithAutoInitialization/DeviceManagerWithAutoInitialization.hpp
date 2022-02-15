/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <thread>
#include <chrono>
#include <condition_variable>

#include <nlohmann/json.hpp>

#include "DeviceManager/IDeviceManager.hpp"
#include "CommonInterfaces/IInitializable.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace utility {
namespace devicemanager {

/*
 * @brief This thread safe device manager automatically deinitialized the device if no one has
 *        requested any information in a time equal to the initialization timeout.
 */
class DeviceManagerWithAutoInitialization : public IDeviceManager {
 public:
    DeviceManagerWithAutoInitialization(
        const std::shared_ptr<crf::utility::commoninterfaces::IInitializable>& device,
        const std::chrono::milliseconds& initializationTimeout = std::chrono::seconds(60));
    DeviceManagerWithAutoInitialization(const DeviceManagerWithAutoInitialization& other) = delete;
    DeviceManagerWithAutoInitialization(DeviceManagerWithAutoInitialization&& other) = delete;
    DeviceManagerWithAutoInitialization() = delete;
    ~DeviceManagerWithAutoInitialization();

    nlohmann::json getStatus() override;

 protected:
    std::chrono::high_resolution_clock::time_point lastRequestTime_;
    crf::utility::logger::EventLogger logger_;
    std::mutex accessMutex_;

    /*
     * @brief Getter access to deviceInitialized_ variable
     * @return True if the device is initialized, false otherwise.
     */
    bool isDeviceInitialized();

    /*
     * @brief Initializes the device if is not yet initialized and makes sure that the thread to
     *        deinitialize after timeout is running.
     * @return True if the initialization was successful.
     */
    bool initializeDevice();

 private:
    std::shared_ptr<crf::utility::commoninterfaces::IInitializable> device_;
    std::chrono::milliseconds initializationTimeout_;
    std::atomic<bool> deviceInitialized_;
    std::thread checkLatestRequestThread_;

    std::mutex initializationMtx_;
    std::condition_variable initializationCV_;

    /*
     * @brief Loop that deinitializes de device if the time that has past since the last request is
     *        bigger than the timeout.
     */
    void checkLatestRequestTime();
};

}  // namespace devicemanager
}  // namespace utility
}  // namespace crf
