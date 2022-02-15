/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *         Jorge Playán Garai CERN BE/CEM/MRO 2021
 *  ==================================================================================================
 */

#include <memory>
#include <thread>
#include <chrono>

#include <nlohmann/json.hpp>

#include "DeviceManager/DeviceManagerWithAutoInitialization/DeviceManagerWithAutoInitialization.hpp"
#include "CommonInterfaces/IInitializable.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace utility {
namespace devicemanager {

DeviceManagerWithAutoInitialization::DeviceManagerWithAutoInitialization(
    const std::shared_ptr<crf::utility::commoninterfaces::IInitializable>& device,
    const std::chrono::milliseconds& initializationTimeout) :
    lastRequestTime_(),
    logger_("DeviceManagerWithAutoInitialization"),
    accessMutex_(),
    device_(device),
    initializationTimeout_(initializationTimeout),
    deviceInitialized_(false),
    checkLatestRequestThread_() {
    logger_->debug("CTor");
    if (device_ == nullptr) {
        throw std::runtime_error("Pointer to device is nullptr");
    }
    if (initializationTimeout_ == std::chrono::milliseconds(0)) {
        throw std::runtime_error("The initialization timeout is not valid");
    }
}

DeviceManagerWithAutoInitialization::~DeviceManagerWithAutoInitialization() {
    logger_->debug("DTor");
    if(isDeviceInitialized()) {
        deviceInitialized_ = false;
        initializationCV_.notify_one();
        if (checkLatestRequestThread_.joinable()) {
            checkLatestRequestThread_.join();
        }
        if (device_ != nullptr) {
            if (!device_->deinitialize()) {
                logger_->warn("Failed to deinitialize the device");
            }
        }
    }
}

bool DeviceManagerWithAutoInitialization::isDeviceInitialized(){
    return deviceInitialized_;
}

nlohmann::json DeviceManagerWithAutoInitialization::getStatus() {
    logger_->debug("getStatus");
    std::scoped_lock<std::mutex> lock(accessMutex_);
    nlohmann::json statusJSON;
    lastRequestTime_ = std::chrono::high_resolution_clock::now();

    if (initializeDevice()) {
        statusJSON["status"] = "initialize";
    } else {
        statusJSON["status"] = "deinitialize";
    }

    return statusJSON;
}

bool DeviceManagerWithAutoInitialization::initializeDevice() {
    if (deviceInitialized_) {
        return true;
    }

    if (!device_->initialize()) {
        logger_->error("Failed to initialize device");
        return false;
    }
    if (checkLatestRequestThread_.joinable()) {
        checkLatestRequestThread_.join();
    }
    
    deviceInitialized_ = true;
    checkLatestRequestThread_ = std::thread(
        &DeviceManagerWithAutoInitialization::checkLatestRequestTime, this);
    return true;
}

void DeviceManagerWithAutoInitialization::checkLatestRequestTime() {
    while (deviceInitialized_) {
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto timeSinceRequest = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime-lastRequestTime_);
        if (timeSinceRequest > initializationTimeout_) {
            std::scoped_lock<std::mutex> lock(accessMutex_);
            if (!device_->deinitialize()) {
                logger_->warn("Failed to deinitialize device - Keep trying");
            } else {
                deviceInitialized_ = false;
                logger_->info("Device deinitialized");
            }
        }

        /*
         * To avoid running this thread nonstop consuming resources we put a sleep ten times less
         * that the initializationTimeout_. Like this we ensure that if lastRequestTime_ does not
         * change there will be exactly 10 iterations of this loop before the call to deinitialize.
         * If during the sleep, lastRequestTime_ is updated we would de-synchronize and take longer
         * to deinitialize. That is why if timeSinceRequest_ is smaller than the standard sleep, we
         * instead sleep the time left for timeSinceRequest to reach timeinitializationTimeout_/10,
         * and in the following iterations we continue sleeping the standard time.
         * We use Conditional Variables to be able to stop the sleep when needed, for example in the
         * DTor
         */
        std::unique_lock<std::mutex> initializationLck(initializationMtx_);
        if (timeSinceRequest >= initializationTimeout_/10) {
            initializationCV_.wait_for(initializationLck, initializationTimeout_/10);
        } else {
            initializationCV_.wait_for(initializationLck, initializationTimeout_/10);
        }
    }
}

}  // namespace devicemanager
}  // namespace utility
}  // namespace crf
