/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <memory>
#include <map>
#include <string>
#include <functional>
#include <atomic>

#include "DeviceManager/DeviceManagerCommunicationPoint/StatusStreamerCommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "DeviceManager/IDeviceManager.hpp"
#include "EventLogger/EventLogger.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"

namespace crf {
namespace utility {
namespace devicemanager {

StatusStreamerCommunicationPoint::StatusStreamerCommunicationPoint(
    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
    std::shared_ptr<crf::utility::devicemanager::IDeviceManager> manager) :
    logger_("StatusStreamerCommunicationPoint"),
    streamActive_(false),
    stopStream_(true),
    stopThreads_(true),
    initialized_(false),
    socket_(socket),
    manager_(manager),
    receiverThread_(),
    frequency_(),
    statusStreamerThread_(){
    packetHandlers_.insert({
        communication::datapackets::JSON_PACKET_TYPE,
        std::bind(&StatusStreamerCommunicationPoint::parseJSONPacket,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "getStatus",
        std::bind(&StatusStreamerCommunicationPoint::getStatusRequestHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "startStreamStatus",
        std::bind(&StatusStreamerCommunicationPoint::startStreamStatusRequestHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "stopStreamStatus",
        std::bind(&StatusStreamerCommunicationPoint::stopStreamStatusRequestHandler,
            this, std::placeholders::_1)});
}

StatusStreamerCommunicationPoint::~StatusStreamerCommunicationPoint() {
    logger_->debug("DTor");
    if (initialized_) {
        deinitialize();
    }
}

bool StatusStreamerCommunicationPoint::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->info("Already initialized");
        return true;
    }
    if (!socket_->isOpen()) {
        if (!socket_->open()) {
            logger_->error("Failed to open the socket");
            return false;
        }
    }
    stopThreads_ = false;
    receiverThread_ = std::thread(&StatusStreamerCommunicationPoint::receiver, this);
    initialized_ = true;
    return true;
}

bool StatusStreamerCommunicationPoint::deinitialize() {
    logger_->debug("deinitialize()");
    if (!initialized_) {
        logger_->info("Already deinitialized");
        return true;
    }
    stopThreads_ = true;
    stopStream_ = true;
    frequencyCV_.notify_one();
    if (socket_->isOpen()) {
        if (!socket_->close()) {
            logger_->error("Failed to close socket");
            return false;
        }
    }

    if (receiverThread_.joinable()) {
        receiverThread_.join();
    }

    if (statusStreamerThread_.joinable()) {
        statusStreamerThread_.join();
    }
    initialized_ = false;
    return true;
}

void StatusStreamerCommunicationPoint::getStatusRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("getStatusRequestHandler");
    communication::datapackets::JSONPacket responseJSON;
    responseJSON.data["command"] = "reply";
    responseJSON.data["replyCommand"] = "getStatus";
    responseJSON.data["message"] = manager_->getStatus();
    socket_->write(responseJSON, responseJSON.getHeader(), true);
    return;
}

void StatusStreamerCommunicationPoint::startStreamStatusRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("startStreamStatusRequestHandler");
    try {
        frequency_ = packet.data.at("frequency").get<float>();
        if (frequency_ <= 0) {
            sendJSONPacket<std::string>("error", "Frequency not valid", true);
            return;
        }
    } catch (const std::exception&) {
        sendJSONPacket<std::string>("error", "Wrong parameters", true);
        return;
    }
    if (!streamActive_) {
        stopStream_ = false;
        statusStreamerThread_ = std::thread(
            &StatusStreamerCommunicationPoint::statusStreamer, this);
    }
    return;
}

void StatusStreamerCommunicationPoint::stopStreamStatusRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("stopStreamStatusRequestHandler");
    if (!streamActive_) {
        sendJSONPacket<std::string>("error", "Stream was not active", true);
        return;
    }
    stopStream_ = true;
    frequencyCV_.notify_one();
    statusStreamerThread_.join();
    return;
}

void StatusStreamerCommunicationPoint::receiver() {
    logger_->debug("receiver()");
    std::string buffer;
    communication::datapackets::PacketHeader header;
    while (!stopThreads_) {
        if (!socket_->isOpen()) {
            stopThreads_ = true;
            stopStream_ = true;
            return;
        }

        auto message = socket_->read();
        if (!message) {
            continue;
        }

        auto header = message.get().second;
        auto search = packetHandlers_.find(header.type());
        if (search == packetHandlers_.end()) {
            sendJSONPacket<std::string>("error", "Not supported packet type", true);
            continue;
        }
        search->second(message.get().first);
    }
}

void StatusStreamerCommunicationPoint::statusStreamer() {
    logger_->debug("statusStreamer");
    streamActive_ = true;
    while (!stopStream_) {
        auto start = std::chrono::high_resolution_clock::now();

        communication::datapackets::JSONPacket responseJSON;
        responseJSON.data["command"] = "reply";
        responseJSON.data["replyCommand"] = "streamStatus";
        responseJSON.data["message"] = manager_->getStatus();
        socket_->write(responseJSON, responseJSON.getHeader(), true);

        auto end = std::chrono::high_resolution_clock::now();
        auto loopDuration = end-start;
        std::unique_lock<std::mutex> frequencyLck(frequencyMtx_);
        if (loopDuration < std::chrono::milliseconds(static_cast<int>(1000/frequency_))) {
            frequencyCV_.wait_for(
                frequencyLck,
                std::chrono::milliseconds(static_cast<int>(1000/frequency_)) - loopDuration);
        }
    }
    streamActive_ = false;
}

void StatusStreamerCommunicationPoint::parseJSONPacket(const std::string& buffer) {
    communication::datapackets::JSONPacket json;
    if (!json.deserialize(buffer)) {
        sendJSONPacket<std::string>("error", "Not valid json format", true);
        return;
    }
    std::string command;
    try {
        command = json.data.at("command").get<std::string>();
    } catch (const std::exception& e) {
        sendJSONPacket<std::string>("error", "Cannot get command field from received json", true);
        return;
    }
    if (jsonCommandHandlers_.find(command) == jsonCommandHandlers_.end()) {
        sendJSONPacket<std::string>("error", "Unknown command: " + command, true);
        return;
    }
    jsonCommandHandlers_[command](json);
    return;
}

}  // namespace devicemanager
}  // namespace utility
}  // namespace crf
