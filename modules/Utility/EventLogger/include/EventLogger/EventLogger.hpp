/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO 2017
 * Contributors: Alejandro Diaz Rosales CERN EN/SMM/MRO 2020
 *               Carlos Prados Sesmero CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <iostream>
#include <memory>
#include <string>

#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/spdlog.h>

#define LOGGER_FILENAME "RoboticFrameworkLog.log"
#define LOGGER_ENABLE_LOGGING "LOGGER_ENABLE_LOGGING"  // Set the env. variable to enable the logs.
#define LOGGER_PRINT_STDOUT "LOGGER_PRINT_STDOUT"  // Set the env. variable to print on STDOUT.
#define LOGGER_ENABLE_DEBUG "LOGGER_ENABLE_DEBUG"  // Set the env. variable to enable debug logs.

namespace crf {
namespace utility {
namespace logger {

struct LoggerParameterInitializer {
    LoggerParameterInitializer();
};

/*
 * @brief Wrapper class of the spdlog library, a very fast C++ logging library. For instructions
 *        please see LoggerTests.cpp file in the Tests folders.
 */
class EventLogger {
 public:
    EventLogger() = delete;
    EventLogger(const EventLogger& l) = default;
    explicit EventLogger(const std::string& name);
    /*
     * @brief operator that prints a colored message into the screen with 5 different levels: info,
     *        debug, warning, error, critical.
     */
    std::shared_ptr<spdlog::logger> operator->() const;

 private:
    static const std::shared_ptr<spdlog::sinks::rotating_file_sink_mt> fileSink_;
    static const LoggerParameterInitializer initializer_;
    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace logger
}  // namespace utility
}  // namespace crf
