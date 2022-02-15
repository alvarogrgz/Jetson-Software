/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO 2017
 * Contributors: Alejandro Diaz Rosales CERN EN/SMM/MRO 2020
 *               Carlos Prados Sesmero CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

#include "EventLogger/EventLogger.hpp"

#define LOGGER_MAX_FILE_SIZE 1024*1024*64  // Using 64MB log files
#define LOGGER_MAX_NUMBER_OF_FILES 10  // Let's say that 10 files (64MB each) is enough

namespace crf {
namespace utility {
namespace logger {

LoggerParameterInitializer::LoggerParameterInitializer() {
    if (!std::getenv(LOGGER_ENABLE_LOGGING)) {
        spdlog::set_level(spdlog::level::off);
        return;
    }
    if (std::getenv(LOGGER_ENABLE_DEBUG)) {
        spdlog::set_level(spdlog::level::debug);
    } else {
        spdlog::set_level(spdlog::level::info);
    }
    spdlog::set_pattern("%^[%Y-%m-%d %H:%M:%S.%e] <PID:%P> <Thread:%t> [%l] [%n] : %v%$");
}


EventLogger::EventLogger(const std::string& name) {
    logger_ = spdlog::get(name);
    if (logger_) return;

    if (std::getenv(LOGGER_PRINT_STDOUT)) {
        logger_ = spdlog::stdout_color_mt(name);
    } else {
        logger_ = std::make_shared<spdlog::logger>(name, fileSink_);
        spdlog::initialize_logger(logger_);
    }
}

std::shared_ptr<spdlog::logger> EventLogger::operator->() const {
    return logger_;
}

const std::shared_ptr<spdlog::sinks::rotating_file_sink_mt> EventLogger::fileSink_
    = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(LOGGER_FILENAME,
        LOGGER_MAX_FILE_SIZE, LOGGER_MAX_NUMBER_OF_FILES);

const LoggerParameterInitializer EventLogger::initializer_ = LoggerParameterInitializer();

}  // namespace logger
}  // namespace utility
}  // namespace crf
