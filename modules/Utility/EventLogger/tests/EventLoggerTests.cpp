/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO 2017
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <array>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "EventLogger/EventLogger.hpp"

struct CustomStructForLoggerTesting {
    CustomStructForLoggerTesting() = default;
    CustomStructForLoggerTesting(int id, const std::string& msg): id_(id), msg_(msg) {}
    int id_;
    std::string msg_;
};

std::ostream& operator<<(std::ostream& stream, const CustomStructForLoggerTesting& val) {
    return stream << " [" << val.id_ << ": " << val.msg_ << "] ";
}

namespace {
std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
            result += buffer.data();
    }
    return result;
}
}  // namespace

class EventLoggerEventShould: public ::testing::Test {
 protected:
    EventLoggerEventShould() {
        std::ofstream fileCleaner(LOGGER_FILENAME, std::ios::out | std::ios::trunc);
        fileCleaner.close();
    }
};

/*
 * This test is rather a quick Logger usage tutorial, than an actual test. I do not have any good
 * idea for a test verifying logs consistency. After quick visual inspection of both log file and
 * stdout it looks that logger is good enough and thread safe.
 *
 * There are some problems with multiple threads accessing the log file when the project is linked
 * against FIFO.cpp and MMAP.cpp; This problem might have the same root cause that results in gtest
 * crashing (segmentation fault on gtest env exit: double free). I was not able to verify, which
 * part of the code in FIFO/MMAP is responsible for this behavior, though.
 *
 * Consequently, please neglect this test result, when the environment is crashing because of
 * FIFO/MMAP files.
 *
 * Please note that this test is only valid in the environment where "cat" and "grep" commands are
 * available.
 */

TEST_F(EventLoggerEventShould, printAsyncMsgsWithoutCollisions) {
    int numOfExpectedLines;
    if (std::getenv(LOGGER_ENABLE_DEBUG)) {
        numOfExpectedLines = 2006;
    } else {
        numOfExpectedLines = 1604;
    }

    CustomStructForLoggerTesting csflt(13, "Friday the 13th");
    crf::utility::logger::EventLogger logger1("MyLogger1");
    crf::utility::logger::EventLogger logger2("MyLogger2");

    logger1->info("Some info msg");
    logger2->info("Some info msg");
    logger1->debug("Some debug msg");
    logger2->debug("Some debug msg");

    auto loopPrintFunction = [](const crf::utility::logger::EventLogger& logger) {
        for (int i=0; i < 100; i++) {
            logger->critical("Loop critical msg number: {}", i);
            logger->error("Loop error msg number: {}", i);
            logger->warn("Loop warning msg number: {}", i);
            logger->info("Loop info msg number: {}", i);
            logger->debug("Loop debug msg number: {}", i);
        }
    };

    std::thread t1(loopPrintFunction, logger1);
    std::thread t2(loopPrintFunction, logger2);
    std::thread t3(loopPrintFunction, logger1);
    std::thread t4(loopPrintFunction, logger2);

    t1.join();
    t2.join();
    t3.join();
    t4.join();

    logger1->info("Printing my custom structure: {}", csflt);
    logger2->info("Printing my custom structure along with some number: {} ... {}", csflt, 69);

    logger1->flush();
    logger2->flush();

    std::string grepCommand;
    grepCommand += "cat ";
    grepCommand += LOGGER_FILENAME;
    grepCommand += " | grep . -c";

    ASSERT_EQ(numOfExpectedLines, std::stoi(exec(grepCommand.c_str())));
}
