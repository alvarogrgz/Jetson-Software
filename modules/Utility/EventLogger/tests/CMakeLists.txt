#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(EVENT_LOGGER_TESTS_FILES main.cpp
                             EventLoggerTests.cpp
)

add_executable(ut-eventlogger ${EVENT_LOGGER_TESTS_FILES})
target_link_libraries(ut-eventlogger gmock
                                     EventLogger
)

add_test(EventLoggerUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-eventlogger)
