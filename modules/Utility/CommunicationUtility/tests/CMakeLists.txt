#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(COMMUNICATION_UTILITY_TESTS_FILES main.cpp
                             StreamReaderWriterTests.cpp)

add_executable(ut-communicationutility ${COMMUNICATION_UTILITY_TESTS_FILES})
target_link_libraries(ut-communicationutility gmock
                                              CommunicationUtility
                                              EventLogger
)

add_test(CommUtilityTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-communicationutility)
