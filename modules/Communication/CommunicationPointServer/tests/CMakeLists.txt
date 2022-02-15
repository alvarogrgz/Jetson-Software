#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(COMMUNICATIONPOINTSERVER_TESTS_FILES main.cpp
                                         CommunicationPointServerTests.cpp
)

add_executable(ut-communicationpointserver ${COMMUNICATIONPOINTSERVER_TESTS_FILES})
target_link_libraries(ut-communicationpointserver gmock
                                                  CommunicationPointServer
)

add_test(CommunicationPointServerUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-communicationpointserver)
