#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(SOCKETS_TESTS_FILES main.cpp
)

if (NOT DISABLE_TCP AND NOT DISABLE_IPC)
    set(SOCKETS_TESTS_FILES ${SOCKETS_TESTS_FILES}
                            #SocketMmapTests.cpp
                            SocketsTests.cpp
    )
endif()

add_executable(ut-sockets ${SOCKETS_TESTS_FILES})
target_link_libraries(ut-sockets gmock
                                 Sockets
)

add_test(SocketsUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-sockets)
