#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(DATAPACKETS_TESTS_FILES main.cpp
                            PacketHeaderTests.cpp
)

if (NOT DISABLE_JSONPacket)
    set(DATAPACKETS_TESTS_FILES ${DATAPACKETS_TESTS_FILES}
                                JSONPacketTests.cpp
    )
endif()

if (NOT DISABLE_RGBDFramePacket)
    set(DATAPACKETS_TESTS_FILES ${DATAPACKETS_TESTS_FILES}
                                RGBDFramePacketTests.cpp
    )
endif()

add_executable(ut-datapackets ${DATAPACKETS_TESTS_FILES})
target_link_libraries(ut-datapackets gmock
                                     EventLogger
                                     DataPackets
)

add_test(DataPacketsUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-datapackets)
