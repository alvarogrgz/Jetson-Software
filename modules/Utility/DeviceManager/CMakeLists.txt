#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "DeviceManager")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS ${nlohmann_json_LIBRARIES}
)
set(INCLUDE_DIRS ./include
                 ${nlohmann_json_INCLUDE_DIRS}
)
set(SRC_FILES
)

if(NOT DISABLE_DeviceManagerCommunicationPoint)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    Sockets
                    DataPackets
                    DataPacketSocket
                    CommunicationPointServer
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/DeviceManagerCommunicationPoint/StatusStreamerCommunicationPoint.cpp
    )
endif()

if(NOT DISABLE_DeviceManagerWithAutoInitialization)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    CommonInterfaces
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/DeviceManagerWithAutoInitialization/DeviceManagerWithAutoInitialization.cpp
    )
endif()

if(NOT DISABLE_DeviceManagerWithPriorityAccess)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    CommonInterfaces
                    ComponentAccessControl
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/DeviceManagerWithPriorityAccess/DeviceManagerWithPriorityAccess.cpp
    )

    if(NOT DISABLE_DeviceManagerCommunicationPoint)
        set(SRC_FILES ${SRC_FILES}
                      ./src/DeviceManagerCommunicationPoint/PriorityAccessCommunicationPoint.cpp
        )
    endif()
endif()

if (DISABLE_DeviceManagerCommunicationPoint AND DISABLE_DeviceManagerWithAutoInitialization AND DISABLE_DeviceManagerWithPriorityAccess)
    add_library(${MODULE_NAME} INTERFACE)
    target_include_directories(${MODULE_NAME} INTERFACE ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} INTERFACE)
else()
    add_library(${MODULE_NAME} SHARED ${SRC_FILES})
    target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

    target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)
endif()

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()
