#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO                                                                    ##
##                                                                                                                   ##
#######################################################################################################################

set(DEVICE_MANAGER_TESTS_FILES main.cpp
)

if(NOT DISABLE_DeviceManagerCommunicationPoint)
    set(DEVICE_MANAGER_TESTS_FILES ${DEVICE_MANAGER_TESTS_FILES}
                                   ./DeviceManagerCommunicationPoint/StatusStreamerCommunicationPointTests.cpp
    )
endif()

if(NOT DISABLE_DeviceManagerWithAutoInitialization)
    set(DEVICE_MANAGER_TESTS_FILES ${DEVICE_MANAGER_TESTS_FILES}
                                   ./DeviceManagerWithAutoInitialization/DeviceManagerWithAutoInitializationTests.cpp
    )
endif()

if(NOT DISABLE_DeviceManagerWithPriorityAccess)
    set(DEVICE_MANAGER_TESTS_FILES ${DEVICE_MANAGER_TESTS_FILES}
                                   ./DeviceManagerWithPriorityAccess/DeviceManagerWithPriorityAccessTests.cpp
    )

    if(NOT DISABLE_DeviceManagerCommunicationPoint)
        set(DEVICE_MANAGER_TESTS_FILES ${DEVICE_MANAGER_TESTS_FILES}
                                       ./DeviceManagerCommunicationPoint/PriorityAccessCommunicationPointTests.cpp
        )
    endif()
endif()

add_executable(ut-devicemanager ${DEVICE_MANAGER_TESTS_FILES})
target_link_libraries(ut-devicemanager gmock
                                       DeviceManager)

add_test(DeviceManagerUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-devicemanager)
