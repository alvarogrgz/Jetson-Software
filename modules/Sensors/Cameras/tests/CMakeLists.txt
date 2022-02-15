#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(CAMERA_TESTS_FILES main.cpp
)

if (NOT DISABLE_CameraClient)
    set(CAMERA_TESTS_FILES ${CAMERA_TESTS_FILES}
                           ./CameraClientTests/CameraClientTests.cpp
    )
endif()

if (NOT DISABLE_CameraCommunicationPoint)
    set(CAMERA_TESTS_FILES ${CAMERA_TESTS_FILES}
                           ./CameraCommunicationPointTests/CameraCommunicationPointTests.cpp
                           ./CameraCommunicationPointTests/CameraManagerTests.cpp
    )
endif()

if (NOT DISABLE_CameraCommunicationPoint AND NOT DISABLE_CameraClient)
    set(CAMERA_TESTS_FILES ${CAMERA_TESTS_FILES}
                           ./CameraClientTests/CameraIntegrationTests.cpp
    )
endif()

add_executable(ut-cameras ${CAMERA_TESTS_FILES})
target_link_libraries(ut-cameras gmock
                                 Cameras
                                 EventLogger
)

add_test(CamerasUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-cameras)
