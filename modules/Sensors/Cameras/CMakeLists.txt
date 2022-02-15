#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Giacomo Lunghi CERN EN/SMM/MRO 2020                                                                       ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "Cameras")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS CommonInterfaces
                ${OpenCV_LIBRARIES}
                ${Boost_LIBRARIES}
                ${nlohmann_json_LIBRARIES}
)
set(INCLUDE_DIRS ./include
                 ${OpenCV_INCLUDE_DIRS}
                 ${Boost_INCLUDE_DIRS}
                 ${nlohmann_json_INCLUDE_DIRS}
)
set(SRC_FILES
)

if(NOT DISABLE_AxisCamera)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    ${CURLPP_LIBRARIES}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${CURLPP_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/AxisCamera/AxisCamera.cpp
    )
endif()

if(NOT DISABLE_CameraClient)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    DataPackets
                    DataPacketSocket
                    VideoCodecs
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/CameraClient/CameraClient.cpp
    )
endif()

if(NOT DISABLE_CameraCommunicationPoint)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    DataPackets
                    DataPacketSocket
                    CommunicationPointServer
                    VideoCodecs
                    VisionUtility
                    DeviceManager
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/CameraCommunicationPoint/CameraCommunicationPoint.cpp
                  ./src/CameraCommunicationPoint/CameraCommunicationPointFactory.cpp
                  ./src/CameraCommunicationPoint/CameraManager.cpp
    )
endif()

if(NOT DISABLE_UVCCamera)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/UVCCamera/UVCCamera.cpp
    )
endif()

if (DISABLE_AxisCamera AND DISABLE_CameraClient AND DISABLE_CameraCommunicationPoint AND DISABLE_UVCCamera)
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

# Optional packages for the different samples
find_package(Boost COMPONENTS program_options)

if(NOT DISABLE_CameraCommunicationPoint)
    if(NOT DISABLE_AxisCamera)
        add_executable(AxisCameraPoint ./samples/AxisCameraPoint.cpp)
        target_link_libraries(AxisCameraPoint ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()

    if(NOT DISABLE_UVCCamera)
        add_executable(UVCCameraPoint ./samples/UVCCameraPoint.cpp)
        target_link_libraries(UVCCameraPoint ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()
endif()

if(NOT DISABLE_CameraClient)
    add_executable(CameraClientSample ./samples/CameraClientSample.cpp)
    target_link_libraries(CameraClientSample ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})
endif()
