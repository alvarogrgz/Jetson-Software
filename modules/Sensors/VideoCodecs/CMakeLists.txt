#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Giacomo Lunghi CERN EN/SMM/MRO 2019                                                                       ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "VideoCodecs")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS
)
set(INCLUDE_DIRS ./include
                 ${OpenCV_INCLUDE_DIRS}
)
set(SRC_FILES
)

if(NOT DISABLE_cvMatVideoCodec)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    CommunicationUtility
                    ${OpenCV_LIBRARIES}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/cvMatVideoCodec/cvMatVideoEncoder.cpp
                  ./src/cvMatVideoCodec/cvMatVideoDecoder.cpp
    )
endif()

if(NOT DISABLE_HEVCVideoCodec)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    ${OpenCV_LIBRARIES}
                    ${x265_LIBRARIES}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${x265_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/HEVCVideoCodec/HEVCVideoEncoder.cpp
    )
endif()

if(NOT DISABLE_JPEGVideoCodec)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    CommunicationUtility
                    ${OpenCV_LIBRARIES}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/JPEGVideoCodec/JPEGVideoEncoder.cpp
                  ./src/JPEGVideoCodec/JPEGVideoDecoder.cpp
    )
endif()

if(NOT DISABLE_x264VideoCodec)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    ${OpenCV_LIBRARIES}
                    ${x264_LIBRARIES}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${x264_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/x264VideoCodec/x264VideoEncoder.cpp
    )
endif()

if (DISABLE_cvMatVideoCodec AND DISABLE_HEVCVideoCodec AND DISABLE_JPEGVideoCodec AND DISABLE_x264VideoCodec)
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
