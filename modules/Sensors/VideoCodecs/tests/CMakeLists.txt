#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(VIDEO_CODECS_TESTS_FILES main.cpp
)

if (NOT DISABLE_cvMatVideoCodec)
    set(VIDEO_CODECS_TESTS_FILES ${VIDEO_CODECS_TESTS_FILES}
                                 cvMatVideoEncoderTests.cpp
                                 cvMatVideoDecoderTests.cpp
    )
endif()

if (NOT DISABLE_HEVCVideoCodec)
    set(VIDEO_CODECS_TESTS_FILES ${VIDEO_CODECS_TESTS_FILES}
                                 HEVCVideoEncoderTests.cpp
    )
endif()

if (NOT DISABLE_JPEGVideoCodec)
    set(VIDEO_CODECS_TESTS_FILES ${VIDEO_CODECS_TESTS_FILES}
                                 JPEGVideoEncoderTests.cpp
                                 JPEGVideoDecoderTests.cpp
    )
endif()

if (NOT DISABLE_x264VideoCodec)
    set(VIDEO_CODECS_TESTS_FILES ${VIDEO_CODECS_TESTS_FILES}
                                 x264VideoEncoderTests.cpp
    )
endif()

add_executable(ut-videocodecs ${VIDEO_CODECS_TESTS_FILES})
target_link_libraries(ut-videocodecs gmock
                                     VideoCodecs
)

add_test(VideoCodecsUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-videocodecs)
