#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Carlos Prados Sesmero && Alejandro Diaz Rosales CERN EN/SMM/MRO                                           ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "VisionUtility")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
                ${PCL_LIBRARIES}
                ${OpenCV_LIBRARIES}
)
set(INCLUDE_DIRS ./include
                 ${OpenCV_INCLUDE_DIRS}
                 ${EIGEN3_INDCLUDE_DIRS}
                 ${PCL_INCLUDE_DIRS}
)
set(SRC_FILES ./src/Image/ImageJSONConverter.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()
