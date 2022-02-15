#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Pawel Ptasznik CERN EN/SMM/MRO 2017                                                                       ##
## Contributors: Alejandro Diaz Rosales CERN EN/SMM/MRO 2020                                                         ##
##               Carlos Prados Sesmero CERN EN/SMM/MRO 2020                                                          ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "EventLogger")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS spdlog::spdlog
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES ./src/EventLogger.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()
