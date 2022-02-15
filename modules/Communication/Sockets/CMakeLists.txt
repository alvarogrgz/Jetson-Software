#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Giacomo Lunghi CERN EN/SMM/MRO 2020                                                                       ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "Sockets")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES
)

if(NOT DISABLE_IPC)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/IPC/UnixSocketServer.cpp
                  ./src/IPC/UnixSocket.cpp
                  #./src/IPC/internal/SocketMmap.cpp
                  #./src/IPC/IPCServer.cpp
                  #./src/IPC/IPCSocket.cpp
    )
endif()

if(NOT DISABLE_TCP)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/TCP/TCPServer.cpp
                  ./src/TCP/TCPSocket.cpp
    )
endif()

if (DISABLE_IPC AND DISABLE_TCP)
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
