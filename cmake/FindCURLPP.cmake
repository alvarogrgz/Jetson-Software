# FindCURLPP.cmake
# The following standard variables get defined:
#  - CURLPP_FOUND:        TRUE if cURLpp is found.
#  - CURLPP_INCLUDE_DIRS: Include directories for cURLpp.
#  - CURLPP_LIBRARIES:    Libraries for all cURLpp component libraries and dependencies.

macro(CURLPP_REPORT_NOT_FOUND REASON_MSG)
    unset(CURLPP_FOUND)
    unset(CURLPP_INCLUDE_DIRS)
    unset(CURLPP_LIBRARIES)
    if(CURLPP_FIND_QUIETLY)
        message(STATUS "Failed to find cURLpp - " ${REASON_MSG} ${ARGN})
    elseif(CURLPP_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find cURLpp - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find cURLpp - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(CURLPP_REPORT_NOT_FOUND)

# First we set it. Later will be unset if not found, calling CURLPP_REPORT_NOT_FOUND
set(CURLPP_FOUND TRUE)

find_path(CURLPP_INCLUDE_DIRS NAMES cURLpp.hpp
                              PATHS /usr/include
                                    /usr/local/include
                              PATH_SUFFIXES curlpp
                              NO_DEFAULT_PATH)
if(NOT CURLPP_INCLUDE_DIRS OR NOT EXISTS ${CURLPP_INCLUDE_DIRS})
    CURLPP_REPORT_NOT_FOUND("Could not find cURLpp includes directory")
else()
    message(STATUS "cURLpp includes directory found: " ${CURLPP_INCLUDE_DIRS})
endif()

find_path(CURLPP_LIBRARIES_PATH NAMES libcurlpp.so
                              PATHS /usr/lib/x86_64-linux-gnu
                                    /usr/local/lib
                              NO_DEFAULT_PATH)
if(NOT CURLPP_LIBRARIES_PATH OR NOT EXISTS ${CURLPP_LIBRARIES_PATH})
    CURLPP_REPORT_NOT_FOUND("Could not find cURLpp libraries directory")
else()
    message(STATUS "cURLpp libraries directory found: " ${CURLPP_LIBRARIES_PATH})
endif()

include(FindPackageHandleStandardArgs)
if(CURLPP_FOUND)
    file(GLOB CURLPP_LIBRARIES ${CURLPP_LIBRARIES_PATH}/libcurlpp.so*)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(CURLPP DEFAULT_MSG CURLPP_INCLUDE_DIRS CURLPP_LIBRARIES)
endif()
