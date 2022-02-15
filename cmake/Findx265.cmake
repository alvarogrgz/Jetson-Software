# Findx265.cmake
# The following standard variables get defined:
#  - x265_FOUND:        TRUE if x265 is found.
#  - x265_INCLUDE_DIRS: Include directories for x265.
#  - x265_LIBRARIES:    Libraries for all x265 component libraries and dependencies.

macro(x265_REPORT_NOT_FOUND REASON_MSG)
    unset(x265_FOUND)
    unset(x265_INCLUDE_DIRS)
    unset(x265_LIBRARIES)
    if(x265_FIND_QUIETLY)
        message(STATUS "Failed to find x265 - " ${REASON_MSG} ${ARGN})
    elseif(x265_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find x265 - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find x265 - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(x265_REPORT_NOT_FOUND)

# First we set it. Later will be unset if not found, calling x265_REPORT_NOT_FOUND
set(x265_FOUND TRUE)

find_path(x265_INCLUDE_DIRS NAMES x265.h
                            PATHS /usr/include
                                  /usr/local/include
                            NO_DEFAULT_PATH)
if(NOT x265_INCLUDE_DIRS OR NOT EXISTS ${x265_INCLUDE_DIRS})
    x265_REPORT_NOT_FOUND("Could not find x265 includes directory")
else()
    message(STATUS "x265 includes directory found: " ${x265_INCLUDE_DIRS})
endif()

find_path(x265_LIBRARIES_PATH NAMES libx265.so
                              PATHS /usr/lib/x86_64-linux-gnu/
                                    /usr/local/lib
                              NO_DEFAULT_PATH)
if(NOT x265_LIBRARIES_PATH OR NOT EXISTS ${x265_LIBRARIES_PATH})
    x265_REPORT_NOT_FOUND("Could not find x265 libraries directory")
else()
    message(STATUS "x265 libraries directory found: " ${x265_LIBRARIES_PATH})
endif()

include(FindPackageHandleStandardArgs)
if(x265_FOUND)
    #file(GLOB x265_LIBRARIES ${x265_LIBRARIES_PATH}/libx265.*)
    set(x265_LIBRARIES x265)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(x265 DEFAULT_MSG x265_INCLUDE_DIRS x265_LIBRARIES)
endif()
