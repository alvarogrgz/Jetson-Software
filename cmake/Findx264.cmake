# Findx264.cmake
# The following standard variables get defined:
#  - x264_FOUND:        TRUE if x264 is found.
#  - x264_INCLUDE_DIRS: Include directories for x264.
#  - x264_LIBRARIES:    Libraries for all x264 component libraries and dependencies.

macro(x264_REPORT_NOT_FOUND REASON_MSG)
    unset(x264_FOUND)
    unset(x264_INCLUDE_DIRS)
    unset(x264_LIBRARIES)
    if(x264_FIND_QUIETLY)
        message(STATUS "Failed to find x264 - " ${REASON_MSG} ${ARGN})
    elseif(x264_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find x264 - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find x264 - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(x264_REPORT_NOT_FOUND)

# First we set it. Later will be unset if not found, calling x264_REPORT_NOT_FOUND
set(x264_FOUND TRUE)

find_path(x264_INCLUDE_DIRS NAMES x264.h
                            PATHS /usr/local/include
                            NO_DEFAULT_PATH)
if(NOT x264_INCLUDE_DIRS OR NOT EXISTS ${x264_INCLUDE_DIRS})
    x264_REPORT_NOT_FOUND("Could not find x264 includes directory")
else()
    message(STATUS "x264 includes directory found: " ${x264_INCLUDE_DIRS})
endif()

find_path(x264_LIBRARIES_PATH NAMES libx264.so
                              PATHS /usr/local/lib
                              NO_DEFAULT_PATH)
if(NOT x264_LIBRARIES_PATH OR NOT EXISTS ${x264_LIBRARIES_PATH})
    x264_REPORT_NOT_FOUND("Could not find x264 libraries directory")
else()
    message(STATUS "x264 libraries directory found: " ${x264_LIBRARIES_PATH})
endif()

include(FindPackageHandleStandardArgs)
if(x264_FOUND)
    file(GLOB x264_LIBRARIES ${x264_LIBRARIES_PATH}/libx264.*)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(x264 DEFAULT_MSG x264_INCLUDE_DIRS x264_LIBRARIES)
endif()
