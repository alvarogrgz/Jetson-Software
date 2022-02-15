# Find.cmake
# The following standard variables get defined:
#  - LZ4_FOUND:           TRUE if LZ4 is found.
#  - LZ4_INCLUDE_DIRS:    Include directories for LZ4.
#  - LZ4_LIBRARIES:       Libraries for all LZ4 component libraries and dependencies.

macro(LZ4_REPORT_NOT_FOUND REASON_MSG)
    unset(LZ4_FOUND)
    unset(LZ4_INCLUDE_DIRS)
    unset(LZ4_LIBRARIES)
    if(LZ4_FIND_QUIETLY)
        message(STATUS "Failed to find LZ4 - " ${REASON_MSG} ${ARGN})
    elseif(LZ4_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find LZ4 - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find LZ4 - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(LZ4_REPORT_NOT_FOUND)

# First we set it. Later will be unset if not found, calling LZ4_REPORT_NOT_FOUND
set(LZ4_FOUND TRUE)

find_path(LZ4_INCLUDE_DIRS NAMES lz4.h
                           DOC "LZ4 include directory"
                           PATHS /usr/local/include
                           NO_DEFAULT_PATH)
if(NOT LZ4_INCLUDE_DIRS OR NOT EXISTS ${LZ4_INCLUDE_DIRS})
    LZ4_REPORT_NOT_FOUND("Could not find LZ4 includes directory")
else()
    message(STATUS "LZ4 includes directory found: " ${LZ4_INCLUDE_DIRS})
endif()

find_path(LZ4_LIBRARIES_PATH NAMES lz4 liblz4 liblz4.so liblz4_static
                             DOC "LZ4 library"
                             PATHS /usr/local/lib
                             NO_DEFAULT_PATH)
if(NOT LZ4_LIBRARIES_PATH OR NOT EXISTS ${LZ4_LIBRARIES_PATH})
    LZ4_REPORT_NOT_FOUND("Could not find LZ4 libraries directory")
else()
    message(STATUS "LZ4 libraries directory found: " ${LZ4_LIBRARIES_PATH})
endif()

include(FindPackageHandleStandardArgs)
if(LZ4_FOUND)
    set(LZ4_LIBRARIES lz4)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(LZ4 DEFAULT_MSG LZ4_INCLUDE_DIRS LZ4_LIBRARIES)
endif()

mark_as_advanced(LZ4_INCLUDE_DIRS LZ4_LIBRARIES LZ4_LIBRARIES_PATH)
