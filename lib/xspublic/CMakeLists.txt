# Copyright 2023 Haul Vision

# This file replaces Makefiles provided by the original authors to build three libraries:
# xscommon, xscontroller, and xstypes.

# Adjusted and based on the other forked repo:
# https://github.com/DEMCON/ros2_xsens_mti_driver/blob/main/lib/xspublic/CMakeLists.txt

## XS COMMON
# Gather a list of source files in the xscommon directory with specific file extensions
file(
        GLOB XSCOMMON_SRC_FILES
        xscommon/*.c xscommon/*.cpp xscommon/*.h
)

# Add a static library target named xscommon
add_library(
        xscommon
        STATIC
        ${XSCOMMON_SRC_FILES}
)

# The target_compile_options command is used to specify compilation options for the xscommon target.
# Note: Use 'SHELL:' prefix to prevent option de-duplication here. This prefix is used to keep the duplicated items.
target_compile_options(
        xscommon
        PRIVATE
        "SHELL:-include xstypes/xsens_compat.h"
        "SHELL:-include xscommon_config.h"
)

# It sets the C++ standard to 14 and enables position-independent code.
# Note: When code is position-independent, it is designed in a way that it can be loaded into memory and executed
# regardless of the specific memory address assigned to it. It gives us some performance benefits by sharing the
# same code among processes.
set_target_properties(
        xscommon
        PROPERTIES
        CXX_STANDARD 14
        POSITION_INDEPENDENT_CODE ON
)

# Specify the include directories for the xscommon target.
target_include_directories(
        xscommon
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        xscommon
)

## XS CONTROLLER

file(
        GLOB XSCONTROLLER_SRC_FILES
        xscontroller/*.c xscontroller/*.cpp xscontroller/*.h
)

add_library(
        xscontroller
        STATIC
        ${XSCONTROLLER_SRC_FILES}
)

target_compile_options(
        xscontroller
        PRIVATE
        "SHELL:-include xscontrollerconfig.h"
)

set_target_properties(
        xscontroller
        PROPERTIES
        CXX_STANDARD 14
        POSITION_INDEPENDENT_CODE ON
)

target_include_directories(
        xscontroller
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        xscontroller
)

# XS TYPES

file(
        GLOB SRC_FILES
        xstypes/*.c xstypes/*.cpp xstypes/*.h
)

list(REMOVE_ITEM ${SRC_FILES} xstypes/xstypesdynlib.c)

add_library(
        xstypes
        STATIC
        ${SRC_FILES}
)

set_target_properties(
        xstypes
        PROPERTIES
        CXX_STANDARD 14
        POSITION_INDEPENDENT_CODE ON
)

target_include_directories(
        xstypes
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)