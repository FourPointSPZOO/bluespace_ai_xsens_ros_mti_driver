# Copyright 2023 Haul Vision

cmake_minimum_required(VERSION 3.8)
project(ros2_xsens_mti_driver)

###############
## Configure ##
###############

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

# Add a directory with headers
include_directories(include)
include_directories(SYSTEM /opt/ros/$ENV{ROS_DISTRO}/include) # IntelliJ can't see rclcpp without that

# Set helper constants
set(DEPENDENCIES
        rclcpp
        rclcpp_components
        tf2
        tf2_ros
        std_msgs
        geometry_msgs
        sensor_msgs)

set(SOURCES
        src/xdainterface.cpp
        src/xdacallback.cpp
        )

file(GLOB MESSAGEPUBLISHERS_INCLUDES
        include/ros2_xsens_mti_driver/messagepublishers/*.h)

set(INCLUDES
        include/ros2_xsens_mti_driver/xdainterface.h
        include/ros2_xsens_mti_driver/xdacallback.h
        ${MESSAGEPUBLISHERS_INCLUDES}
        )

set(XSPUBLIC_LIBS
        xscontroller
        xscommon
        xstypes
        )

# linking dlopen: https://stackoverflow.com/questions/33678965/need-to-link-cmake-project-to-dl-library
# linking pthreads: https://stackoverflow.com/questions/1620918/cmake-and-libpthread
set(THREADS_PREFER_PTHREAD_FLAG ON)
set(THIRD_PARTY_LIBS
        Threads::Threads
        ${CMAKE_DL_LIBS}
        )

###########
## Build ##
###########

set(EXEC_NAME xsens_mti_node)
set(LIB_NAME xsens_mti_node_component)

### Automatically build the xspublic in a separate CMakeLists.txt
add_subdirectory(lib/xspublic)

### Create a shared library from the specified files
add_library(${LIB_NAME} SHARED
        ${INCLUDES}
        ${SOURCES}
        )

target_link_libraries(${LIB_NAME}
        ${XSPUBLIC_LIBS}
        ${THIRD_PARTY_LIBS}
        )

target_include_directories(${LIB_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${LIB_NAME}
        ${DEPENDENCIES}
        )

# required gJournal symbol definition if the node is called
# as a component without main.cpp that contains that definition
target_compile_definitions(
        ${LIB_NAME}
        PRIVATE
        -DROS2_COMPONENT
)

# Register an rclcpp component with the ament resource index and create an executable.
rclcpp_components_register_nodes(${LIB_NAME} "XdaInterface")

### Create the regular executable (non-component)
add_executable(${EXEC_NAME}
        src/main.cpp
        )

# Link the created shared library
target_link_libraries(${EXEC_NAME} ${LIB_NAME})

#############
## Install ##
#############

# install binaries
install(TARGETS ${EXEC_NAME} ${LIB_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )

# install launchfiles and configuration files
install(DIRECTORY launch param
        DESTINATION share/${PROJECT_NAME}
        )

# install header files
install(DIRECTORY include/
        DESTINATION include/
        )

##########
## Test ##
##########

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

#################
## Ament Cmake ##
#################

ament_export_include_directories(include)
ament_export_libraries(${LIB_NAME})
ament_export_dependencies(${DEPENDENCIES})
ament_package()
