# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

if(DEFINED JOINT)
    add_compile_definitions(JOINT=${JOINT})
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(robot-actuator-firmware)
