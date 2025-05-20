# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/RGB_Functions)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(LED_BLINK_LIBRERIAS)