# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Include local components
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/esp32_s3)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_s3)
