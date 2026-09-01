# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# This branch is a deliberately small ESP32-S3 PCB signal generator.  Do not
# pull the production Wi-Fi/control stack into the board-test image.
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32s3_pcb_tester)
