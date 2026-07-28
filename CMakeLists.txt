cmake_minimum_required(VERSION 3.22)

set(ROLE "tx" CACHE STRING "Build role: tx or rx")

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/main_${ROLE}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mpu9250_driver)