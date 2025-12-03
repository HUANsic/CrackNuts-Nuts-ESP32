# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# 允许不指定IDF_TARGET，或在编译时指定
if(NOT IDF_TARGET)
    message(STATUS "NutsLib: Chip Unspecified, Using esp32s3 by default.")
    set(IDF_TARGET "esp32s3")
endif()

# 或者明确支持多个目标
if(IDF_TARGET STREQUAL "esp32s3")
    add_compile_definitions(IDF_TARGET_ESP32_S3)
    message(STATUS "NutsLib: Target Chip:ESP32-S3.")
elseif(IDF_TARGET STREQUAL "esp32c3")
    add_compile_definitions(IDF_TARGET_ESP32_C3)
    message(STATUS "NutsLib: Target Chip:ESP32-C3.")
else()
    message(FATAL_ERROR "NutsLib: Please Specify IDF_TARGET as esp32s3 or esp32c3.")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(UART)