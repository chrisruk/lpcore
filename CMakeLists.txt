# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(PROJECT_VER "0.02")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

if( $ENV{IDF_TARGET} STREQUAL "esp32s3")
    message(STATUS "THING = " $ENV{IDF_TARGET})
    list(PREPEND SDKCONFIG_DEFAULTS "sdkconfig.defaults.esp32s3")
elseif( $ENV{IDF_TARGET} STREQUAL "esp32c6")
    message(STATUS "THING = " $ENV{IDF_TARGET})
    list(PREPEND SDKCONFIG_DEFAULTS "sdkconfig.defaults.esp32c6")
endif()

project(Test)

idf_build_set_property(COMPILE_OPTIONS "-fdiagnostics-color=always" APPEND)
