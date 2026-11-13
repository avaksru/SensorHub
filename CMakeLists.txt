# Sensor Hub — ESP-IDF + esp-iot-solution (project: sensor_hub)
cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED ENV{IOT_SOLUTION_PATH})
    get_filename_component(IOT_SOLUTION_PATH "${CMAKE_SOURCE_DIR}/../../.." ABSOLUTE)
    set(ENV{IOT_SOLUTION_PATH} ${IOT_SOLUTION_PATH})
    message(WARNING "IOT_SOLUTION_PATH not set, using default $ENV{IOT_SOLUTION_PATH}")
endif()

set(EXTRA_COMPONENT_DIRS ${EXTRA_COMPONENT_DIRS}
    $ENV{IOT_SOLUTION_PATH}/components/i2c_bus
    $ENV{IOT_SOLUTION_PATH}/components/sensors/pressure/bme280
    $ENV{IOT_SOLUTION_PATH}/components/sensors/humiture/aht20
    $ENV{IOT_SOLUTION_PATH}/components/sensors/humiture/sht3x
    $ENV{IOT_SOLUTION_PATH}/components/sensors/ntc_driver
)
if(EXISTS "${CMAKE_SOURCE_DIR}/third_party/esp-idf-lib/components")
    list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_SOURCE_DIR}/third_party/esp-idf-lib/components")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sensor_hub)
