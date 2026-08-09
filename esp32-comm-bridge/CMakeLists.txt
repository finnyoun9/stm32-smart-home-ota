cmake_minimum_required(VERSION 3.16)

# Append the shared protocol sources
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../shared)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_comm_bridge)

# Create SPIFFS image from data/ directory if it exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/data")
    spiffs_create_partition_image(storage ${CMAKE_CURRENT_SOURCE_DIR}/data FLASH_IN_PROJECT)
endif()
