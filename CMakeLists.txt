cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(xpol-csi)

# Bake the built web bundle (web/dist) into the `storage` SPIFFS partition so it
# flashes with the app. Run `npm --prefix web run build` first.
if(EXISTS "${CMAKE_SOURCE_DIR}/web/dist")
    spiffs_create_partition_image(storage "${CMAKE_SOURCE_DIR}/web/dist" FLASH_IN_PROJECT)
else()
    message(WARNING "web/dist not found - build the web bundle (npm --prefix web run build) before flashing the filesystem")
endif()
