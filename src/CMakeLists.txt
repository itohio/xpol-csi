file(GLOB_RECURSE app_sources ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "."
    REQUIRES
        esp_wifi
        nvs_flash
        esp_netif
        esp_event
        esp_http_server
        esp_timer
        spiffs
        driver
)
