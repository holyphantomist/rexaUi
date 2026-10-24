file(GLOB_RECURSE app_sources
    "${CMAKE_CURRENT_LIST_DIR}/*.c"
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
)

idf_component_register(
    SRCS ${app_sources}
    REQUIRES bt esp_adc esp_common esp_driver_gpio esp_driver_i2c esp_driver_ledc esp_driver_rmt esp_driver_sdmmc esp_driver_spi esp_event esp_http_client esp_hw_support esp_netif esp_pm esp_rom esp_system esp_timer esp_wifi fatfs freertos heap log lwip lua mbedtls nvs_flash sdmmc
)
