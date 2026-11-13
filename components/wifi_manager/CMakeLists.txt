idf_component_register(
    SRCS "wifi_manager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES app_config storage_nvs esp_wifi esp_netif esp_event esp_timer
)
