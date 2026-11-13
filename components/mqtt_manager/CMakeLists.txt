idf_component_register(
    SRCS "mqtt_manager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES app_config storage_nvs mqtt esp_event esp_wifi esp_netif
)
