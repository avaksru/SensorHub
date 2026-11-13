idf_component_register(
    SRCS "time_sync.cpp"
    INCLUDE_DIRS "include"
    REQUIRES app_config esp_event esp_netif
)
