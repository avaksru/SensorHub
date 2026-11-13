idf_component_register(
    SRCS "app_main.cpp"
    REQUIRES
        storage_littlefs storage_nvs app_config sensor_manager wifi_manager
        mqtt_manager http_api lua_runtime esp_http_server web_console time_sync
        virtual_state
)
