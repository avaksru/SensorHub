idf_component_register(
    SRCS "lua_runtime.cpp" "adapter_lua.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        sensor_abstraction app_config gpio_map mqtt_manager storage_littlefs cjson soc lua
        esp_driver_gpio esp_driver_ledc esp_adc esp_netif esp_timer time_sync
        ledc_shared rgb_led virtual_state
)


