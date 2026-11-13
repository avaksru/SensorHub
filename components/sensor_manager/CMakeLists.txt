idf_component_register(
    SRCS "sensor_manager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer sensor_abstraction app_config adc_shared onewire_shared bus_i2c adapter_esp_iot_bme280 adapter_esp_iot_aht20 adapter_esp_iot_sht3x adapter_scd4x adapter_sht4x adapter_esp_iot_ntc adapter_builtin_esp_temp adapter_builtin_adc adapter_builtin_pin adapter_builtin_gpio_in adapter_ds18b20 lua_runtime adapter_virtual
)
