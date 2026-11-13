idf_component_register(
    SRCS "adapter_esp_iot_bme280.cpp"
    INCLUDE_DIRS "include"
    REQUIRES sensor_abstraction app_config bus_i2c bme280
)
