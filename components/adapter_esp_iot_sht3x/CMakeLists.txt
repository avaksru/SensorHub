idf_component_register(
    SRCS "adapter_esp_iot_sht3x.cpp"
    INCLUDE_DIRS "include"
    REQUIRES sensor_abstraction app_config bus_i2c sht3x
)
