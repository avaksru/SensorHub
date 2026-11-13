idf_component_register(
    SRCS "adapter_builtin_esp_temp.cpp"
    INCLUDE_DIRS "include"
    REQUIRES sensor_abstraction app_config esp_driver_tsens
)