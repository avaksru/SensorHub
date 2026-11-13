idf_component_register(
    SRCS "adapter_builtin_adc.cpp"
    INCLUDE_DIRS "include"
    REQUIRES sensor_abstraction app_config adc_shared esp_adc
)