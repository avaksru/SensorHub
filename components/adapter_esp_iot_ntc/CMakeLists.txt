idf_component_register(
    SRCS "adapter_esp_iot_ntc.cpp"
    INCLUDE_DIRS "include"
    REQUIRES sensor_abstraction app_config adc_shared ntc_driver esp_adc
)