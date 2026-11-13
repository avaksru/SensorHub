idf_component_register(
    SRCS "ledc_shared.cpp"
    INCLUDE_DIRS "include"
    REQUIRES soc esp_driver_ledc freertos
)
