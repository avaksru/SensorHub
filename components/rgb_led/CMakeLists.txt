idf_component_register(
    SRCS "rgb_led.cpp"
    INCLUDE_DIRS "include"
    REQUIRES soc ledc_shared esp_driver_gpio esp_driver_ledc esp_driver_rmt esp_timer freertos
)
