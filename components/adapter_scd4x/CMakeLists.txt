idf_component_register(
    SRCS "adapter_scd4x.cpp"
    INCLUDE_DIRS "include"
    REQUIRES sensor_abstraction app_config bus_i2c i2c_bus
)
