idf_component_register(
    SRCS "bus_i2c.cpp"
    INCLUDE_DIRS "include"
    REQUIRES app_config i2c_bus
)
