idf_component_register(
    SRCS "adapter_ds18b20.cpp"
    INCLUDE_DIRS "include"
    REQUIRES sensor_abstraction app_config onewire_shared ds18b20 onewire_bus
)