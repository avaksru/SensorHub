idf_component_register(
    SRCS "adapter_virtual.cpp"
    INCLUDE_DIRS "include"
    REQUIRES sensor_abstraction app_config virtual_state
)
